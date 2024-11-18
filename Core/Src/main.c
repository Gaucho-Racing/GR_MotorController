/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "fdcan.h"
#include "hrtim.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define deadTime 640
#define U_TIMER LL_HRTIM_TIMER_A
#define V_TIMER LL_HRTIM_TIMER_E
#define W_TIMER LL_HRTIM_TIMER_F
#define U_current phase_current[0]
#define V_current phase_current[1]
#define W_current phase_current[2]
// Motor settings
#define USE_HALL_SENSOR 0 // 0 for encoder + FOC, 1 for hall sensor lookup
#define N_POLES 10.0f // number of poles
#define N_STEP_ENCODER 8192.0f // Encoder number of steps
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const uint8_t HallToStep[8] = {0, 2, 6, 1, 4, 3, 5, 0};
const int StepToPhase[7][3] = {
  {0,  0,  0},
  {1, -1,  0},
  {1,  0, -1},
  {0,  1, -1},
  {-1, 1,  0},
  {-1, 0,  1},
  {0, -1,  1}
};
uint16_t Encoder_os = 0; // encoder offset angle
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void writePwm(uint32_t timer, int32_t duty);

int16_t Read_ADC1_Channel(uint32_t channel);

void print_CANBus(char* text);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
char printBuffer[1024];
PID PID_I_d, PID_I_q;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  PID_setParams(&PID_I_d, 0.1f, 0.1f, 0.0f);
  PID_setParams(&PID_I_q, 0.1f, 10.0f, 0.0f);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_HRTIM1_Init();
  MX_ADC2_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_ADC3_Init();
  MX_USART3_UART_Init();
  MX_ADC4_Init();
  /* USER CODE BEGIN 2 */
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_15); // enable gate drivers

  /*
    HRTIM pin & comparator config
    Output 1: HG, start on comp2, stop on comp1, center align to 0
    Output 2: LG, start on comp4, stop on comp3, center aligh to 0
    Comp ranges: 1&3 0~65527, 2&4 24~65527
  */
  // turn off all phases
  writePwm(U_TIMER, 0);
  writePwm(V_TIMER, 0);
  writePwm(W_TIMER, 0);
  // enable phase timers
  LL_HRTIM_EnableOutput(HRTIM1, 
    LL_HRTIM_OUTPUT_TA1 | LL_HRTIM_OUTPUT_TA2 | 
    LL_HRTIM_OUTPUT_TE1 | LL_HRTIM_OUTPUT_TE2 | 
    LL_HRTIM_OUTPUT_TF1 | LL_HRTIM_OUTPUT_TF2);
  LL_HRTIM_TIM_CounterEnable(HRTIM1, 
    LL_HRTIM_TIMER_MASTER | LL_HRTIM_TIMER_A | LL_HRTIM_TIMER_E | LL_HRTIM_TIMER_F);
  
  // enable ADC1 (phase current sense)
  if (LL_ADC_IsEnabled(ADC1) == 0){
    LL_ADC_StartCalibration(ADC1, LL_ADC_DIFFERENTIAL_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0);
  }
  LL_ADC_Enable(ADC1);
  while (!LL_ADC_IsActiveFlag_ADRDY(ADC1));
  int32_t U_current_os = 0;
  int32_t V_current_os = 0;
  int32_t W_current_os = 0;

  // enable microsecond counter
  LL_TIM_EnableCounter(TIM2);

  if (!USE_HALL_SENSOR) {
    // RM44SI encoder setup (Emrax motor)
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_15); // data port input mode
  }

  // CANBus setup
  HAL_FDCAN_Start(&hfdcan2);

  uint32_t micros = TIM2->CNT;
  uint32_t lastMicros = 0;
  float dt = (micros - lastMicros) * 1e-6f;
  int32_t dutyCycle = 0;
  uint8_t HallSig;
  float CCTRL_p, CCTRL_i = 0;
  float phase_current[3];
  float AC_current;
  float Target_current = 2.0f;
  uint32_t count = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    micros = TIM2->CNT;
    dt = (micros - lastMicros) * 1e-6f;

    // read phase currents from ADC1
    U_current = (float)(Read_ADC1_Channel(LL_ADC_CHANNEL_1) - U_current_os) * 0.0625f;
    V_current = (float)(Read_ADC1_Channel(LL_ADC_CHANNEL_8) - V_current_os) * 0.0625f;
    W_current = (float)(Read_ADC1_Channel(LL_ADC_CHANNEL_6) - W_current_os) * 0.0625f;

    // drive motor with hall sensor
    if (USE_HALL_SENSOR) {
      AC_current = sqrtf(((U_current * U_current) + (V_current * V_current) + (W_current * W_current)));
      CCTRL_p = Target_current - AC_current;
      CCTRL_i += CCTRL_p * (micros - lastMicros) * dt;
      CCTRL_i = fmaxf(fminf(CCTRL_i, 1), 0);
      dutyCycle = (fmaxf(fminf(CCTRL_p*0.01f + CCTRL_i, 1), 0)) * 32000.0f;
      HallSig = GPIOC->IDR >> 13;
      HallSig = ((HallSig & 0b001) << 2) + (HallSig & 0b010) + ((HallSig & 0b100) >> 2);
      writePwm(U_TIMER, StepToPhase[HallToStep[HallSig]][0] * dutyCycle);
      writePwm(V_TIMER, StepToPhase[HallToStep[HallSig]][1] * dutyCycle);
      writePwm(W_TIMER, StepToPhase[HallToStep[HallSig]][2] * dutyCycle);
    }
    // drive motor with encoder and FOC
    else {
      // read physical rotor position
      uint16_t phys_position;
      float elec_position;
      uint8_t TxBuffer[2];
      uint8_t RxBuffer[2];
      HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)TxBuffer, (uint8_t *)RxBuffer, 1, 5000);
      memcpy(&phys_position, RxBuffer, 2);
      phys_position = (phys_position & 0x7FFF) >> 2;
      phys_position += Encoder_os;
      elec_position = fmodf(phys_position / N_STEP_ENCODER * N_POLES, 1.0f) * M_PI * 2.0f; // radians
      float sin_elec_position = sinf(elec_position);
      float cos_elec_position = cosf(elec_position);
      
      // FOC
      // Clarke transform
      float I_a = U_current * 0.66666667f - V_current * 0.33333333f - W_current * 0.33333333f;
      float I_b = 0.5773502691896257f * (V_current - W_current);
      // Park transform
      float I_d = I_a * cos_elec_position + I_b * sin_elec_position;
      float I_q = I_b * cos_elec_position - I_a * sin_elec_position;
      AC_current = I_q;
      // PI controllers on Q and D
      float cmd_d = 0;//PID_update(&PID_I_d, I_d, 0.0f, dt);
      float cmd_q = PID_update(&PID_I_q, I_q, Target_current, dt);
      // Inverse Park transform
      float cmd_a = cmd_d * cos_elec_position - cmd_q * sin_elec_position;
      float cmd_b = cmd_q * cos_elec_position + cmd_d * sin_elec_position;
      // Inverse Clarke transform
      int16_t duty_u = cmd_a * -32000;
      int16_t duty_v = (cmd_a * -0.5f + 0.8660254037844386f * cmd_b) * -32000;
      int16_t duty_w = (cmd_a * -0.5f - 0.8660254037844386f * cmd_b) * -32000;
      // Update duty cycle
      writePwm(U_TIMER, duty_u);
      writePwm(V_TIMER, duty_v);
      writePwm(W_TIMER, duty_w);
      // debug print
      // sprintf(printBuffer, "phys_pos:%04u, elec_pos:%5.03f, I_q:%+5.03f, I_d:%+5.03f, cmd_d:%+5.03f, cmd_q:%+5.03f, U:%+05d, V:%+05d, W:%+05d, dt:%.05f\n", 
      //   phys_position, elec_position, I_q, I_d, cmd_d, cmd_q, duty_u, duty_v, duty_w, dt);
      // sprintf(printBuffer, "I_q:%+5.03f, I_d:%+5.03f, cmd_d:%+5.03f, cmd_q:%+5.03f, dt:%.05f\n", 
      //   I_q, I_d, cmd_d, cmd_q, dt);
      // print_CANBus(printBuffer);
    }

    // calibrate current sensor offset
    if (count == 0) {
      LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);
      writePwm(U_TIMER, 0);
      writePwm(V_TIMER, 0);
      writePwm(W_TIMER, 0);
      LL_mDelay(2);
      U_current_os = 0;
      V_current_os = 0;
      W_current_os = 0;
      for (uint16_t i = 0; i < 1024; i++) {
        U_current_os += Read_ADC1_Channel(LL_ADC_CHANNEL_1);
        V_current_os += Read_ADC1_Channel(LL_ADC_CHANNEL_8);
        W_current_os += Read_ADC1_Channel(LL_ADC_CHANNEL_6);
      }
      U_current_os = U_current_os >> 10;
      V_current_os = V_current_os >> 10;
      W_current_os = W_current_os >> 10;
      LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12);
    }
    count++;
    if (count > 100000) count = 0;
    lastMicros = micros;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSI_Enable();
   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 20, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Insure 1us transition state at intermediate medium speed clock*/
  for (__IO uint32_t i = (170 >> 1); i !=0; i--);

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(160000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void writePwm(uint32_t timer, int32_t duty) {
  int16_t duty_abs = (duty < 0) ? -duty : duty;
  if (duty_abs < deadTime) {
    duty_abs = 0;
  }
  else if (duty_abs > 32000 - deadTime) {
    duty_abs = 32000;
  }
  if (duty >= 0) {
    LL_HRTIM_TIM_SetCompare2(HRTIM1, timer, 64001 - duty_abs);
    LL_HRTIM_TIM_SetCompare1(HRTIM1, timer, duty_abs);
    LL_HRTIM_TIM_SetCompare4(HRTIM1, timer, 64001);
    LL_HRTIM_TIM_SetCompare3(HRTIM1, timer, 0);
  }
  else {
    LL_HRTIM_TIM_SetCompare2(HRTIM1, timer, 64001);
    LL_HRTIM_TIM_SetCompare1(HRTIM1, timer, 0);
    LL_HRTIM_TIM_SetCompare4(HRTIM1, timer, 64001 - duty_abs);
    LL_HRTIM_TIM_SetCompare3(HRTIM1, timer, duty_abs);
  }
}

int16_t Read_ADC1_Channel(uint32_t channel)
{
  // Set the ADC channel
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, channel);

  // Start ADC conversion
  LL_ADC_REG_StartConversion(ADC1);

  // Wait until conversion is complete
  while (LL_ADC_IsActiveFlag_EOC(ADC1) == 0);
  
  // Get ADC conversion result
  int16_t adc_value = LL_ADC_REG_ReadConversionData12(ADC1);
  
  // Clear the EOC flag
  LL_ADC_ClearFlag_EOC(ADC1);
  
  return adc_value;
}

void print_CANBus(char* text) {
  TxHeader.Identifier = 0x3FF;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
  uint32_t i = 0;
  while (text[i] != '\0') {
    TxData[i & 0b111U] = text[i];
    if ((i & 0b111U) == 0b111U) {
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
      while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0);
    }
    i++;
  }
  if (i & 0b111U) {
    for (uint8_t j = i & 0b111U; j < 8; j++) {
      TxData[j] = 0;
    }
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
