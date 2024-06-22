from math import *

# Settings
V_in = 600
D = 0.5
I_out = 100
f_sw = 40e3
D_ripple = 0.01
T_a = 30

# Electrical info
Q_g = 476e-9
V_f = 3.6
V_g = 25
R_on = 8.9e-3
E_on = 1.54e-3
E_off = 0.49e-3
E_recov = 0.53e-3

# Thermal info
T_Jmax = 150
R_thJC = 0.2

V_ripple = V_in * D_ripple
I_ripple = I_out / 2
C_in = (1 / f_sw) * I_ripple / V_ripple 

P_sw = f_sw * (E_on + E_off + E_recov)
P_cond = D * I_out**2 * R_on + (1 - D) * I_out * V_f
E_drv = V_g * Q_g
P_drv = f_sw * E_drv
P_fet = P_sw + P_cond

T_Cmax = T_Jmax - R_thJC * P_fet
R_thCA = (T_Cmax - T_a) / P_fet

T_Cmax = round(T_Cmax, 1)
R_thCA = round(R_thCA, 3)
P_sw = round(P_sw, 3)
P_cond = round(P_cond, 3)
P_fet = round(P_fet, 3)

print("Switching loss: " + str(P_sw) + "W")
print("Conduction loss: " + str(P_cond) + "W")
print("Total: " + str(P_fet) + "W")
print()
print("Gate drive loss: " + str(P_drv) + "W")
print()
print("Input cap: " + str(round(C_in * 1e6)) + "uF")
print()
print("Max allowed case temperature: " + str(T_Cmax) + "℃")
print("Max allowed heatsink resistance: " + str(R_thCA) + "℃/W")