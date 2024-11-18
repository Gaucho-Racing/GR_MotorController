/*
   PID.h - library for implementing a PID controller
   Created by Yandong Liu, 20240108
*/

#include "PID.h"
#include <math.h>

float PID_update(PID* self, float current, float target, float dt) {
  float thisp = target - current;
  self->d = (thisp - self->p) / dt;
  self->p = thisp;
  self->i += self->p * dt;
  self->i = fminf(fmaxf(self->i, self->_iMin), self->_iMax);
  self->out = self->p * self->_kp + self->i + self->d * self->_kd;
  return fminf(fmaxf(self->out, -1.0f), 1.0f);
}

void PID_setParams(PID* self, float kp, float ki, float kd) {
  self->_kp = kp;
  self->_ki = ki;
  self->_kd = kd;
  self->_iMin = -1.0f;
  self->_iMax = 1.0f;
}
