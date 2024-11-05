/*
 * PID.h - library for implementing a PID controller
 * Created by Yandong Liu, 20240108
 */

#ifndef PID_H
#define PID_H

typedef struct {
    float _kp, _ki, _kd;
    float _iMax, _iMin;
    float p, i, d, out;
} PID;

float PID_update(PID* self, float current, float target, float dt);
void PID_setParams(PID* self, float kp, float ki, float kd);

#endif
