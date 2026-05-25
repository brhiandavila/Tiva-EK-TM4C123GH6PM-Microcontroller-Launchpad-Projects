#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct{
    float kp;        // proportional gain
    float ki;        // integral gain
    float kd;        // derivative gain;
    float integral;  // accumulated error
    float prevError; // previous error for derivative
} PID_t;

void PID_Init(void);
float PID_Update(float targetSpeed, float actualSpeed);
void PID_Reset(void);

#endif // PID_H
