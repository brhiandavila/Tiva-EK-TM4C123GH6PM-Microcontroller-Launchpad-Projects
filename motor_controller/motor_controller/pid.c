#include <stdint.h>
#include <stdbool.h>

#include "pid.h"

static PID_t pid;

#define D_FILTER_ALPHA 0.7f

void PID_Init(void){
    pid.kp = 0.001f;
    pid.ki = 0.00005f;
    pid.kd = 0.0002f;
    pid.integral = 0.0f;
    pid.prevError = 0.0f;
    pid.filteredDerivative = 0.0f;
}

float PID_Update(float targetSpeed, float actualSpeed){
    float error;
    float proportional;
    float integral;
    float rawDerivative;
    float derivative;

    error = targetSpeed - actualSpeed;
    proportional = (pid.kp * error);
    pid.integral += error;

    if(pid.integral > 1000)
        pid.integral = 1000;
    if(pid.integral < -1000)
        pid.integral = -1000;

    integral = (pid.ki * pid.integral);

    rawDerivative = (error - pid.prevError);
    pid.filteredDerivative = (D_FILTER_ALPHA * pid.filteredDerivative)
                            + ((1.0f - D_FILTER_ALPHA) * rawDerivative);

    derivative = pid.kd * pid.filteredDerivative;
    pid.prevError = error;

    return (proportional + integral + derivative);
}

void PID_Reset(void){
    pid.integral = 0.0f;
    pid.prevError = 0.0f;
    pid.filteredDerivative = 0.0f;
}
