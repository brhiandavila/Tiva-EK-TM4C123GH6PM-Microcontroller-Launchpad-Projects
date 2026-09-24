#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "sensors.h"
#include "sensor_task.h"
#include "motor_task.h"
#include "cmd_task.h"
#include "drv8833.h"
#include "encoder.h"
#include "pid.h"
#include "uart_mutex.h"

#include "FreeRTOS.h"
#include "task.h"

#include "utils/uartstdio.h"

// 300 RPM / 60 = 5 revolutions per second
// 5 * 909 counts per revolution = 4,545 counts per second
// 4,545 * 0.010 seconds per window = ~45 counts per 10ms
#define TARGET_SPEED    45
#define OVERCURRENT_MA  1000.0f
#define OVERTEMP_C      80.0f

void vMotorTask(void *pvParameters){
    UART_Print("Motor task started\r\n"); // TEST

    float actualSpeed;
    float output;
    float pwm = 10.0f; // 10% duty cycle

    bool motorStarted = false;
    bool motorFaulted = false;

    SensorData_t xSensorData;

    for(;;){
        xQueuePeek(xSensorQueue, &xSensorData, 0);

        /* Fault check runs every cycle, even while already faulted --
         * this is a LATCHING fault: once tripped, the motor stays
         * stopped regardless of what the sensor reads next, until the
         * fault is explicitly cleared. This is deliberate: a reading
         * that momentarily dips back under threshold does not mean the
         * underlying problem is gone. */
        if(xSensorData.current > OVERCURRENT_MA){
            if(!motorFaulted)
                UART_Print("FAULT: overcurrent %d mA - MOTOR STOPPED (latched)\r\n",
                            (int32_t)xSensorData.current);
            motorFaulted = true;
        }

        if(xSensorData.temperature > OVERTEMP_C){
            if(!motorFaulted)
                UART_Print("FAULT: overtemp %d C - MOTOR STOPPED (latched)\r\n",
                           (int32_t)xSensorData.temperature);
            motorFaulted = true;
        }

        if(motorFaulted){
            DRV8833_SetMotor(0);
            motorStarted = false; // so PID re-inits cleanly if ever un-latched
            vTaskDelay(pdMS_TO_TICKS(10));
            continue; // skip PID entirely, so motor stays off
        }

        actualSpeed = Encoder_GetSpeed();

        if(!motorStarted && actualSpeed > 0){
            PID_Reset();
            motorStarted = true;
        }

        output = PID_Update(TARGET_SPEED, actualSpeed);
        pwm += output;

        if(pwm > 25)
            pwm = 25;
        if(pwm < 5)
            pwm = 5; // Never let PWM drop below 5%

        DRV8833_SetMotor((int32_t)pwm);
        if(!bLogPaused){
            UART_Print("Speed: %d | PWM Duty Cycle: %d\r\n", (int32_t)actualSpeed, (int32_t)pwm);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
