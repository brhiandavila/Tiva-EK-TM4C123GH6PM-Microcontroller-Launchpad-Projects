#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "sensors.h"
#include "sensor_task.h"
#include "motor_task.h"
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
#define TARGET_SPEED 45

void vMotorTask(void *pvParameters){
    UART_Print("Motor task started\r\n"); // TEST

    float actualSpeed;
    float output;
    float pwm = 10.0f; // 10% duty cycle
    bool motorStarted = false;

    Encoder_Init();
    PID_Init();

    SensorData_t xSensorData;

    for(;;){
        xQueueReceive(xSensorQueue, &xSensorData, 0);

        if(xSensorData.current > 1000.0f && xSensorData.current < 2000.0f){
            DRV8833_SetMotor(0);
            UART_Print("OVERCURRENT: %d mA - MOTOR STOPPED\r\n", (int32_t)xSensorData.current);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        if(xSensorData.temperature > 80.0f && xSensorData.temperature < 85.0f){
            DRV8833_SetMotor(0);
            UART_Print("OVER TEMP: %d C - MOTOR STOPPED\r\n", (int32_t)xSensorData.temperature);
            vTaskDelay(pdMS_TO_TICKS(1000));
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
        UART_Print("Speed: %d | PWM Duty Cycle: %d\r\n", (int32_t)actualSpeed, (int32_t)pwm);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
