#include <i2c_driver.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "sensor_task.h"
#include "cmd_task.h"
#include "sensors.h"
#include "uart_mutex.h"

#include "FreeRTOS.h"
#include "task.h"

#include "utils/uartstdio.h"

#define SENSOR_PERIOD_MS 100

void vSensorTask(void *pvParameters){
    UART_Print("Sensor task started\r\n"); // TEST

    SensorData_t xSensorData;

    xSensorData.temperature = 25.0f;  // safe default
    xSensorData.current = 0.0f;
    xSensorData.voltage = 0.0f;
    xSensorData.power = 0.0f;

    for(;;){
        float reading;

        /* Each read is checked in two stages: did the I2C transaction
         * itself succeed, and if so, is the value physically plausible.
         * Either failure means xSensorData keeps its last-known-good
         * value for that field, rather than being overwritten with
         * garbage or a value from a fault condition. */

        if(INA260_ReadCurrent(&reading)){
            if(reading < 2000.0f && reading > -500.0f)
                xSensorData.current = reading;
            else
                UART_Print("WARN: current reading %d mA out of plausible range\r\n", (int32_t)reading);
        } else {
            UART_Print("WARN: INA260 current read failed (I2C error)\r\n");
        }

        if(INA260_ReadVoltage(&reading)){
            if(reading < 10000.0f && reading > 0.0f)
                xSensorData.voltage = reading;
            else
                UART_Print("WARN: voltage reading %d mV out of plausible range\r\n", (int32_t)reading);
        } else {
            UART_Print("WARN: INA260 voltage read failed (I2C error)\r\n");
        }

        if(INA260_ReadPower(&reading)){
            if(reading < 20000.0f && reading > 0.0f)
                xSensorData.power = reading;
            else
                UART_Print("WARN: power reading %d mW out of plausible range\r\n", (int32_t)reading);
        } else {
            UART_Print("WARN: INA260 power read failed (I2C error)\r\n");
        }

        if(TMP117_ReadTemp(&reading)){
            if(reading > -10.0f && reading < 85.0f)
                xSensorData.temperature = reading;
            else
                UART_Print("WARN: temp reading %d C out of plausible range\r\n", (int32_t)reading);
        } else {
            UART_Print("WARN: TMP117 temp read failed (I2C error)\r\n");
        }

        xQueueOverwrite(xSensorQueue, &xSensorData);
        if(!bLogPaused){
            UART_Print("STATUS - I: %d mA  V: %d mV  T: %d C\n",
                (int32_t)xSensorData.current,
                (int32_t)xSensorData.voltage,
                (int32_t)xSensorData.temperature);
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}
