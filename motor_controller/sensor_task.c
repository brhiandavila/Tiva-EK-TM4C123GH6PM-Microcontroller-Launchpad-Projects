#include <i2c_helper.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "sensor_task.h"
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
        float current = INA260_ReadCurrent();
        if(current < 2000.0f && current > -500.0f)
            xSensorData.current = current;

        float voltage = INA260_ReadVoltage();
        if(voltage < 10000.0f && voltage > 0.0f)
            xSensorData.voltage = voltage;

        float power = INA260_ReadPower();
        if(power < 20000.0f && power > 0.0f)
            xSensorData.power = power;

        float temp = TMP119_ReadTemp();
        if(temp > -10.0f && temp < 85.0f)
            xSensorData.temperature = temp;

        xQueueSend(xSensorQueue, &xSensorData, 0);
        UART_Print("STATUS - I: %d mA  V: %d mV  T: %d C\n",
            (int32_t)xSensorData.current,
            (int32_t)xSensorData.voltage,
            (int32_t)xSensorData.temperature);
        vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}
