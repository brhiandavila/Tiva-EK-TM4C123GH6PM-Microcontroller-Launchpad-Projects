/*
 * print_task.c
 *
 * Blocks on queue waiting for sensor data.
 * Prints all six axes and temperature when data arrives.
 */

#include "print_task.h"
#include "mpu6050.h"
#include "uart_driver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void vSensorPrintTask(void *pvParameters)
{
    PrintTaskParams_t *pxParams = (PrintTaskParams_t *)pvParameters;
    MPU6050_Data_t xData;

    for (;;)
    {
        /* Block until sensor task puts data in the queue.
         * portMAX_DELAY — wait forever, CPU free while blocked. */
        if (xQueueReceive(pxParams->xQueue, &xData, portMAX_DELAY) == pdTRUE)
        {
            /* Convert raw temp to Celsius then split into whole and decimal parts
             * because UARTprintf does not support %f */
            float fTempC = (xData.raw_temp / 340.0f) + 36.53f;
            int32_t iTempWhole   = (int32_t)fTempC;
            int32_t iTempDecimal = (int32_t)((fTempC - iTempWhole) * 10);
            if (iTempDecimal < 0) iTempDecimal = -iTempDecimal;

            vUARTPrint(pxParams->xUARTMutex,
                       "[%6ums] AX:%6d AY:%6d AZ:%6d | "
                       "GX:%6d GY:%6d GZ:%6d | T:%d.%dC\r\n",
                       (uint32_t)(xTaskGetTickCount()),
                       xData.accel_x, xData.accel_y, xData.accel_z,
                       xData.gyro_x,  xData.gyro_y,  xData.gyro_z,
                       iTempWhole, iTempDecimal);
        }
    }
}
