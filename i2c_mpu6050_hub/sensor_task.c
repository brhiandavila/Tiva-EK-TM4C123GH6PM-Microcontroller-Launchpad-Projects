/*
 * sensor_task.c
 *
 * Waits on counting semaphore (given every 100ms by software timer),
 * reads all MPU6050 axes via burst read, sends struct to print queue.
 */

#include "sensor_task.h"
#include "mpu6050.h"
#include "uart_driver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

volatile bool bSensorPaused = false;

void vSensorReadTask(void *pvParameters)
{
    SensorTaskParams_t *pxParams = (SensorTaskParams_t *)pvParameters;
    MPU6050_Data_t xData;

    for (;;)
    {
        /* If paused, block for 50ms at a time instead of spinning.
         * This yields the CPU so command task can process R command.
         * We do NOT take the semaphore while paused — this prevents
         *   the counting semaphore from accumulating a backlog while
         *   the sensor is paused. When resumed, we start fresh. */
        if (bSensorPaused)
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        /* Only take semaphore when not paused */
        xSemaphoreTake(pxParams->xSemaphore, portMAX_DELAY);

        /* Read sensor and send to queue */
        xData.timestamp_ms = (uint32_t)(xTaskGetTickCount());

        if (MPU6050_readAll(&xData))
        {
            xQueueSend(pxParams->xQueue, &xData, 0);
        }
        else
        {
            vUARTPrint(pxParams->xUARTMutex,
                       "[SensorTask] MPU6050 read failed\r\n");
        }
    }
}
