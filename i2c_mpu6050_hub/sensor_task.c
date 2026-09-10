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
         * Skip taking the semaphore while paused. This works correctly
         * because prvSensorTimerCallback() (main.c) also checks
         * bSensorPaused before giving — so no tokens accumulate on
         * either side during a pause. */
        if (bSensorPaused)
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        /* Only take semaphore when not paused */
        xSemaphoreTake(pxParams->xSemaphore, portMAX_DELAY);

        if (MPU6050_readAll(&xData))
        {
            xData.timestamp_ms = (uint32_t)(xTaskGetTickCount()); /* Read sensor and send to queue */
            xQueueSend(pxParams->xQueue, &xData, 0);
        }
        else
        {
            vUARTPrint(pxParams->xUARTMutex,
                       "[SensorTask] MPU6050 read failed\r\n");
        }
    }
}
