/*
 * sensor_task.h
 *
 * vSensorReadTask — waits on counting semaphore, reads MPU6050,
 * sends data struct to queue.
 */

#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <stdbool.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"

/* Global suspend flag — set by command task, checked by sensor task
 * Declared volatile because two tasks access it */
extern volatile bool bSensorPaused;

/*---------------------------------------------------------------------------
 * Task parameter struct
 * Received via pvParameters at task entry.
 * Contains exactly what this task needs — nothing more.
 *--------------------------------------------------------------------------*/
typedef struct {
    SemaphoreHandle_t xSemaphore;   /* Counting semaphore from timer     */
    QueueHandle_t     xQueue;       /* Queue to send sensor data into     */
    SemaphoreHandle_t xUARTMutex;   /* For error printing only            */
} SensorTaskParams_t;

/*---------------------------------------------------------------------------
 * vSensorReadTask
 * Entry point. Pass SensorTaskParams_t* as pvParameters at xTaskCreate.
 *--------------------------------------------------------------------------*/
void vSensorReadTask(void *pvParameters);

#endif /* SENSOR_TASK_H */
