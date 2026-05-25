/*
 * print_task.h
 *
 * vSensorPrintTask — receives MPU6050_Data_t from queue,
 * prints formatted output to UART terminal.
 */

#ifndef PRINT_TASK_H
#define PRINT_TASK_H

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"

typedef struct {
    QueueHandle_t     xQueue;       /* Receives sensor data from here    */
    SemaphoreHandle_t xUARTMutex;   /* Protects UART during print        */
} PrintTaskParams_t;

void vSensorPrintTask(void *pvParameters);

#endif /* PRINT_TASK_H */
