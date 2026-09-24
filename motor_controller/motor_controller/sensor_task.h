#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include "FreeRTOS.h"
#include "queue.h"

extern QueueHandle_t xSensorQueue;

void vSensorTask(void *pvParameters);

#endif // SENSOR_TASK_H
