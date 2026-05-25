/*
 * command_task.h
 *
 * vCommandTask — receives single character commands from UART RX ISR
 * via task notification. Handles sensor task suspend/resume.
 */

#ifndef COMMAND_TASK_H
#define COMMAND_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

typedef struct {
    SemaphoreHandle_t xUARTMutex;   /* For printing command responses     */
} CommandTaskParams_t;

void vCommandTask(void *pvParameters);

#endif /* COMMAND_TASK_H */
