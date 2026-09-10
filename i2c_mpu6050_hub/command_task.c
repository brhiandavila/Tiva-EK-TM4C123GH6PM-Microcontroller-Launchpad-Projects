/*
 * command_task.c
 *
 * Receives single character commands via UART0 RX interrupt.
 * UART0IntHandler notifies this task with the received byte using
 * a direct task notification — the task blocks on xTaskNotifyWait()
 * until a byte arrives, consuming no CPU while idle.
 * Supported commands:
 *   'P' or 'p' — pause sensor reading
 *   'R' or 'r' — resume sensor reading
 *   '?'        — print system status
 */

#include "inc/hw_memmap.h"
#include "command_task.h"
#include "uart_driver.h"
#include "driverlib/uart.h"
#include "sensor_task.h"
#include "FreeRTOS.h"
#include "task.h"

void vCommandTask(void *pvParameters)
{
    CommandTaskParams_t *pxParams = (CommandTaskParams_t *)pvParameters;
    char cCommand;

    vUARTPrint(pxParams->xUARTMutex,
               "\r\nCommands: P=pause R=resume ?=status\r\n");

    for (;;)
    {
        uint32_t ulNotifiedByte;
        xTaskNotifyWait(0x00,              /* don't clear any bits on entry */
                        0xFFFFFFFF,        /* clear all bits on exit */
                        &ulNotifiedByte,   /* the byte the ISR sent */
                        portMAX_DELAY);    /* block forever until notified */

        /* Poll for a character — block here until one arrives.
         * UARTCharGet blocks until a character is available.
         * This is safe because the command task has nothing else to do
         * while waiting for input. */
        cCommand = (char)ulNotifiedByte;

        switch(cCommand)
        {
            case 'P':
            case 'p':
                if(!bSensorPaused)
                {
                    bSensorPaused = true;
                    vUARTPrint(pxParams->xUARTMutex,
                               "[CMD] Sensor PAUSED\r\n");
                }
                else
                {
                    vUARTPrint(pxParams->xUARTMutex,
                               "[CMD] Already paused\r\n");
                }
                break;

            case 'R':
            case 'r':
                if(bSensorPaused)
                {
                    bSensorPaused = false;
                    vUARTPrint(pxParams->xUARTMutex,
                               "[CMD] Sensor RESUMED\r\n");
                }
                else
                {
                    vUARTPrint(pxParams->xUARTMutex,
                               "[CMD] Already running\r\n");
                }
                break;

            case '?':
                vUARTPrint(pxParams->xUARTMutex,
                           "[CMD] Status: %s\r\n",
                           bSensorPaused ? "PAUSED" : "RUNNING");
                break;

            default:
                vUARTPrint(pxParams->xUARTMutex,
                           "[CMD] Unknown: '%c'\r\n", cCommand);
                break;
        }
    }
}
