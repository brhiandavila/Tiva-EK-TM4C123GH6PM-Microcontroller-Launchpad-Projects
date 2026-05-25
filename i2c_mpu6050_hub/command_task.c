/*
 * command_task.c
 *
 * Polls UART0 for single character commands from a terminal.
 * UARTCharGet blocks until a character arrives — no ISR needed.
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
        /* Poll for a character — block here until one arrives.
         * UARTCharGet blocks until a character is available.
         * This is safe because the command task has nothing else to do
         * while waiting for input. */
        cCommand = (char)UARTCharGet(UART0_BASE);

        switch (cCommand)
        {
            case 'P':
            case 'p':
                if (!bSensorPaused)
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
                if (bSensorPaused)
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
