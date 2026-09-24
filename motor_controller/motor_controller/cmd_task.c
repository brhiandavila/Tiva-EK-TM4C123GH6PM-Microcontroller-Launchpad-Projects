#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#include "FreeRTOS.h"
#include "task.h"

#include "uart_mutex.h"
#include "cmd_task.h"

volatile bool bLogPaused = false;

static TaskHandle_t xCommandTaskHandle = NULL;

void CMD_UART0RxInit(void){
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    IntPrioritySet(INT_UART0, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntEnable(INT_UART0);
}

void UART0IntHandler(void){
    uint32_t ui32Status = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, ui32Status);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    while(UARTCharsAvail(UART0_BASE)){
        int32_t ch = UARTCharGetNonBlocking(UART0_BASE);

        if(xCommandTaskHandle != NULL){
            /* eSetValueWithOverwrite carries the byte itself as the
             * notification's value. Unlike vTaskNotifyGiveFromISR(),
             * which only increments a counter. This is what lets the
             * task read the actual received character back out below,
             * with no separate shared buffer needed between ISR and
             * task. */
            xTaskNotifyFromISR(xCommandTaskHandle,
                                (uint32_t)ch,
                                eSetValueWithOverwrite,
                                &xHigherPriorityTaskWoken);
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void vCommandTask(void *pvParameters){
    xCommandTaskHandle = xTaskGetCurrentTaskHandle();

    UART_Print("\r\nCommands: P=pause R=resume ?=status\r\n");

    for(;;){
        uint32_t ulNotifiedByte;
        xTaskNotifyWait(0x00, 0xFFFFFFFF, &ulNotifiedByte, portMAX_DELAY);

        char cCommand = (char)ulNotifiedByte;

        switch(cCommand){
            case 'P':
            case 'p':
                if(!bLogPaused){
                    bLogPaused = true;
                    UART_Print("[CMD] Log PAUSED\r\n");
                } else {
                    UART_Print("[CMD] Already paused\r\n");
                }
                break;

            case 'R':
            case 'r':
                if(bLogPaused){
                    bLogPaused = false;
                    UART_Print("[CMD] Log RESUMED\r\n");
                } else {
                    UART_Print("[CMD] Already running\r\n");
                }
                break;

            case '?':
                UART_Print("[CMD] Status: %s\r\n", bLogPaused ? "PAUSED" : "RUNNING");
                UART_Print("[CMD] Cmd task stack HWM (words free): %u\r\n",
                           uxTaskGetStackHighWaterMark(NULL));
                break;

            default:
                UART_Print("[CMD] Unknown: '%c'\r\n", cCommand);
                break;
        }
    }
}
