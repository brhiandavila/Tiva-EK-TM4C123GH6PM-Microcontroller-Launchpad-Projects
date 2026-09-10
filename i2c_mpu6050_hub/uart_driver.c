/*
 * uart_driver.c
 *
 * UART0 — PA0 (RX), PA1 (TX), 115200 8-N-1
 * Thread-safe print via mutex and UART RX interrupt.
 * Commands received via UARTCharGet polling in vCommandTask.
 */

#include "uart_driver.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/rom_map.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Registered via vUARTSetCommandTask(); the ISR notifies this task
 * when a byte arrives. Distinct from the local variable of the same
 * name in main.c, which only exists to pass the handle here. */
static TaskHandle_t xCommandTaskHandle = NULL;

/*---------------------------------------------------------------------------
 * vUARTDriverInit
 *--------------------------------------------------------------------------*/
void vUARTDriverInit(void)
{
    /* Enable GPIOA — PA0 (RX) and PA1 (TX) live here */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    /* Enable UART0 peripheral clock */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    /* Wait for both peripherals to be ready */
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));

    /* Route PA0 and PA1 to UART0 alternate function */
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Use internal 16 MHz PIOSC as UART clock source.
     * This keeps UART baud rate independent of the PLL — important
     * because the UART baud divisor math works cleanly at 16 MHz. */
    MAP_UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    /* Initialize UART stdio at 115200 baud using 16 MHz clock */
    UARTStdioConfig(0, 115200, 16000000);

    /* Enable UART0 RX interrupt and RX timeout — RT is required so a single
     * byte sitting in the FIFO below the trigger level still gets delivered
     * once the line goes idle, rather than waiting for more bytes to arrive. */
    MAP_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    /* Priority must be numerically >= configMAX_SYSCALL_INTERRUPT_PRIORITY (0xA0),
     * same rule as I2C0 — this ISR calls xTaskNotifyFromISR. */
    MAP_IntPrioritySet(INT_UART0, 0xA0);
    MAP_IntEnable(INT_UART0);
}

/*---------------------------------------------------------------------------
 * vUARTPrint
 *
 * Takes the mutex, prints, gives the mutex back.
 *
 * PITFALL: Note the xSemaphoreGive is OUTSIDE the if block.
 *   If you put it inside the if, a failed mutex take means you skip
 *   the give — but you never took it so that is actually correct.
 *   The pattern here is intentional: only give if you successfully took.
 *--------------------------------------------------------------------------*/
void vUARTPrint(SemaphoreHandle_t xMutex, const char *pcFormat, ...)
{
    va_list vaArgs;

    if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    {
        va_start(vaArgs, pcFormat);
        UARTvprintf(pcFormat, vaArgs);
        va_end(vaArgs);

        xSemaphoreGive(xMutex);
    }
}

void vUARTSetCommandTask(TaskHandle_t xTaskToNotify)
{
    xCommandTaskHandle = xTaskToNotify;
}

void UART0IntHandler(void){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t ui32Status;

    /* Clear interrupt flag first */
    ui32Status = MAP_UARTIntStatus(UART0_BASE, true);
    MAP_UARTIntClear(UART0_BASE, ui32Status);

    /* Drain all available bytes — RX interrupt can fire with more
     * than one byte waiting in the FIFO */
    while(MAP_UARTCharsAvail(UART0_BASE))
    {
        uint8_t ucByte = (uint8_t)MAP_UARTCharGetNonBlocking(UART0_BASE);

        if(xCommandTaskHandle != NULL)
        {
            xTaskNotifyFromISR(xCommandTaskHandle,
                               (uint32_t)ucByte,
                               eSetValueWithOverwrite,
                               &xHigherPriorityTaskWoken);
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
