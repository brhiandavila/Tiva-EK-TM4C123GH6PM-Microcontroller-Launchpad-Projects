/*
 * uart_driver.c
 *
 * UART0 — PA0 (RX), PA1 (TX), 115200 8-N-1
 * Thread-safe print via mutex.
 * Commands received via UARTCharGet polling in vCommandTask.
 */

#include "uart_driver.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/rom_map.h"
#include "utils/uartstdio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

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

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    {
        va_start(vaArgs, pcFormat);
        UARTvprintf(pcFormat, vaArgs);
        va_end(vaArgs);

        xSemaphoreGive(xMutex);
    }
}
