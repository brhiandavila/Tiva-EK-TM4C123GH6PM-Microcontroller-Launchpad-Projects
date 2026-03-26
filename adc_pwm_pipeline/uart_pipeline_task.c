/*
 * uart_pipeline_task.c
 *
 * Observer side of the ADC/PWM pipeline.  Wakes every 2 seconds and
 * reads the latest ADC value from the queue without removing it.
 * Calculates the brightness as a percentage and prints both values
 * to the UART terminal.  A mutex protects UARTprintf() from
 * concurrent access.
 *
 * Peripherals: UART0 (PA0/PA1)
 * FreeRTOS:    Mutex (xUartMutex), Queue (xAdcQueue)
 */

/* Standard includes. */
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Hardware includes. */
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

/*
 * The rate at which the UART logger task prints pipeline status.
 */
#define UART_LOG_RATE_MS    ( 2000 )

/*
 * The mutex handle that protects the UART resource from concurrent
 * access between tasks.  Owned here, used only within this file.
 */
static SemaphoreHandle_t xUartMutex = NULL;

/*
 * The queue handle owned by adc_pipeline_task.c.  Referenced here
 * to peek at the latest ADC sample for logging purposes.
 */
extern QueueHandle_t xAdcQueue;

/*
 * Public entry point called by main() to create the mutex and
 * register the UART logger task with the scheduler.
 */
void vUARTPipelineTask( void );

/*
 * The UART logger task that wakes every 2 seconds and prints the
 * current ADC value and brightness percentage to the terminal.
 */
static void prvUARTLoggerTask( void *pvParameters );

/*
 * Mutex guarded printf wrapper.  Identical pattern to uart_thread_safe
 * reference.  Accepts variable arguments via va_list.
 */
static void prvUARTPrintf( const char *pcString, ... );
/*-----------------------------------------------------------*/

void vUARTPipelineTask( void )
{
    /* Create the mutex to guard the UART resource before creating
     * the task that uses it.  Matches the pattern from uart_thread_safe
     * reference — mutex must exist before any task tries to take it. */
    xUartMutex = xSemaphoreCreateMutex();

    if( xUartMutex != NULL )
    {
        /* Create the UART logger task.
         *
         * The xTaskCreate parameters in order are:
         *  - The function that implements the task.
         *  - The text name for the task - for debug only.
         *  - The size of the stack to allocate to the task.
         *  - No parameter passed to the task.
         *  - The priority assigned to the task.
         *  - The task handle is not required, so NULL is passed. */
        xTaskCreate( prvUARTLoggerTask,
                     "UART Logger",
                     configMINIMAL_STACK_SIZE,
                     NULL,
                     tskIDLE_PRIORITY + 1,
                     NULL );
    }
}

static void prvUARTLoggerTask( void *pvParameters )
{
uint32_t ulADCValue;
uint32_t ulBrightness;

    for( ;; )
    {
        /* Block for 2 seconds before printing pipeline status.
         * Unlike the PWM task which blocks on a semaphore, this
         * task blocks on time — it is an observer on its own
         * schedule, independent of the ADC sampling rate. */
        vTaskDelay( pdMS_TO_TICKS( UART_LOG_RATE_MS ) );

        /* Peek at the latest ADC value without removing it from
         * the queue so the PWM task can continue reading it.
         * A block time of 0 means don't wait if the queue is
         * empty — this can happen on the very first iteration
         * before the ADC task has deposited its first sample. */
        if( xQueuePeek( xAdcQueue,
                        &ulADCValue,
                        ( TickType_t ) 0 ) == pdTRUE )
        {
            /* Calculate brightness as a percentage of the full
             * 12-bit ADC range for human readable output. */
            ulBrightness = ( ulADCValue * 100UL ) / 4095UL;

            /* Print the current pipeline status to the terminal
             * using the mutex guarded printf wrapper. */
            prvUARTPrintf( "[%u ms] ADC: %4u | Brightness: %3u%%\n",
                           ( unsigned ) xTaskGetTickCount(),
                           ulADCValue,
                           ulBrightness );
        }
    }
}

static void prvUARTPrintf( const char *pcString, ... )
{
va_list vaArgP;

    /* Attempt to take the mutex before accessing the UART resource.
     * Block for up to 100ms waiting for the mutex if it is not
     * immediately available.  This prevents the logger task from
     * blocking indefinitely if something goes wrong. */
    if( xSemaphoreTake( xUartMutex, pdMS_TO_TICKS( 100 ) ) == pdTRUE )
    {
        /* Start processing the variable argument list using the
         * format string as the reference point. */
        va_start( vaArgP, pcString );

        /* Pass both the format string and variable arguments to
         * UARTvprintf which handles the actual UART transmission. */
        UARTvprintf( pcString, vaArgP );

        /* Clean up the variable argument list now that we are
         * finished with it. */
        va_end( vaArgP );

        /* Return the mutex so other tasks can use the UART. */
        xSemaphoreGive( xUartMutex );
    }
}

void vApplicationTickHook( void )
{
    /* This function will be called by each tick interrupt if
     * configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code
     * can be added here, but the tick hook is called from an interrupt
     * context, so code must not attempt to block, and only the interrupt
     * safe FreeRTOS API functions can be used (those that end in
     * FromISR()). */
}
