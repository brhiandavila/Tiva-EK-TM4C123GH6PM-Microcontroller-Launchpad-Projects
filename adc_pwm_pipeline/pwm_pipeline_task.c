/*
 * pwm_pipeline_task.c
 *
 * Consumer side of the ADC/PWM pipeline.  Blocks on the binary
 * semaphore given by the ADC producer task.  When signaled, reads
 * the latest ADC value from the queue and scales it to a Timer2A
 * match value to control LED brightness on PB0.
 *
 * Peripherals: Timer2A in split pair PWM mode (PB0)
 * FreeRTOS:    Binary semaphore (xAdcSemaphore), Queue (xAdcQueue)
 */

/* Standard includes. */
#include <stdio.h>
#include <stdint.h>
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
#include "driverlib/timer.h"
#include "utils/uartstdio.h"

/*
 * The PWM frequency for LED brightness control.  1kHz is fast enough
 * that the human eye perceives smooth brightness rather than flicker.
 * At 80MHz this gives a load value of 80,000 counts.
 */
#define TIMER_2A_PWM_RATE   ( configCPU_CLOCK_HZ / 10000 )

/*
 * The queue handle owned by adc_pipeline_task.c.  Referenced here
 * to receive ADC samples from the producer task.
 */
extern QueueHandle_t xAdcQueue;

/*
 * The binary semaphore handle owned by adc_pipeline_task.c.
 * Referenced here to block until the ADC producer signals new data.
 */
extern SemaphoreHandle_t xAdcSemaphore;

/*
 * Public entry point called by main() to initialize Timer2A in PWM
 * mode and create the PWM consumer task.
 */
void vPWMPipelineTask( void );

/*
 * The PWM consumer task that blocks on the semaphore and updates
 * LED brightness based on the latest ADC value from the queue.
 */
static void prvPWMConsumerTask( void *pvParameters );

/*
 * Configures Timer2A in split pair PWM mode on PB0.
 */
static void prvConfigureTimer2PWM( void );

void vPWMPipelineTask( void )
{
    /* Configure Timer2A in split pair PWM mode on PB0. */
    prvConfigureTimer2PWM();

    /* Create the PWM consumer task.
     *
     * The xTaskCreate parameters in order are:
     *  - The function that implements the task.
     *  - The text name for the task - for debug only.
     *  - The size of the stack to allocate to the task.
     *  - No parameter passed to the task.
     *  - The priority assigned to the task.
     *  - The task handle is not required, so NULL is passed. */
    xTaskCreate( prvPWMConsumerTask,
                 "PWM Consumer",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 2,
                 NULL );
}

static void prvConfigureTimer2PWM( void )
{
    /* Enable the Timer2 peripheral. */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_TIMER2 );

    /* Enable GPIO Port B which contains PB0 (T2CCP0). */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOB );

    /* Wait for both peripherals to be ready. */
    while( !SysCtlPeripheralReady( SYSCTL_PERIPH_TIMER2 ) );
    while( !SysCtlPeripheralReady( SYSCTL_PERIPH_GPIOB ) );

    /* Configure the pin muxing to assign PB0 to Timer2A CCP0
     * function.  This tells the pin mux that PB0 is being used
     * as a timer output rather than a standard GPIO pin. */
    GPIOPinConfigure( GPIO_PB0_T2CCP0 );

    /* Configure PB0 as a timer output pin. */
    GPIOPinTypeTimer( GPIO_PORTB_BASE, GPIO_PIN_0 );

    /* Configure Timer2 in split pair mode with Timer2A set to
     * PWM output.  Split pair mode allows Timer2A and Timer2B
     * to run independently with different configurations. */
    TimerConfigure( TIMER2_BASE, TIMER_CFG_SPLIT_PAIR |
                                 TIMER_CFG_A_PWM );

    /* Set the Timer2A load value to define the PWM period.
     * At 80MHz with a load of 10,000 this gives exactly 1kHz. */
    TimerLoadSet( TIMER2_BASE, TIMER_A, TIMER_2A_PWM_RATE );

    /* Set an initial match value for 0% brightness until the
     * first ADC sample arrives.  Match = Load means the pin
     * never goes high — LED starts off. */
    TimerMatchSet( TIMER2_BASE, TIMER_A, TIMER_2A_PWM_RATE );

    /* Enable Timer2A to begin generating the PWM signal. */
    TimerEnable( TIMER2_BASE, TIMER_A );
}

static void prvPWMConsumerTask( void *pvParameters )
{
uint32_t ulADCValue;
uint32_t ulMatchValue;
uint32_t ulLoad;

    /* Store the load value once rather than recalculating every
     * iteration. */
    ulLoad = TIMER_2A_PWM_RATE;

    for( ;; )
    {
        /* Block here indefinitely until the ADC producer task
         * gives the semaphore signaling a new sample is ready.
         * This task consumes zero CPU time while waiting. */
        if( xSemaphoreTake( xAdcSemaphore, portMAX_DELAY ) == pdTRUE )
        {
            /* A new ADC sample is in the queue.  Read it without
             * removing it so the UART task can also read the same
             * value for logging purposes. */
            if( xQueuePeek( xAdcQueue,
                            &ulADCValue,
                            ( TickType_t ) 0 ) == pdTRUE )
            {
                /* Scale the 12-bit ADC value (0-4095) to a Timer2A
                 * match value (0-ulLoad).  The subtraction inverts
                 * the relationship so a higher ADC value produces
                 * a higher duty cycle and brighter LED. */
                ulMatchValue = ulLoad - ( ( ulADCValue * ulLoad ) / 4095UL );

                /* Update the Timer2A match register with the new
                 * duty cycle.  The PWM hardware applies this on
                 * the next cycle automatically. */
                TimerMatchSet( TIMER2_BASE, TIMER_A, ulMatchValue );
            }
        }
    }
}
