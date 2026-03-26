/*
 * adc_pipeline_task.c
 *
 * Producer side of the ADC/PWM pipeline.  Initializes ADC0 to sample
 * the potentiometer on PE3 (AIN0) every 50ms using sequencer 3.
 * Creates the queue and binary semaphore shared across the pipeline,
 * then signals the PWM task each time a new sample is ready.
 *
 * Peripherals: ADC0, Sequencer 3, AIN0 (PE3)
 * FreeRTOS:    Queue (xAdcQueue), Binary Semaphore (xAdcSemaphore)
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
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h"

/*
 * The number of items the ADC queue can hold.  A depth of 5 ensures
 * the producer never blocks even if the consumer is momentarily busy.
 */
#define ADC_QUEUE_DEPTH     ( 5 )

/*
 * The rate at which the ADC task samples the potentiometer.
 */
#define ADC_SAMPLE_RATE_MS  ( 50 )

/*
 * The queue handle shared across the pipeline.  Declared here and
 * referenced as extern in pwm_pipeline_task.c and uart_pipeline_task.c
 */
QueueHandle_t xAdcQueue = NULL;

/*
 * The binary semaphore handle shared between the ADC producer task
 * and the PWM consumer task.  Declared here, referenced as extern
 * in pwm_pipeline_task.c
 */
SemaphoreHandle_t xAdcSemaphore = NULL;

/*
 * Public entry point called by main() to initialize the ADC hardware,
 * create the queue and semaphore, and create the producer task.
 */
void vADCPipelineTask( void );

/*
 * The ADC producer task that samples the potentiometer every 50ms
 * and signals the PWM task via the binary semaphore.
 */
static void prvADCProducerTask( void *pvParameters );

/*
 * Configures ADC0 to sample AIN0 (PE3) using sequencer 3 with a
 * software trigger.
 */
static void prvConfigureADC( void );

void vADCPipelineTask( void )
{
    /* Configure ADC0 to sample the potentiometer on PE3. */
    prvConfigureADC();

    /* Create the queue to pass ADC samples to the PWM task.
     * This must be created before the PWM and UART tasks are
     * created in main, as both depend on this handle. */
    xAdcQueue = xQueueCreate( ADC_QUEUE_DEPTH, sizeof( uint32_t ) );

    /* Create the binary semaphore used to signal the PWM task
     * that a new ADC sample is ready. */
    xAdcSemaphore = xSemaphoreCreateBinary();

    /* Verify both were created successfully before proceeding. */
    if( ( xAdcQueue != NULL ) && ( xAdcSemaphore != NULL ) )
    {
        /* Create the ADC producer task.
         *
         * The xTaskCreate parameters in order are:
         *  - The function that implements the task.
         *  - The text name for the task - for debug only.
         *  - The size of the stack to allocate to the task.
         *  - No parameter passed to the task.
         *  - The priority assigned to the task.
         *  - The task handle is not required, so NULL is passed. */
        xTaskCreate( prvADCProducerTask,
                     "ADC Producer",
                     configMINIMAL_STACK_SIZE,
                     NULL,
                     tskIDLE_PRIORITY + 3,
                     NULL );
    }
}

static void prvConfigureADC( void )
{
    /* Enable the ADC0 peripheral. */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_ADC0 );

    /* Enable GPIO Port E which contains AIN0 (PE3). */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOE );

    /* Wait for both peripherals to be ready. */
    while( !SysCtlPeripheralReady( SYSCTL_PERIPH_ADC0 ) );
    while( !SysCtlPeripheralReady( SYSCTL_PERIPH_GPIOE ) );

    /* Configure PE3 as an ADC input pin. */
    GPIOPinTypeADC( GPIO_PORTE_BASE, GPIO_PIN_3 );

    /* Configure sequencer 3 on ADC0 to be triggered by the
     * processor and at the highest priority (0). */
    ADCSequenceConfigure( ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0 );

    /* Configure the single step in sequencer 3.
     *
     * The ADCSequenceStepConfigure parameters in order are:
     *  - ADC base:        ADC0
     *  - Sequencer:       3
     *  - Step:            0 (first and only step)
     *  - Channel/flags:   AIN0, interrupt enable, end of sequence */
    ADCSequenceStepConfigure( ADC0_BASE, 3, 0,
                              ADC_CTL_CH0 |
                              ADC_CTL_IE  |
                              ADC_CTL_END );

    /* Enable sequencer 3 on ADC0. */
    ADCSequenceEnable( ADC0_BASE, 3 );

    /* Clear any pending ADC interrupts before we begin. */
    ADCIntClear( ADC0_BASE, 3 );
}

static void prvADCProducerTask( void *pvParameters )
{
uint32_t ulADCValue;
TickType_t xLastWakeTime;

    /* Initialize xLastWakeTime with the current tick count.  This
     * variable is updated by vTaskDelayUntil() every 50ms to keep
     * the sampling rate consistent regardless of execution time. */
    xLastWakeTime = xTaskGetTickCount();

    for( ;; )
    {
        /* Block until exactly 50ms has passed since the last sample.
         * Unlike vTaskDelay() which delays 50ms from whenever it is
         * called, vTaskDelayUntil() guarantees a fixed 50ms period
         * even if the task was preempted or took time to execute. */
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( ADC_SAMPLE_RATE_MS ) );

        /* Trigger a software conversion on sequencer 3. */
        ADCProcessorTrigger( ADC0_BASE, 3 );

        /* Wait for the conversion to complete by polling the
         * interrupt status flag we enabled with ADC_CTL_IE. */
        while( !ADCIntStatus( ADC0_BASE, 3, false ) );

        /* Clear the interrupt flag now that we have detected it. */
        ADCIntClear( ADC0_BASE, 3 );

        /* Read the conversion result from the sequencer 3 FIFO
         * into our local variable. */
        ADCSequenceDataGet( ADC0_BASE, 3, &ulADCValue );

        /* Send the ADC value to the queue.  A block time of 0 means
         * do not block if the queue is full — matching the pattern
         * used in queue_example.  At a depth of 5 this should
         * never happen in normal operation. */
        xQueueOverwrite( xAdcQueue,
                       ( void * ) &ulADCValue);

        /* Give the binary semaphore to wake the PWM consumer task
         * and signal that a fresh ADC sample is in the queue. */
        xSemaphoreGive( xAdcSemaphore );
    }
}
