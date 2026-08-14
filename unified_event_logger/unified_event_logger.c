/* Standard Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/* Hardware Includes */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"

#define EVENT_SW1   0x01
#define EVENT_SW2   0x02
#define EVENT_TIMER 0x03

#define EVENT_QUEUE_LENGTH      5
#define STATUS_TIMER_PERIOD_MS  5000
#define LED_PULSE_MS            200

/*
 * xEvent_t - the struct passed through the event queue.
 *
 * Every event source (ISR, software timer) fills one of these
 * and enqueues it. The handler task reads it and acts on
 * ucEventType.
 *
 * ucEventType  : identifies what happened (EVENT_SW1/SW2/TIMER)
 * ulTimestamp  : FreeRTOS tick count at the time of the event
 * ulCount      : running count of this specific event type
 */
typedef struct
{
    uint8_t ucEventType;
    uint32_t ulTimestamp;
    uint32_t ulCount;
} xEvent_t;

static QueueHandle_t xEventQueue = NULL;
static SemaphoreHandle_t xUARTMutex = NULL;

static TimerHandle_t xStatusTimer = NULL;

static uint32_t g_ulSW1Count = 0;
static uint32_t g_ulSW2Count = 0;
static uint32_t g_ulTimerCount = 0;

static volatile TickType_t g_xLastButtonTick = 0;

void vUnifiedEventLoggerTask( void );

static void prvEventHandlerTask( void *pvParameters );

static void prvStatusTimerCallback( TimerHandle_t xTimer );

static void prvConfigureButtons( void );
static void prvConfigureDebugPins( void );
void xButtonsHandler( void );

void vApplicationTickHook( void );

void vUnifiedEventLoggerTask( void )
{
    prvConfigureButtons();
    prvConfigureDebugPins();

    xEventQueue = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(xEvent_t));

    if(xEventQueue == NULL)
    {
        for( ;; );
    }

    xUARTMutex = xSemaphoreCreateMutex();

    if(xUARTMutex == NULL)
    {
        for( ;; );
    }

    xStatusTimer = xTimerCreate( "StatusTimer",
                                 pdMS_TO_TICKS(STATUS_TIMER_PERIOD_MS),
                                 pdTRUE,
                                 (void*)0,
                                 prvStatusTimerCallback);

    if(xStatusTimer != NULL)
    {
        xTimerStart(xStatusTimer, 0);
    }

    xTaskCreate(prvEventHandlerTask,
                "EventHandler",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);

    UARTprintf("Unified Event Logger Started.\n"
               "Press S1 or S2, or wait 5s for the timer.\n");
}

static void prvConfigureButtons( void )
{
    ButtonsInit();

    GPIOIntRegister(BUTTONS_GPIO_BASE, xButtonsHandler);

    GPIOIntTypeSet(BUTTONS_GPIO_BASE, ALL_BUTTONS, GPIO_FALLING_EDGE);

    GPIOIntEnable(BUTTONS_GPIO_BASE, ALL_BUTTONS);

    IntEnable(INT_GPIOF);

    IntMasterEnable();
}

static void prvConfigureDebugPins( void )
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));

    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE,
                           GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    /* Start all debug pins low */
    GPIOPinWrite(GPIO_PORTC_BASE,
                 GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0);
}

void xButtonsHandler( void )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t ui32Status;
    xEvent_t xEvent;

    ui32Status = GPIOIntStatus(BUTTONS_GPIO_BASE, true);
    GPIOIntClear(BUTTONS_GPIO_BASE, ui32Status);

    TickType_t xNow = xTaskGetTickCountFromISR();
    if((xNow - g_xLastButtonTick) < pdMS_TO_TICKS(200))
    {
        return;
    }
    g_xLastButtonTick = xNow;

    xEvent.ulTimestamp = xNow;

    xEvent.ulCount = 0;

    if(ui32Status & LEFT_BUTTON)
    {
        xEvent.ucEventType = EVENT_SW1;
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4); // PC4 high - SW1
    }
    else if(ui32Status & RIGHT_BUTTON)
    {
        xEvent.ucEventType = EVENT_SW2;
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5); // PC5 high - SW2
    }
    else
    {
        return;
    }

    xQueueSendFromISR(xEventQueue, &xEvent, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void prvStatusTimerCallback(TimerHandle_t xTimer)
{
    xEvent_t xEvent;

    xEvent.ucEventType = EVENT_TIMER;
    xEvent.ulTimestamp = xTaskGetTickCount();

    xEvent.ulCount = 0;

    xQueueSend(xEventQueue, &xEvent, 0);
}

static void prvEventHandlerTask( void *pvParameters )
{
    /*
     * This task is the heart of the project.
     *
     * It blocks indefinitely on xEventQueue. When a struct
     * arrives it:
     *   1. Increments the appropriate counter.
     *   2. Pulses the corresponding LED.
     *   3. Prints a message to UART protected by the mutex.
     */
    xEvent_t xEvent;

    while( 1 )
    {
        /*
         * Block here until an event struct is available.
         * portMAX_DELAY = wait forever. The task consumes no
         * CPU while waiting, just like prvQueueReceiveTask1
         * in queue_example.
         */
        if( xQueueReceive( xEventQueue, &xEvent, portMAX_DELAY ) == pdPASS )
        {
            switch( xEvent.ucEventType )
            {
                case EVENT_SW1:
                g_ulSW1Count++;

                LEDWrite( 0x07, RED_LED );
                GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6); // PC6 high - LED active
                vTaskDelay( pdMS_TO_TICKS( LED_PULSE_MS ) );
                LEDWrite( 0x07, 0 );
                GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0); // PC6 low - LED off

                /* Print to UART protected by mutex. */
                if( xSemaphoreTake( xUARTMutex, portMAX_DELAY ) == pdTRUE )
                {
                    UARTprintf( "SW1 pressed  - Count: %u, Tick: %u\n",
                                g_ulSW1Count,
                                xEvent.ulTimestamp );
                    xSemaphoreGive( xUARTMutex );
                }
                GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0); // clear SW1 debug pin now
                break;

                case EVENT_SW2:
                g_ulSW2Count++;

                LEDWrite( 0x07, BLUE_LED );
                GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6); // PC6 high - LED active
                vTaskDelay( pdMS_TO_TICKS( LED_PULSE_MS ) );
                LEDWrite( 0x07, 0 );
                GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0); // PC6 low - LED off

                /* Print to UART protected by mutex. */
                if( xSemaphoreTake( xUARTMutex, portMAX_DELAY ) == pdTRUE )
                {
                    UARTprintf( "SW2 pressed  - Count: %u, Tick: %u\n",
                                g_ulSW2Count,
                                xEvent.ulTimestamp );
                    xSemaphoreGive( xUARTMutex );
                }
                GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0); // clear SW1 debug pin now
                break;

                case EVENT_TIMER:
                g_ulTimerCount++;
                GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7); // PC7 high - timer event detected
                LEDWrite( 0x07, GREEN_LED );
                GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6); // PC6 high - LED active
                vTaskDelay( pdMS_TO_TICKS( LED_PULSE_MS ) );
                LEDWrite( 0x07, 0 );
                GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0); // PC6 low - LED off
                GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0); // PC7 low

                /* Print to UART protected by mutex. */
                if( xSemaphoreTake( xUARTMutex, portMAX_DELAY ) == pdTRUE )
                {
                    UARTprintf( "Timer fired  - Count: %u, Tick: %u\n",
                                g_ulTimerCount,
                                xEvent.ulTimestamp );
                    xSemaphoreGive( xUARTMutex );
                }
                break;

                default:
                /* Unknown event type - ignore. */
                break;
            }
        }
    }
}

void vApplicationTickHook( void ){}
