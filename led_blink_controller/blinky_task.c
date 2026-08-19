/*
 * blinky_task
 *
 * Copyright (C) 2022 Texas Instruments Incorporated
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

/******************************************************************************
 *
 * blinky_task
 *
 * Blinks the onboard LED on the TM4C123GH6PM at a user-adjustable rate.
 * SW1 cycles the blink rate through three fixed speeds (fast/medium/slow);
 * SW2 is wired the same way but currently drives the same cycle as SW1.
 *
 * A debounced GPIO interrupt notifies prvButtonTask directly via a FreeRTOS
 * task notification. prvButtonTask advances to the next rate and pushes it
 * into a 1-deep queue. prvLEDBlinkTask non-blockingly checks that queue each
 * time it comes back around its own blink delay, so a button press is
 * picked up on the LED's next toggle rather than waiting on the queue and
 * stalling until the current blink phase finishes.
 *
 * A third, independent task (prvUARTStatusTask) reports the current blink
 * rate over UART every 3 seconds.
 *
 * vBlinkyTask() creates the queue and all three tasks, then returns to
 * main() to start the scheduler.
 *
 */

/* Standard includes. */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Hardware includes. */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
/*-----------------------------------------------------------*/

/* Blink rates depending on button task*/
#define BLINK_RATE_FAST     500  // 500ms
#define BLINK_RATE_MEDIUM   1000 // 1000ms
#define BLINK_RATE_SLOW     2000 // 2000ms

/* Debug pins */
#define DEBUG_SW1_PULSE    GPIO_PIN_4
#define DEBUG_SW2_PULSE    GPIO_PIN_5
#define DEBUG_LED_STATE    GPIO_PIN_6

static QueueHandle_t xBlinkRateQueue = NULL;
static TaskHandle_t xButtonTaskHandle = NULL;
static uint32_t g_ui32CurrentRate = BLINK_RATE_MEDIUM;

/*
 * Time stamp global variable.
 */
volatile uint32_t g_ui32TimeStamp = 0;

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvButtonTask( void *pvParameters );
static void prvLEDBlinkTask( void *pvParameters );
static void prvUARTStatusTask( void *pvParameters );

/*
 * Called by main() to create the simple blinky style application.
 */
void vBlinkyTask( void );

/*
 * Hardware configuration for the buttons SW1 and SW2 to generate interrupts.
 */
static void prvConfigureButton( void );
static void prvConfigureDebugPins( void );
void xButtonsHandler( void );
/*-----------------------------------------------------------*/

void vBlinkyTask( void )
{
    /* Configure a button to generate interrupts (for test purposes). */
    prvConfigureButton();
    prvConfigureDebugPins();

    /* This is the queue to pass blink rates */
    xBlinkRateQueue = xQueueCreate(1, sizeof(uint32_t));

    /* Create the LED Blink Task */
    xTaskCreate(prvLEDBlinkTask,
                "LED",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);

    /* Create the Button Task and store the handle */
    xTaskCreate(prvButtonTask,
                "Button",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                &xButtonTaskHandle);

    /* Create the UART status task */
    xTaskCreate(prvUARTStatusTask,
                "UART",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);
}
/*-----------------------------------------------------------*/

static void prvButtonTask( void *pvParameters )
{
    uint32_t ui32NewRate = BLINK_RATE_MEDIUM;

    while(1){
        /* Wait for notification from button  */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /*Cycle through each blink rate*/
        if(ui32NewRate == BLINK_RATE_MEDIUM)
        {
            ui32NewRate = BLINK_RATE_FAST;
        }
        else if(ui32NewRate == BLINK_RATE_FAST)
        {
            ui32NewRate = BLINK_RATE_SLOW;
        }
        else
        {
            ui32NewRate = BLINK_RATE_MEDIUM;
        }

        /* Send new rate to queue */
        xQueueSend(xBlinkRateQueue, &ui32NewRate, 0);

        /* Update global variable for UART reporting*/
        g_ui32CurrentRate = ui32NewRate;
    }
}
/*-----------------------------------------------------------*/

static void prvLEDBlinkTask( void *pvParameters )
{
    uint32_t ui32BlinkRate = BLINK_RATE_MEDIUM;

    while(1){
        /* Check if there's a new blink rate in the queue */
        if(xQueueReceive(xBlinkRateQueue, &ui32BlinkRate, 0) == pdPASS)
        {
            /* New rate is now stored in ui32BlinkRate variable*/
        }

        /* Toggle the LED */
        LEDWrite(0x0F, RED_LED);
        GPIOPinWrite(GPIO_PORTC_BASE, DEBUG_LED_STATE, DEBUG_LED_STATE);
        vTaskDelay(pdMS_TO_TICKS(ui32BlinkRate));

        /* Turn off LED */
        LEDWrite(0x0F, 0);
        GPIOPinWrite(GPIO_PORTC_BASE, DEBUG_LED_STATE, 0);
        vTaskDelay(pdMS_TO_TICKS(ui32BlinkRate));
    }
}
/*-----------------------------------------------------------*/

static void prvUARTStatusTask( void *pvParameters )
{
    while(1)
    {
        /* Print the current blink rate */
        UARTprintf("Current Blink Rate: %d ms\n", g_ui32CurrentRate);

        /* Wait 3 seconds before next status update */
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

static void prvConfigureButton( void )
{
    /* Initialize the LaunchPad Buttons. */
    ButtonsInit();

    /* Register the interrupt handler */
    GPIOIntRegister(BUTTONS_GPIO_BASE, xButtonsHandler);

    /* Configure both switches to trigger an interrupt on a falling edge. */
    GPIOIntTypeSet(BUTTONS_GPIO_BASE, ALL_BUTTONS, GPIO_FALLING_EDGE);

    /* Enable the interrupt for Port F in the GPIO peripheral. */
    GPIOIntEnable(BUTTONS_GPIO_BASE, ALL_BUTTONS);

    /* Enable the Port F interrupt in the NVIC. */
    IntEnable(INT_GPIOF);

    /* Enable global interrupts in the NVIC. */
    IntMasterEnable();
}
/*-----------------------------------------------------------*/

void xButtonsHandler( void )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t ui32Status;

    /* Read the buttons interrupt status to find the cause of the interrupt. */
    ui32Status = GPIOIntStatus(BUTTONS_GPIO_BASE, true);

    /* Clear the interrupt. */
    GPIOIntClear(BUTTONS_GPIO_BASE, ui32Status);

    /* Debounce the input with 200ms filter */
    if ((xTaskGetTickCount() - g_ui32TimeStamp ) > 200)
    {
        if (ui32Status & LEFT_BUTTON) {
            GPIOPinWrite(GPIO_PORTC_BASE, DEBUG_SW1_PULSE, DEBUG_SW1_PULSE);
            GPIOPinWrite(GPIO_PORTC_BASE, DEBUG_SW1_PULSE, 0);
        }

        if (ui32Status & RIGHT_BUTTON) {
            GPIOPinWrite(GPIO_PORTC_BASE, DEBUG_SW2_PULSE, DEBUG_SW2_PULSE);
            GPIOPinWrite(GPIO_PORTC_BASE, DEBUG_SW2_PULSE, 0);
        }

        /* Notify the Button Task that a button was pressed */
        vTaskNotifyGiveFromISR(xButtonTaskHandle, &xHigherPriorityTaskWoken);

        /* Update time stamp */
        g_ui32TimeStamp = xTaskGetTickCount();
    }

    /* Perform a context switch if necessary*/
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/*-----------------------------------------------------------*/

static void prvConfigureDebugPins( void ){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));

    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, (DEBUG_SW1_PULSE | DEBUG_SW2_PULSE | DEBUG_LED_STATE));

    GPIOPinWrite(GPIO_PORTC_BASE, (DEBUG_SW1_PULSE | DEBUG_SW2_PULSE | DEBUG_LED_STATE), 0);
}
/*-----------------------------------------------------------*/
void vApplicationTickHook( void )
{
    /* This function will be called by each tick interrupt if
        configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
        added here, but the tick hook is called from an interrupt context, so
        code must not attempt to block, and only the interrupt safe FreeRTOS API
        functions can be used (those that end in FromISR()). */

    /* Only the full demo uses the tick hook so there is no code is
        executed here. */
}


