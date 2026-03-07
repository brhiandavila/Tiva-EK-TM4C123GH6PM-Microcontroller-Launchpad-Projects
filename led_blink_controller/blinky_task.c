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
 * This project demonstrates how to configure the TM4C123GH6PM to blink a LED
 * using FreeRTOS queue.  Two tasks and one queue are created for this example.
 * The sending task sends a parameter at an interval of 1000ms to the queue.
 * The receiving task is in blocked state until  something is received in the
 * queue.  Upon receiving a correct parameter from the queue it sets the LED
 * for 500ms and then clears the LED for another 500ms.
 *
 * When either user switch SW1 or SW2 on the EK-TM4C123GXL is pressed, an
 * interrupt is generated to change the blink rate.  The amount of increase
 * or decrease is controlled by the #define BLINK_RATE.  SW1 is pressed to
 * speed up the blink rate while SW2 is pressed to slow down the blink rate.
 *
 * vBlinkyTask() creates one queue, and two tasks.  It then starts the
 * scheduler.
 *
 * The Queue Send Task:
 * The queue send task is implemented by the prvQueueSendTask() function in
 * this file.  prvQueueSendTask() sits in a loop that causes it to repeatedly
 * block for 1000 milliseconds, before sending the value 100 to the queue that
 * was created within vBlinkyTask().  Once the value is sent, the task loops
 * back around to block for another 1000 milliseconds.
 *
 * The Queue Receive Task:
 * The queue receive task is implemented by the prvQueueReceiveTask() function
 * in this file.  prvQueueReceiveTask() sits in a loop where it repeatedly
 * blocks on attempts to read data from the queue that was created within
 * vBlinkyTask().  When data is received, the task checks the value of the
 * data, and if the value equals the expected 100, sets the LED.  The 'block
 * time' parameter passed to the queue receive function specifies that the
 * task should be held in the Blocked state indefinitely to wait for data to
 * be available on the queue.  The queue receive task will only leave the
 * Blocked state when the queue send task writes to the queue.  As the queue
 * send task writes to the queue every 1000 milliseconds, the queue receive
 * task leaves the Blocked state every 1000 milliseconds.  Once unblocked,
 * the LED is first set for 500ms and then clear for 500ms.
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

#define BLINK_RATE_FAST 500 // 500ms

#define BLINK_RATE_MEDIUM 1000 // 1000ms

#define BLINK_RATE_SLOW 2000 // 2000ms

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
void xButtonsHandler( void );
/*-----------------------------------------------------------*/

void vBlinkyTask( void )
{
    /* Configure a button to generate interrupts (for test purposes). */
    prvConfigureButton();

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
        vTaskDelay(pdMS_TO_TICKS(ui32BlinkRate));

        /* Turn off LED */
        LEDWrite(0x0F, 0);
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
        /* Notify the Button Task that a button was pressed */
        vTaskNotifyGiveFromISR(xButtonTaskHandle, &xHigherPriorityTaskWoken);

        /* Update time stamp */
        g_ui32TimeStamp = xTaskGetTickCount();
    }

    /* Perform a context switch if necessary*/
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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


