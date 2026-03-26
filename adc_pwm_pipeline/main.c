/*
 * adc_pwm_pipeline
 *
 * Demonstrates a producer/consumer pipeline using FreeRTOS on the
 * Tiva C Series EK-TM4C123GH6PM LaunchPad. The ADC task samples a potentiometer
 * every 50ms and passes the value through a queue to the PWM task,
 * which controls LED brightness via Timer2 in PWM mode. A binary
 * semaphore signals the PWM task when new data is ready. A UART
 * logger task prints the current ADC value and brightness level
 * every 2 seconds, protected by a mutex.
 *
 * Tasks:       vADCPipelineTask, vPWMPipelineTask, vUARTPipelineTask
 * Peripherals: ADC0 (AIN0/PE3), Timer2A (PWM/PB0), UART0 (PA0/PA1)
 * FreeRTOS:    Queue, binary semaphore, mutex
 */

/* Standard includes. */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Hardware includes. */
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"

static void prvSetupHardware( void );

static void prvConfigureUART( void );

extern void vADCPipelineTask( void );
extern void vPWMPipelineTask( void );
extern void vUARTPipelineTask( void );

int main( void )
{
    /* Prepare the hardware to run this example. */
    prvSetupHardware();

    /* Create the ADC producer task, which initializes ADC0 and begins
     * sampling the potentiometer every 50ms.  Also creates the queue
     * and binary semaphore used by the pipeline. */
    vADCPipelineTask();

    /* Create the PWM consumer task, which initializes Timer2A in PWM
     * mode on PB0 and blocks until the ADC task signals new data. */
    vPWMPipelineTask();

    /* Create the UART logger task, which prints the current ADC value
     * and brightness level every 2 seconds, protected by a mutex. */
    vUARTPipelineTask();

    /* Start the scheduler.  If all is well this function will not
     * return.  If it does return then there was not enough FreeRTOS
     * heap memory available to create the idle or timer tasks. */
    vTaskStartScheduler();

    for( ;; );
}

static void prvSetupHardware( void )
{
    /* Run from the PLL at 80 MHz.  Any updates to the PLL rate here
     * would need to be reflected in FreeRTOSConfig.h by updating the
     * value of configCPU_CLOCK_HZ with the new system clock frequency. */
    MAP_SysCtlClockSet( SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
                        SYSCTL_OSC_INT | SYSCTL_XTAL_16MHZ );

    /* Configure device pins. */
    PinoutSet( false );

    /* Configure UART0 to send messages to terminal. */
    prvConfigureUART();
}

static void prvConfigureUART( void )
{
    /* Enable GPIO port A which is used for UART0 pins. */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOA );

    /* Configure the pin muxing for UART0 functions on port A0 and A1. */
    GPIOPinConfigure( GPIO_PA0_U0RX );
    GPIOPinConfigure( GPIO_PA1_U0TX );

    /* Enable UART0 so that we can configure the clock. */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_UART0 );

    /* Use the internal 16MHz oscillator as the UART clock source. */
    UARTClockSourceSet( UART0_BASE, UART_CLOCK_PIOSC );

    /* Select the alternate (UART) function for these pins. */
    GPIOPinTypeUART( GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1 );

    /* Initialize the UART for console I/O. */
    UARTStdioConfig( 0, 115200, 16000000 );
}

void vApplicationMallocFailedHook( void )
{
    /* vApplicationMallocFailedHook() will only be called if
     * configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.
     * It is called if a call to pvPortMalloc() fails because there
     * is insufficient free heap memory.  pvPortMalloc() is called
     * internally by the kernel whenever a task, queue, semaphore,
     * or mutex is created. */
    IntMasterDisable();
    for( ;; );
}

void vApplicationIdleHook( void )
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK
     * is set to 1 in FreeRTOSConfig.h.  It will be called on each
     * iteration of the idle task.  No blocking API calls should be
     * made from here. */
}

void vApplicationStackOverflowHook( TaskHandle_t pxTask,
                                    char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* vApplicationStackOverflowHook() will only be called if
     * configCHECK_FOR_STACK_OVERFLOW is set to 1 or 2 in
     * FreeRTOSConfig.h.  The stack of the offending task will
     * be corrupted at this point.  The task name and handle are
     * passed in for debug identification. */
    IntMasterDisable();
    for( ;; );
}

void *malloc( size_t xSize )
{
    /* There should not be a heap defined, so trap any attempts
     * to call malloc directly.  Always use pvPortMalloc()
     * instead so FreeRTOS manages the heap correctly. */
    IntMasterDisable();
    for( ;; );
}
