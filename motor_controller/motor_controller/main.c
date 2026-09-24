/******************************************************************************
 *
 * This project implements a closed-loop DC motor speed controller on the
 * TM4C123GH6PM using the FreeRTOS kernel.
 *
 * Motor speed is measured via a quadrature encoder (QEI0, velocity-capture
 * mode) and regulated by a PI(D) control loop against a fixed target,
 * driving a DRV8833 motor driver through hardware PWM (PWM0, Generator 2).
 *
 * An INA260 current/voltage/power monitor and TMP117 temperature sensor are
 * sampled over a fully interrupt-driven I2C0 master driver. The motor
 * latches into a safe stopped state if either reading exceeds a configured
 * overcurrent or overtemperature threshold.
 *
 * Speed and current telemetry is broadcast over CAN0 (500kbit/s) to a
 * second, independent node, which also transmits its own periodic
 * heartbeat frame back onto the same bus.
 *
 * main() brings up all hardware peripherals before starting the scheduler,
 * then creates four tasks: sensor acquisition, motor control, CAN
 * communication, and a UART command interface (P/R/? to pause, resume, or
 * query routine status output).
 *
 * This example uses UARTprintf for output of UART messages. UARTprintf is
 * not a thread-safe API, all UART output in this project is routed through
 * a mutex-protected wrapper (UART_Print()) rather than called directly.
 *
 * Open a terminal with 115,200 8-N-1 to see the output for this demo.
 *
 */

/* Standard includes. */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "sensor_task.h"
#include "motor_task.h"
#include "drv8833.h"
#include "sensors.h"
#include "can_task.h"
#include "cmd_task.h"
#include "uart_mutex.h"
#include "i2c_driver.h"
#include "encoder.h"
#include "pid.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

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
/*-----------------------------------------------------------*/

/* Set up the clock and pin configurations to run this example. */
static void prvSetupHardware( void );

/* This function sets up UART0 to be used for a console to display information
 * as the example is running. */
static void prvConfigureUART(void);

/*-----------------------------------------------------------*/
QueueHandle_t xSensorQueue = NULL;
SemaphoreHandle_t xUARTMutex;

int main(void)
{
    prvSetupHardware();

    I2C_Init();
    CAN_Init();
    DRV8833_Init();
    CMD_UART0RxInit();
    Encoder_Init();
    PID_Init();

    xUARTMutex = xSemaphoreCreateMutex();
    if(xUARTMutex == NULL)
        for(;;);

    UART_Print("main started\r\n");

    xSensorQueue = xQueueCreate(1, sizeof(SensorData_t));
    if(xSensorQueue == NULL || xUARTMutex == NULL)
        for(;;);

    xTaskCreate(vSensorTask,
                "Sensor",
                1024,
                NULL,
                2,
                NULL);

    xTaskCreate(vMotorTask,
                "Motor",
                512,
                NULL,
                3,
                NULL);

    xTaskCreate(vCANTask,
                "CAN",
                256,
                NULL,
                1,
                NULL);

    xTaskCreate(vCommandTask,
                "Command",
                256,
                NULL,
                1,
                NULL);

    vTaskStartScheduler();
}

/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
    /* Run from the PLL at 80 MHz.  Any updates to the PLL rate here would
     * need to be reflected in FreeRTOSConfig.h by updating the value of
     * configCPU_CLOCK_HZ with the new system clock frequency. */
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_INT |
                       SYSCTL_XTAL_16MHZ);

    /* Configure UART0 to send messages to terminal. */
    prvConfigureUART();

    /* Configure device pins. */
    PinoutSet(false);
}
/*-----------------------------------------------------------*/

void prvConfigureUART(void)
{
    /* Enable GPIO port A which is used for UART0 pins.
     * TODO: change this to whichever GPIO port you are using. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    /* Wait until hardware peripheral is ready. */
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    /* Configure the pin muxing for UART0 functions on port A0 and A1.
     * This step is not necessary if your part does not support pin muxing.
     * TODO: change this to select the port/pin you are using. */
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    /* Enable UART0 so that we can configure the clock. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    /* Wait until hardware peripheral is ready. */
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));

    /* Select the alternate (UART) function for these pins.
     * TODO: change this to select the port/pin you are using. */
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Use the internal 16MHz oscillator as the UART clock source. */
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    /* Initialize the UART for console I/O. */
    UARTStdioConfig(0, 115200, 16000000);
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    IntMasterDisable();
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
    task.  It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()).  If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    IntMasterDisable();
    for( ;; );
}
/*-----------------------------------------------------------*/

void *malloc( size_t xSize )
{
    /* There should not be a heap defined, so trap any attempts to call
    malloc. */
    IntMasterDisable();
    for( ;; );
}
/*-----------------------------------------------------------*/


