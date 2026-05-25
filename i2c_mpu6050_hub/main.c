/*
 * i2c_mpu6050_hub
 *
 * Project 6 -- MPU6050 over I2C with FreeRTOS
 * TM4C123GH6PM @ 80 MHz
 *
 * Architecture:
 *   Software Timer (100ms) -> Counting Semaphore -> vSensorReadTask
 *   vSensorReadTask -> Queue -> vSensorPrintTask
 *   UART polling -> vCommandTask
 *
 * Project 6 change vs Project 5:
 *   i2c_driver.c refactored from polling to interrupt driven.
 *   I2C transactions now block the calling task via binary semaphore
 *   instead of spinning on I2CMasterBusy(). The scheduler must be
 *   running before any I2C transaction can occur -- therefore sensor
 *   verification moves into vStartupTask instead of main().
 */

#include <stdint.h>
#include <stdbool.h>

#include "sensor_task.h"
#include "print_task.h"
#include "command_task.h"
#include "uart_driver.h"
#include "i2c_driver.h"
#include "mpu6050.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"

#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"

/*---------------------------------------------------------------------------
 * FreeRTOS handles
 *--------------------------------------------------------------------------*/
SemaphoreHandle_t xSensorSemaphore = NULL;
QueueHandle_t     xSensorQueue     = NULL;
SemaphoreHandle_t xUARTMutex       = NULL;
TimerHandle_t     xSensorTimer     = NULL;

static SensorTaskParams_t  xSensorParams;
static PrintTaskParams_t   xPrintParams;
static CommandTaskParams_t xCommandParams;

/*---------------------------------------------------------------------------
 * Forward declarations
 *--------------------------------------------------------------------------*/
static void prvSetupHardware(void);
static void prvInitProject(void);
static void vSensorTimerCallback(TimerHandle_t xTimer);
static void vStartupTask(void *pvParameters);

extern void vSensorReadTask(void *pvParameters);
extern void vSensorPrintTask(void *pvParameters);
extern void vCommandTask(void *pvParameters);

/*---------------------------------------------------------------------------
 * main
 *
 * Hardware init only. No sensor communication here.
 * The interrupt driven I2C driver requires xSemaphoreTake() which
 * requires the scheduler to be running. All sensor verification
 * happens in vStartupTask after vTaskStartScheduler().
 *--------------------------------------------------------------------------*/
int main(void)
{
    prvSetupHardware();
    prvInitProject();
    vTaskStartScheduler();
    for(;;);
}

/*---------------------------------------------------------------------------
 * prvSetupHardware
 *--------------------------------------------------------------------------*/
static void prvSetupHardware(void)
{
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
                       SYSCTL_OSC_INT    | SYSCTL_XTAL_16MHZ);

    PinoutSet(false);
    vUARTDriverInit();
    I2C_init();
}

/*---------------------------------------------------------------------------
 * prvInitProject
 *--------------------------------------------------------------------------*/
static void prvInitProject(void)
{
    /* Primitives */
    xUARTMutex = xSemaphoreCreateMutex();
    configASSERT(xUARTMutex != NULL);

    xSensorSemaphore = xSemaphoreCreateCounting(4, 0);
    configASSERT(xSensorSemaphore != NULL);

    xSensorQueue = xQueueCreate(5, sizeof(MPU6050_Data_t));
    configASSERT(xSensorQueue != NULL);

    /* Parameter structs */
    xSensorParams.xSemaphore = xSensorSemaphore;
    xSensorParams.xQueue     = xSensorQueue;
    xSensorParams.xUARTMutex = xUARTMutex;

    xPrintParams.xQueue     = xSensorQueue;
    xPrintParams.xUARTMutex = xUARTMutex;

    xCommandParams.xUARTMutex = xUARTMutex;

    /* Startup task -- priority 4, highest in the system.
     * Runs first after scheduler starts. Verifies sensor then
     * starts the timer and deletes itself. */
    xTaskCreate(vStartupTask,
                "Startup",
                configMINIMAL_STACK_SIZE * 2,
                NULL,
                4,
                NULL);

    /* Sensor pipeline tasks */
    xTaskCreate(vSensorReadTask,
                "SensorRead",
                configMINIMAL_STACK_SIZE * 2,
                &xSensorParams,
                3,
                NULL);

    xTaskCreate(vSensorPrintTask,
                "SensorPrint",
                configMINIMAL_STACK_SIZE * 2,
                &xPrintParams,
                2,
                NULL);

    TaskHandle_t xCommandTaskHandle = NULL;
    xTaskCreate(vCommandTask,
                "Command",
                configMINIMAL_STACK_SIZE * 2,
                &xCommandParams,
                1,
                &xCommandTaskHandle);
    configASSERT(xCommandTaskHandle != NULL);

    /* Timer created but NOT started here.
     * vStartupTask starts it after sensor is confirmed alive. */
    xSensorTimer = xTimerCreate("SensorTimer",
                                pdMS_TO_TICKS(100),
                                pdTRUE,
                                NULL,
                                vSensorTimerCallback);
    configASSERT(xSensorTimer != NULL);
}

/*---------------------------------------------------------------------------
 * vStartupTask
 *
 * Runs once after scheduler starts. Verifies MPU6050 is alive using
 * the interrupt driven I2C driver -- safe here because scheduler is
 * running and xSemaphoreTake() can block correctly.
 *
 * Starts the sensor timer only after hardware is confirmed working.
 * Deletes itself when done -- no resources wasted after startup.
 *--------------------------------------------------------------------------*/
static void vStartupTask(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(500));

    uint8_t whoAmI = 0;

    if (!MPU6050_whoAmI(&whoAmI) || whoAmI != 0x68)
    {
        UARTprintf("FATAL: MPU6050 not found. Halting.\r\n");
        for(;;);
    }
    UARTprintf("MPU6050 found at 0x%02X\r\n", whoAmI);

    if (!MPU6050_init())
    {
        UARTprintf("FATAL: MPU6050 init failed. Halting.\r\n");
        for(;;);
    }
    UARTprintf("MPU6050 awake. System starting...\r\n");

    xTimerStart(xSensorTimer, portMAX_DELAY);
    vTaskDelete(NULL);
}

/*---------------------------------------------------------------------------
 * vSensorTimerCallback
 *--------------------------------------------------------------------------*/
static void vSensorTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;
    xSemaphoreGive(xSensorSemaphore);
}

/*---------------------------------------------------------------------------
 * Hook functions
 *--------------------------------------------------------------------------*/
void vApplicationMallocFailedHook(void)
{
    IntMasterDisable();
    for(;;);
}

void vApplicationIdleHook(void)
{
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void)pcTaskName;
    (void)pxTask;
    IntMasterDisable();
    for(;;);
}

void *malloc(size_t xSize)
{
    IntMasterDisable();
    for(;;);
}
