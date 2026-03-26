/* Standard includes. */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

/* Hardware includes. */
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h"

/*
 * The mutex that protects the UART — defined in uart_task.c, shared here.
 */
extern SemaphoreHandle_t xMutex;

EventGroupHandle_t xEventGroup = NULL;

static uint32_t ui32PotValue = 0;
static uint32_t ui32TempRawValue = 0;

/*
 * The task that reads the potentiometer on PE3 via ADC0.
 */
static void prvPotentiometerTask( void *pvParameters );

static void prvTemperatureSensorTask( void *pvParameters );

static void prvProcessingTask( void *pvParameters );

/*
 * Configure ADC0 to sample the potentiometer on PE3 (channel AIN0).
 */
static void prvConfigureADC( void );

static void prvConfigureTempSensor( void );

void vADCTask( void )
{
    /* Configure ADC0 for potentiometer sampling. */
    prvConfigureADC();

    /* Configure ADC0 for temperature sampling */
    prvConfigureTempSensor();

    xEventGroup = xEventGroupCreate();

    if( xEventGroup == NULL ){
        for(;;);
    }

    /* Create the potentiometer task.
     *
     * xTaskCreate parameters in order:
     *  - Task function
     *  - Debug name
     *  - Stack size
     *  - No parameters passed
     *  - Priority
     *  - No handle needed */
    xTaskCreate( prvPotentiometerTask,
                 "Pot",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 NULL );

    xTaskCreate( prvTemperatureSensorTask,
                 "Temperature",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 NULL );

    xTaskCreate( prvProcessingTask,
                 "Processing",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 2,
                 NULL );
}

static void prvPotentiometerTask( void *pvParameters )
{
    uint32_t ui32ADCValue[1];

    for(;;){
        ADCProcessorTrigger(ADC0_BASE, 3);

        while(!ADCIntStatus(ADC0_BASE, 3, false));

        ADCIntClear(ADC0_BASE, 3);

        ADCSequenceDataGet(ADC0_BASE, 3, ui32ADCValue);

        ui32PotValue = ui32ADCValue[0];

        xEventGroupSetBits(xEventGroup, 0x01);

        vTaskDelay( pdMS_TO_TICKS( 500 ) );
    }
}

static void prvTemperatureSensorTask( void *pvParameters ){
    uint32_t ui32ADCValue[1];

    for(;;){
        ADCProcessorTrigger(ADC0_BASE, 2);

        while(!ADCIntStatus(ADC0_BASE, 2, false));

        ADCIntClear(ADC0_BASE, 2);

        ADCSequenceDataGet(ADC0_BASE, 2, ui32ADCValue);

        ui32TempRawValue = ui32ADCValue[0];

        xEventGroupSetBits(xEventGroup, 0x02);

        vTaskDelay( pdMS_TO_TICKS( 500 ) );
    }
}

static void prvProcessingTask( void *pvParameters ){
    float fTempC;
    int32_t i32TempWhole;
    int32_t i32TempFrac;

    for(;;){
        /* Block until both bit 0 (potentiometer) and bit 1 (temperature)
         * are set. Bits are cleared automatically when this unblocks. */
        xEventGroupWaitBits(
                xEventGroup,    /* The event group handle */
                0x03,           /* Wait for bit 0 AND bit 1 */
                pdTRUE,         /* Clear bits on exit */
                pdTRUE,         /* Wait for ALL bits */
                portMAX_DELAY); /* Block forever */

        fTempC = 147.5f - ((75.0f * 3.3f * (float)ui32TempRawValue) / 4096.0f);
        i32TempWhole = (int32_t)fTempC;
        i32TempFrac = (int32_t)((fTempC - i32TempWhole) * 10);

        if( xSemaphoreTake( xMutex, portMAX_DELAY ) == pdTRUE ){
            UARTprintf("Potentiometer: %d | Temperature: %d.%d C\n", ui32PotValue, i32TempWhole, i32TempFrac);
            xSemaphoreGive( xMutex );
        }
    }
}

static void prvConfigureADC( void )
{
    /* Enable the ADC0 peripheral. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    /* Enable GPIO port E which is used for AIN0 (PE3). */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    /* Configure PE3 as an ADC input pin. */
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    /* Configure ADC0 sequencer 3 (single sample, lowest priority).
     * Sequencer 3 holds only one sample — perfect for a single channel. */
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    /* Configure the single step in sequencer 3:
     *  - Channel 0 (AIN0 = PE3)
     *  - Set the interrupt flag on this step (ADC_CTL_IE)
     *  - Mark this as the last step in the sequence (ADC_CTL_END) */
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
                             ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);

    /* Enable sequencer 3. */
    ADCSequenceEnable(ADC0_BASE, 3);

    /* Clear any pending ADC interrupt. */
    ADCIntClear(ADC0_BASE, 3);
}

static void prvConfigureTempSensor( void ){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_TS | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 2);
    ADCIntClear(ADC0_BASE, 2);
}
