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

/* Guards UART0 output; created in uart_task.c */
extern SemaphoreHandle_t xMutex;

/* Bits set by each producer task; prvProcessingTask blocks until both are
 * set, then both are cleared automatically. */
#define EVENT_BIT_POT   ( 1 << 0 )
#define EVENT_BIT_TEMP  ( 1 << 1 )
#define EVENT_BITS_ALL  ( EVENT_BIT_POT | EVENT_BIT_TEMP )

/* Debug pins (PC4-PC6) mirror the event flow for logic analyzer capture.
 * Not part of the sensor pipeline itself - purely for visually verifying
 * that the event group actually waits for both producers before the
 * processing task fires. */
#define DEBUG_PIN_POT_EVENT     GPIO_PIN_4  /* PC4 */
#define DEBUG_PIN_TEMP_EVENT    GPIO_PIN_5  /* PC5 */
#define DEBUG_PIN_PROCESSING    GPIO_PIN_6  /* PC6 */

EventGroupHandle_t xEventGroup = NULL;

static uint32_t ui32PotValue = 0;
static uint32_t ui32TempRawValue = 0;

static void prvPotentiometerTask( void *pvParameters );
static void prvTemperatureSensorTask( void *pvParameters );
static void prvProcessingTask( void *pvParameters );

static void prvConfigureADC( void );
static void prvConfigureTempSensor( void );
static void prvConfigureDebugPins( void );

void vADCTask( void )
{
    prvConfigureADC();
    prvConfigureTempSensor();
    prvConfigureDebugPins();

    xEventGroup = xEventGroupCreate();

    if( xEventGroup == NULL ){
        for(;;);
    }

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
/*-----------------------------------------------------------*/

static void prvPotentiometerTask( void *pvParameters )
{
    uint32_t ui32ADCValue[1];

    for(;;){
        ADCProcessorTrigger(ADC0_BASE, 3);
        while(!ADCIntStatus(ADC0_BASE, 3, false));
        ADCIntClear(ADC0_BASE, 3);
        ADCSequenceDataGet(ADC0_BASE, 3, ui32ADCValue);

        ui32PotValue = ui32ADCValue[0];

        GPIOPinWrite(GPIO_PORTC_BASE, DEBUG_PIN_POT_EVENT, DEBUG_PIN_POT_EVENT);
        xEventGroupSetBits(xEventGroup, EVENT_BIT_POT);
        GPIOPinWrite(GPIO_PORTC_BASE, DEBUG_PIN_POT_EVENT, 0);

        vTaskDelay(pdMS_TO_TICKS(500));
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

        GPIOPinWrite(GPIO_PORTC_BASE, DEBUG_PIN_TEMP_EVENT, DEBUG_PIN_TEMP_EVENT);
        xEventGroupSetBits(xEventGroup, EVENT_BIT_TEMP);
        GPIOPinWrite(GPIO_PORTC_BASE, DEBUG_PIN_TEMP_EVENT, 0);

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
                EVENT_BITS_ALL, /* Wait for bit 0 AND bit 1 */
                pdTRUE,         /* Clear bits on exit */
                pdTRUE,         /* Wait for ALL bits */
                portMAX_DELAY); /* Block forever */

        GPIOPinWrite(GPIO_PORTC_BASE, DEBUG_PIN_PROCESSING, DEBUG_PIN_PROCESSING);

        fTempC = 147.5f - ((75.0f * 3.3f * (float)ui32TempRawValue) / 4096.0f);
        i32TempWhole = (int32_t)fTempC;
        i32TempFrac = (int32_t)((fTempC - i32TempWhole) * 10);

        if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE){
            UARTprintf("Potentiometer: %d | Temperature: %d.%d C\n", ui32PotValue, i32TempWhole, i32TempFrac);
            xSemaphoreGive(xMutex);
        }

        GPIOPinWrite(GPIO_PORTC_BASE, DEBUG_PIN_PROCESSING, 0);
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

    /* Sequencer 3: single sample, lowest priority - sized for exactly one
     * channel, which is all the potentiometer needs. */
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

    /* Sequencer 2, internal temp sensor channel - no GPIO pin config
     * needed since this doesn't come from an external pin. */
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_TS | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 2);
    ADCIntClear(ADC0_BASE, 2);
}

static void prvConfigureDebugPins( void ){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));

    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE,
                          DEBUG_PIN_POT_EVENT | DEBUG_PIN_TEMP_EVENT |
                          DEBUG_PIN_PROCESSING);

    GPIOPinWrite(GPIO_PORTC_BASE,
                DEBUG_PIN_POT_EVENT | DEBUG_PIN_TEMP_EVENT | DEBUG_PIN_PROCESSING,
                0);
}
