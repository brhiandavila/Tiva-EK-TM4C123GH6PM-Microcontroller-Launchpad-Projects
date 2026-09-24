#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "sensors.h"
#include "sensor_task.h"
#include "encoder.h"
#include "uart_mutex.h"

#include "utils/uartstdio.h"
#include "can_task.h"
#include "cmd_task.h"

#define TELEMETRY_MSG_OBJ   1
#define TELEMETRY_ID        0x001
#define HEARTBEAT_MSG_OBJ   2
#define HEARTBEAT_ID        0x002

static TaskHandle_t xCANTaskHandle = NULL;

void CAN_Init(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0));

    GPIOPinConfigure(GPIO_PB4_CAN0RX);
    GPIOPinConfigure(GPIO_PB5_CAN0TX);
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    CANInit(CAN0_BASE);
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 500000);

    /* Set up message object 2 as a dedicated RX object, filtered to
     * accept ONLY the heartbeat ID (0x002) -- ID+mask filtering means
     * anything else on the bus (including our own TX echoes, if the
     * peripheral were in loopback) is rejected in hardware before the
     * CPU is ever interrupted. */
    tCANMsgObject sCANRxMessage;

    sCANRxMessage.ui32MsgID = HEARTBEAT_ID;
    sCANRxMessage.ui32MsgIDMask = 0x7FF;      /* exact-match mask, standard 11-bit ID */
    sCANRxMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    sCANRxMessage.ui32MsgLen = 8;             /* max DLC; actual length read back per-message */

    CANMessageSet(CAN0_BASE, HEARTBEAT_MSG_OBJ, &sCANRxMessage, MSG_OBJ_TYPE_RX);

    CANIntRegister(CAN0_BASE, CAN0IntHandler);
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR);
    IntEnable(INT_CAN0);

    CANEnable(CAN0_BASE);
}

void CAN0IntHandler(void){
    uint32_t ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    if(ui32Status == HEARTBEAT_MSG_OBJ){
        tCANMsgObject sCANRxMessage;
        uint8_t pui8MsgData[8];
        sCANRxMessage.pui8MsgData = pui8MsgData;

        /* Reading the message object clears its NEWDAT/pending flag.
         * Required or this interrupt fires again immediately. */
        CANMessageGet(CAN0_BASE, HEARTBEAT_MSG_OBJ, &sCANRxMessage, true);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if(xCANTaskHandle != NULL){
            vTaskNotifyGiveFromISR(xCANTaskHandle, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else if(ui32Status == TELEMETRY_MSG_OBJ){
        uint32_t ui32CANStatus = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        if(ui32CANStatus & CAN_STATUS_BUS_OFF){
            UART_Print("CAN ERROR: bus-off condition detected\r\n");
        }

        CANIntClear(CAN0_BASE, TELEMETRY_MSG_OBJ);
    }
    else{
        /* Bus error or unexpected status. Cleared here rather than
         * left pending, but not otherwise handled. A future revision
         * could report this over UART for visibility. */
        CANIntClear(CAN0_BASE, ui32Status);
    }
}

void vCANTask(void *pvParameters){
    tCANMsgObject sCANMessage;
    uint8_t pui8MsgData[8];
    SensorData_t xSensorData;

    int32_t speed;
    int16_t i16Speed;
    int16_t i16Current;

    xCANTaskHandle = xTaskGetCurrentTaskHandle();

    for(;;){
        /* Non-blocking check for a heartbeat notification. Consistent
         * with the "check in passing, never stall" pattern used in
         * LED Blink Controller's queue read. This task's real job is
         * the 100ms telemetry cycle below, heartbeat receipt is just
         * observed opportunistically each loop. */
        if(ulTaskNotifyTake(pdTRUE, 0) > 0){
            UART_Print("CAN RX - Heartbeat received from STM32 peer\r\n");
        }

        xQueuePeek(xSensorQueue, &xSensorData, 0);
        if(!bLogPaused){
            UART_Print("CAN sees current: %d mA\n", (int32_t)xSensorData.current);
        }

        speed = Encoder_GetSpeed();

        i16Speed   = (int16_t)speed;
        i16Current = (int16_t)xSensorData.current;

        pui8MsgData[0] = (i16Speed >> 8) & 0xFF;
        pui8MsgData[1] = i16Speed & 0xFF;
        pui8MsgData[2] = (i16Current >> 8) & 0xFF;
        pui8MsgData[3] = i16Current & 0xFF;

        sCANMessage.ui32MsgID = 0x001;
        sCANMessage.ui32MsgIDMask = 0;
        sCANMessage.ui32Flags = 0;
        sCANMessage.ui32MsgLen = 4;
        sCANMessage.pui8MsgData = pui8MsgData;

        CANMessageSet(CAN0_BASE, 1, &sCANMessage, MSG_OBJ_TYPE_TX);

        if(!bLogPaused){
            UART_Print("CAN TX - Speed: %d Current: %d\r\n", (int32_t)speed, (int32_t)xSensorData.current);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
