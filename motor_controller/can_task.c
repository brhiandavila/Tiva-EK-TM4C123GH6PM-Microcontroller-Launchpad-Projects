#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_can.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/can.h"
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
    CANEnable(CAN0_BASE);
}

void vCANTask(void *pvParameters){
    tCANMsgObject sCANMessage;
    uint8_t pui8MsgData[8];
    SensorData_t xSensorData;

    int32_t speed;
    int16_t i16Speed;
    int16_t i16Current;

    for(;;){
        xQueueReceive(xSensorQueue, &xSensorData, 0);
        UART_Print("CAN sees current: %d mA\n", (int32_t)xSensorData.current);

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

        UART_Print("CAN TX - Speed: %d Current: %d\r\n", (int32_t)speed, (int32_t)xSensorData.current);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
