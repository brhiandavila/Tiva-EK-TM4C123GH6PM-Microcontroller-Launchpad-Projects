#include "stm32l476xx.h"
#include "clock_config.h"
#include "systick.h"
#include "uart.h"
#include "can1.h"

int main(void)
{
    SystemClock_Config();
    SysTick_Init(80000000);
    UART2_Init();
    CAN1_Init();

    UART2_SendString("STM32 CAN peer started\r\n");

    uint32_t lastHeartbeat = 0;
    while (1)
    {
        if (millis() - lastHeartbeat >= 750)
        {
            CAN1_SendHeartbeat();
            lastHeartbeat = millis();
        }
    }
}
