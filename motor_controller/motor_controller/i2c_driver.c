#include <i2c_driver.h>
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "utils/uartstdio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "uart_mutex.h"


typedef enum{
    I2C_STATE_IDLE = 0,
    I2C_STATE_READ_SWITCH,
    I2C_STATE_READ_SINGLE,
    I2C_STATE_READ_CONT,
    I2C_STATE_READ_FINISH,
    I2C_STATE_WRITE_DATA,
    I2C_STATE_WRITE_FINISH
} I2C_State_t;

typedef struct{
    uint8_t slaveAddr;
    uint8_t regAddr;
    uint8_t *pBuffer;
    uint8_t writeData;
    uint8_t bytesTotal;
    uint8_t bytesReceived;
    bool isRead;
    bool error;
    I2C_State_t state;
    SemaphoreHandle_t xSemaphore;
} I2C_Context_t;

static I2C_Context_t gContext;

void I2C0IntHandler(void){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t ui32status;

    ui32status = I2CMasterIntStatusEx(I2C0_BASE, true);
    I2CMasterIntClearEx(I2C0_BASE, ui32status);

    if(I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE){
        gContext.error = true;
        gContext.state = I2C_STATE_IDLE;
        xSemaphoreGiveFromISR(gContext.xSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return;
    }

    switch(gContext.state){
        case I2C_STATE_READ_SWITCH:
            I2CMasterSlaveAddrSet(I2C0_BASE, gContext.slaveAddr, true);

            if(gContext.bytesTotal == 1){
                gContext.state = I2C_STATE_READ_SINGLE;
                I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
            }
            else
            {
                gContext.state = I2C_STATE_READ_CONT;
                I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
            }
            break;

        case I2C_STATE_READ_SINGLE:
            gContext.pBuffer[gContext.bytesReceived++] = I2CMasterDataGet(I2C0_BASE);
            gContext.state = I2C_STATE_IDLE;
            xSemaphoreGiveFromISR(gContext.xSemaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            break;

        case I2C_STATE_READ_CONT:
            gContext.pBuffer[gContext.bytesReceived++] = I2CMasterDataGet(I2C0_BASE);

            if(gContext.bytesReceived == (gContext.bytesTotal - 1)){
                gContext.state = I2C_STATE_READ_FINISH;
                I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
            }
            else{
                I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
            }
            break;

        case I2C_STATE_READ_FINISH:
            gContext.pBuffer[gContext.bytesReceived++] = I2CMasterDataGet(I2C0_BASE);
            gContext.state = I2C_STATE_IDLE;
            xSemaphoreGiveFromISR(gContext.xSemaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            break;

        case I2C_STATE_WRITE_DATA:
            I2CMasterDataPut(I2C0_BASE, gContext.writeData);
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
            gContext.state = I2C_STATE_WRITE_FINISH;
            break;

        case I2C_STATE_WRITE_FINISH:
            gContext.state = I2C_STATE_IDLE;
            xSemaphoreGiveFromISR(gContext.xSemaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            break;

        default:
            break;
    }
}

void I2C_Init(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);
    I2CMasterEnable(I2C0_BASE);

    gContext.xSemaphore = xSemaphoreCreateBinary();
    gContext.state = I2C_STATE_IDLE;

    IntPrioritySet(INT_I2C0, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    I2CMasterIntEnableEx(I2C0_BASE, I2C_MASTER_INT_DATA);

    /* Enable I2C0 interrupt in the NVIC */
    IntEnable(INT_I2C0);
}

static bool i2cRunTransaction(void){
    gContext.error = false;
    gContext.bytesReceived = 0;

    /* Write mode */
    I2CMasterSlaveAddrSet(I2C0_BASE, gContext.slaveAddr, false);
    I2CMasterDataPut(I2C0_BASE, gContext.regAddr);

    gContext.state = gContext.isRead ? I2C_STATE_READ_SWITCH : I2C_STATE_WRITE_DATA;

    /* BURST_SEND_START (not SINGLE_SEND) -- holds the bus, no STOP here.
     * This is what makes the repeated START possible on the read path. */
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    xSemaphoreTake(gContext.xSemaphore, portMAX_DELAY);

    return !gContext.error;
}

bool I2C_readBytes(uint8_t slaveAddr, uint8_t regAddr, uint8_t *dest, uint8_t count)
{
    gContext.slaveAddr  = slaveAddr;
    gContext.regAddr    = regAddr;
    gContext.pBuffer    = dest;
    gContext.bytesTotal = count;
    gContext.isRead     = true;

    return i2cRunTransaction();
}

bool I2C_readByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t *dest)
{
    return I2C_readBytes(slaveAddr, regAddr, dest, 1);
}

bool I2C_readWord(uint8_t slaveAddr, uint8_t regAddr, uint16_t *dest){
    uint8_t buf[2];

    if(!I2C_readBytes(slaveAddr, regAddr, buf, 2)){
        return false;
    }

    *dest = ((uint16_t) buf[0] << 8) | buf[1];

    return true;
}

bool I2C_writeByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t data)
{
    gContext.slaveAddr = slaveAddr;
    gContext.regAddr   = regAddr;
    gContext.writeData = data;
    gContext.isRead    = false;

    return i2cRunTransaction();
}
