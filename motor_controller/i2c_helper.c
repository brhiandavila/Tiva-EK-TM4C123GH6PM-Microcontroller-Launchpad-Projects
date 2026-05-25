#include <stdint.h>
#include <stdbool.h>

#include "driverlib/i2c.h"
#include "i2c_helper.h"
#include "FreeRTOS.h"
#include "task.h"

#include "uart_mutex.h"
#include "utils/uartstdio.h"

void I2C_Init(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
    I2CMasterEnable(I2C0_BASE);
}

uint16_t I2C_ReadWord(uint8_t slaveAddr, uint8_t regAddr){
    uint8_t highByte;
    uint8_t lowByte;

    I2CMasterSlaveAddrSet(I2C0_BASE, slaveAddr, false);
    I2CMasterDataPut(I2C0_BASE, regAddr);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while(I2CMasterBusy(I2C0_BASE));

    // Switch directions from writing to the slave to reading from it
    I2CMasterSlaveAddrSet(I2C0_BASE, slaveAddr, true);

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(I2C0_BASE));

    highByte = I2CMasterDataGet(I2C0_BASE);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(I2CMasterBusy(I2C0_BASE));

    lowByte = I2CMasterDataGet(I2C0_BASE);

    return (uint16_t) (highByte << 8 | lowByte);
}
