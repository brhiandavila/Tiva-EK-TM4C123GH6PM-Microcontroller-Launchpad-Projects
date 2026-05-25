#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

void I2C_Init(void);
void I2C_WriteByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t data);
uint8_t I2C_ReadByte(uint8_t slaveAddr, uint8_t regAddr);
uint16_t I2C_ReadWord(uint8_t slaveAddr, uint8_t regAddr);

#endif // I2C_H
