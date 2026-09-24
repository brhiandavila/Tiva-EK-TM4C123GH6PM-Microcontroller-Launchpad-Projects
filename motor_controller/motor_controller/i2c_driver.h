#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

void I2C_Init(void);

bool I2C_readByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t *dest);
bool I2C_readBytes(uint8_t slaveAddr, uint8_t regAddr, uint8_t *dest, uint8_t count);
bool I2C_readWord(uint8_t slaveAddr, uint8_t regAddr, uint16_t *dest);
bool I2C_writeByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t data);

#endif // I2C_DRIVER_H
