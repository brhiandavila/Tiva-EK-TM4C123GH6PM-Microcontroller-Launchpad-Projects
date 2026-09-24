/******************************************************************************
 * PIN MAPPING
 *
 * INA260 Current Monitor (I2C0)
 * - PB2 -> SCL
 * - PB3 -> SDA
 * - I2C Address: 0x40
 *
 * TMP117 Temperature Sensor (I2C0)
 * - PB2 -> SCL
 * - PB3 -> SDA
 * - I2C Address: 0x48
 *
 *****************************************************************************/
#include <i2c_driver.h>
#include <stdint.h>
#include <stdbool.h>

#include "sensors.h"
#include "uart_mutex.h"

#include "utils/uartstdio.h"

bool INA260_ReadCurrent(float *dest){
    uint16_t raw;

    if(!I2C_readWord(INA260_ADDR, 0x01, &raw))
        return false;

    *dest = (float)(uint16_t)raw * 1.25f;
    return true;
}

bool INA260_ReadVoltage(float *dest){
    uint16_t raw;

    if(!I2C_readWord(INA260_ADDR, 0x02, &raw))
        return false;

    *dest = (float)raw * 1.25f;
    return true;
}

bool INA260_ReadPower(float *dest){
    uint16_t raw;

    if(!I2C_readWord(INA260_ADDR, 0x03, &raw))
        return false;

    *dest = (float)raw * 10.0f;
    return true;
}

bool TMP117_ReadTemp(float *dest){
    uint16_t raw;

    if(!I2C_readWord(TMP117_ADDR, 0x00, &raw))
        return false;

    /* TMP117 register is a signed 16-bit value. Interpret it as
     * such BEFORE any range check, so legitimate negative temperatures
     * aren't misread as a garbage/error condition. */
    int16_t signedRaw = (int16_t)raw;

    *dest = (float)signedRaw * 0.0078125f;
    return true;
}
