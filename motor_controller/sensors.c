/******************************************************************************
 * PIN MAPPING
 *
 * INA260 Current Monitor (I2C0)
 * - PB2 -> SCL
 * - PB3 -> SDA
 * - I2C Address: 0x40
 *
 * TMP119 Temperature Sensor (I2C0)
 * - PB2 -> SCL
 * - PB3 -> SDA
 * - I2C Address: 0x48
 *
 *****************************************************************************/

#include "sensors.h"
#include "uart_mutex.h"

#include "utils/uartstdio.h"

float INA260_ReadCurrent(void){
    float current;

    current = (int16_t)I2C_ReadWord(INA260_ADDR, 0x01);

    return (current * 1.25);
}

float INA260_ReadVoltage(void){
    float voltage;

    voltage = I2C_ReadWord(INA260_ADDR, 0x02);

    return (voltage * 1.25);
}

float INA260_ReadPower(void){
    float power;

    power = I2C_ReadWord(INA260_ADDR, 0x03);

    return (power * 10);
}

float TMP119_ReadTemp(void){
    uint16_t raw = I2C_ReadWord(TMP119_ADDR, 0x00);

    UART_Print("TMP raw: %d\r\n", raw);
    if(raw > 5000 || raw == 0){
        return -999.0f;  // sentinel garbage value
    }

    float temp = (int16_t)raw;
    return (temp * 0.0078125f);
}
