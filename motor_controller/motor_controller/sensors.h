#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>

#define INA260_ADDR     0x40
#define TMP117_ADDR     0x48

typedef struct{
    float current;
    float voltage;
    float power;
    float temperature;
} SensorData_t;

// INA260
bool INA260_ReadCurrent(float *dest);
bool INA260_ReadVoltage(float *dest);
bool INA260_ReadPower(float *dest);

// TMP117
bool TMP117_ReadTemp(float *dest);

#endif // SENSORS_H
