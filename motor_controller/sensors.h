#ifndef SENSORS_H
#define SENSORS_H

#include <i2c_helper.h>

#define INA260_ADDR     0x40
#define TMP119_ADDR     0x48

typedef struct{
    float current;
    float voltage;
    float power;
    float temperature;
} SensorData_t;

// INA260
float INA260_ReadCurrent(void);
float INA260_ReadVoltage(void);
float INA260_ReadPower(void);

// TMP119
float TMP119_ReadTemp(void);

#endif // SENSOR_H
