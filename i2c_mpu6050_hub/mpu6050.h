/*
 * mpu6050.h
 *
 * MPU-6050 register definitions and public API.
 * No TM4C123-specific code here — this layer only knows about the sensor.
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------
 * I2C address
 * AD0 pin LOW (tied to GND on Adafruit board) -> 0x68
 * AD0 pin HIGH -> 0x69
 * -----------------------------------------------------------------------*/
#define MPU6050_ADDR        0x68

/* -------------------------------------------------------------------------
 * Register addresses — only what Project 5 needs
 * -----------------------------------------------------------------------*/
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_ACCEL_XOUT_H    0x3B   /* First of 14 consecutive bytes */
#define MPU6050_REG_WHO_AM_I        0x75   /* Should return 0x68            */

/* -------------------------------------------------------------------------
 * Data struct
 *
 * PITFALL #5: All fields are int16_t (signed). The raw bytes from the
 *   sensor are two's complement. Using uint16_t here would corrupt all
 *   negative readings without a compile-time warning.
 * -----------------------------------------------------------------------*/
typedef struct {
    int16_t  accel_x;
    int16_t  accel_y;
    int16_t  accel_z;
    int16_t  gyro_x;
    int16_t  gyro_y;
    int16_t  gyro_z;
    int16_t  raw_temp;        /* °C = (raw_temp / 340.0f) + 36.53f         */
    uint32_t timestamp_ms;    /* Populated by caller via xTaskGetTickCount  */
} MPU6050_Data_t;


/*---------------------------------------------------------------------------
 * MPU6050_whoAmI
 * Reads the WHO_AM_I register (0x75).
 * Returns true and writes 0x68 into *result if the sensor is present
 * and the I2C bus is working. Use this as your first hardware sanity check.
 *--------------------------------------------------------------------------*/
bool MPU6050_whoAmI(uint8_t *result);

/*---------------------------------------------------------------------------
 * MPU6050_init
 * Wakes the MPU-6050 from sleep mode (clears bit 6 of PWR_MGMT_1).
 * The sensor powers up in sleep mode by default — if you skip this,
 * all register reads return 0 and you will spend an hour debugging.
 * Returns true on success.
 *--------------------------------------------------------------------------*/
bool MPU6050_init(void);

/*---------------------------------------------------------------------------
 * MPU6050_readAll
 * Burst-reads all 14 bytes (accel + temp + gyro) and parses them into dest.
 * Returns true on success.
 *--------------------------------------------------------------------------*/
bool MPU6050_readAll(MPU6050_Data_t *dest);

#endif /* MPU6050_H */
