/*
 * mpu6050.c
 */

#include "mpu6050.h"
#include "i2c_driver.h"

bool MPU6050_whoAmI(uint8_t *result){
    return I2C_readByte(MPU6050_ADDR, MPU6050_REG_WHO_AM_I, result);
}

bool MPU6050_init(void)
{
    /* Write 0x00 to PWR_MGMT_1.
     * Bit 6 (SLEEP) defaults to 1 at power-on. Clearing the entire
     * register wakes the device and selects the internal 8 MHz oscillator.
     * For better gyro stability you would later set bits [2:0] to 0x01
     * to use the gyro X PLL as the clock — but 0x00 is correct for now. */
    return I2C_writeByte(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x00);
}

bool MPU6050_readAll(MPU6050_Data_t *dest)
{
    uint8_t raw[14];

    /* Burst-read 14 bytes starting at ACCEL_XOUT_H (0x3B).
     * The MPU-6050 register layout is fixed and consecutive:
     *
     *   raw[0..1]   = ACCEL_X  high byte, low byte
     *   raw[2..3]   = ACCEL_Y
     *   raw[4..5]   = ACCEL_Z
     *   raw[6..7]   = TEMP
     *   raw[8..9]   = GYRO_X
     *   raw[10..11] = GYRO_Y
     *   raw[12..13] = GYRO_Z
     */
    if (!I2C_readBurst(MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, raw, 14))
    {
        return false;
    }

    /* PITFALL #5 — sign extension:
     * Cast to int16_t AFTER combining bytes, not before.
     * (int16_t)(raw[0] << 8) would sign-extend the shift on some compilers.
     * The pattern (int16_t)((uint16_t)raw[H] << 8 | raw[L]) is portable
     * and correct on all C99-compliant compilers. */
    dest->accel_x = (int16_t)((uint16_t)raw[0]  << 8 | raw[1]);
    dest->accel_y = (int16_t)((uint16_t)raw[2]  << 8 | raw[3]);
    dest->accel_z = (int16_t)((uint16_t)raw[4]  << 8 | raw[5]);
    dest->raw_temp = (int16_t)((uint16_t)raw[6] << 8 | raw[7]);
    dest->gyro_x  = (int16_t)((uint16_t)raw[8]  << 8 | raw[9]);
    dest->gyro_y  = (int16_t)((uint16_t)raw[10] << 8 | raw[11]);
    dest->gyro_z  = (int16_t)((uint16_t)raw[12] << 8 | raw[13]);

    /* timestamp_ms is intentionally left to the caller.
     * This function has no knowledge of FreeRTOS tick time —
     * that separation keeps the driver layer portable. */
    dest->timestamp_ms = 0;

    return true;
}
