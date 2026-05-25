/*
 * i2c_driver.h
 *
 * TM4C123 I2C0 driver — PB2 (SCL), PB3 (SDA), 400 kHz Fast Mode
 *
 * The implementation (i2c_driver.c) is now interrupt-driven —
 * callers block on a FreeRTOS binary semaphore instead of polling.
 * This header remains dependency-free; FreeRTOS includes are in .c only.
 */

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

/*---------------------------------------------------------------------------
 * I2C_init
 *
 * Configures I2C0 on PB2 (SCL) and PB3 (SDA) at 400 kHz (Fast Mode).
 * Must be called once before any read/write calls.
 * Call after SysCtlClockSet() — the I2C baud divisor depends on system clock.
 *--------------------------------------------------------------------------*/
void I2C_init(void);

/*---------------------------------------------------------------------------
 * I2C_writeByte
 *
 * Writes a single byte to a register on a slave device.
 *
 *   slaveAddr  — 7-bit I2C address of the target device (e.g. 0x68 for MPU)
 *   regAddr    — register address to write to
 *   data       — byte value to write
 *
 * Returns true on success, false if the bus reported an error.
 *
 * PITFALL: Never call this from an ISR. I2C transactions are blocking
 *   (polling-based). In a later step we protect this with a mutex.
 *--------------------------------------------------------------------------*/
bool I2C_writeByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t data);

/*---------------------------------------------------------------------------
 * I2C_readByte
 *
 * Reads a single byte from a register on a slave device.
 *
 *   slaveAddr  — 7-bit I2C address of the target device
 *   regAddr    — register address to read from
 *   dest       — pointer to store the received byte
 *
 * Returns true on success, false on error.
 *
 * Internally performs a write (send regAddr) followed by a repeated START
 * and then a read — this is the standard I2C register-read pattern.
 *--------------------------------------------------------------------------*/
bool I2C_readByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t *dest);

/*---------------------------------------------------------------------------
 * I2C_readBurst
 *
 * Reads `length` consecutive bytes starting at `regAddr`.
 * The MPU-6050 auto-increments its internal register pointer, so a burst
 * read of 14 bytes starting at 0x3B gives you all 6 axes + temperature
 * in one transaction — no bus overhead between axes.
 *
 *   slaveAddr  — 7-bit I2C address
 *   regAddr    — first register address
 *   dest       — buffer to write into (must be at least `length` bytes)
 *   length     — number of bytes to read
 *
 * Returns true on success, false on error.
 *--------------------------------------------------------------------------*/
bool I2C_readBurst(uint8_t slaveAddr, uint8_t regAddr, uint8_t *dest, uint8_t length);

#endif/* I2C_DRIVER_H */
