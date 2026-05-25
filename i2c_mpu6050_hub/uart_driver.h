/*
 * uart_driver.h
 *
 * UART0 driver for i2c_mpu6050_hub
 * Provides thread-safe printing via mutex and UART RX interrupt
 * for command processing.
 */

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/*---------------------------------------------------------------------------
 * vUARTDriverInit
 *
 * Initializes UART0 on PA0 (RX) and PA1 (TX) at 115200 8-N-1.
 * Also enables the UART RX interrupt for command processing.
 * Must be called once before any other UART functions.
 * Call from prvSetupHardware() before scheduler starts.
 *--------------------------------------------------------------------------*/
void vUARTDriverInit(void);

/*---------------------------------------------------------------------------
 * vUARTPrint
 *
 * Mutex-protected wrapper around UARTprintf.
 * Safe to call from any task context.
 *
 * Do NOT call from an ISR — takes a mutex which can block.
 * Do NOT call before vUARTDriverInit() — UART not configured yet.
 *
 *   xMutex  — the shared UART mutex handle
 *   pcFormat — printf-style format string
 *   ...      — variable arguments
 *--------------------------------------------------------------------------*/
void vUARTPrint(SemaphoreHandle_t xMutex,
                const char *pcFormat, ...);

/*---------------------------------------------------------------------------
 * vUARTSetCommandTask
 *
 * Registers the command task handle with the UART driver so the
 * RX ISR knows which task to notify when a character arrives.
 * Must be called after vCommandTask is created, before scheduler starts.
 *--------------------------------------------------------------------------*/
void vUARTSetCommandTask(TaskHandle_t xTaskToNotify);

#endif /* UART_DRIVER_H */
