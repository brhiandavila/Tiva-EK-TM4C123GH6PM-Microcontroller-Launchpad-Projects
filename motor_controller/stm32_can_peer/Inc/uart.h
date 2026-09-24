/*
 * uart.h
 *
 *  Created on: Sep 18, 2026
 *      Author: chuch
 */

#ifndef UART_H_
#define UART_H_

#include <stdint.h>

void UART2_Init(void);
void UART2_SendString(const char *s);
void UART2_SendInt(int32_t val);

#endif /* UART_H_ */
