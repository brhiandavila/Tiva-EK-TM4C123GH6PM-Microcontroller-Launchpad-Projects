/*
 * systick.h
 *
 *  Created on: Sep 18, 2026
 *      Author: chuch
 */

#ifndef SYSTICK_H_
#define SYSTICK_H_

#include <stdint.h>

void SysTick_Init(uint32_t core_clock_hz);
uint32_t millis(void);

#endif /* SYSTICK_H_ */
