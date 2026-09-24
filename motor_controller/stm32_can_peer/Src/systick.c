/*
 * systick.c
 *
 *  Created on: Sep 18, 2026
 *      Author: chuch
 */

#include "stm32l476xx.h"
#include "systick.h"

static volatile uint32_t msTicks = 0;

void SysTick_Handler(void) { msTicks++; }

void SysTick_Init(uint32_t core_clock_hz)
{
    SysTick_Config(core_clock_hz / 1000); /* 1ms tick */
}

uint32_t millis(void) { return msTicks; }
