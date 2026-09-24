/*
 * clock_config.c
 *
 *  Created on: Sep 18, 2026
 *      Author: chuch
 */

#include "stm32l476xx.h"
#include "clock_config.h"

void SystemClock_Config(void)
{
    /* 1. Enable power interface clock, confirm voltage scaling Range 1
     *    (required to run the core at 80MHz -- Range 2 caps out lower). */
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | PWR_CR1_VOS_0; /* Range 1 */
    while (PWR->SR2 & PWR_SR2_VOSF); /* wait for voltage scaling to settle */

    /* 2. Set MSI (internal oscillator) to 8MHz and select it as PLL
     *    input. This replaces HSE entirely -- the NUCLEO-L476RG does
     *    NOT connect the ST-LINK's MCO to OSC_IN by default (SB16/SB50
     *    are open from the factory to save power in low-power modes),
     *    so HSE would hang forever waiting on HSERDY with no physical
     *    clock signal present. MSI needs no external clock and no
     *    board modification -- this is the board's actual intended
     *    default clock path. */
    RCC->CR &= ~RCC_CR_MSIRANGE;
    RCC->CR |= RCC_CR_MSIRANGE_7;   /* range 7 = 8MHz */
    RCC->CR |= RCC_CR_MSIRGSEL;     /* apply the CR range (not the CSR one) */
    while (!(RCC->CR & RCC_CR_MSIRDY));

    /* 3. Flash wait states -- MUST be set before raising SYSCLK above
     *    the safe read speed at the current voltage. At 80MHz, Range 1
     *    requires 4 wait states per the reference manual. */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_4WS;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_4WS);

    /* 4. Configure the main PLL, sourced from MSI instead of HSE:
     *    VCO input  = MSI / M = 8MHz / 1  = 8MHz
     *    VCO output = VCO_in * N          = 8MHz * 20 = 160MHz
     *    SYSCLK     = VCO_out / R         = 160MHz / 2 = 80MHz
     *    (Same M/N/R as the original HSE-based plan -- only the
     *    source changed, so APB1 still lands at 80MHz and the CAN1
     *    bit-timing math we already did is still valid.) */
    RCC->PLLCFGR = 0;
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_MSI;          /* PLL source = MSI */
    RCC->PLLCFGR |= (0 << RCC_PLLCFGR_PLLM_Pos);      /* M = 1 (encoded as 0) */
    RCC->PLLCFGR |= (20 << RCC_PLLCFGR_PLLN_Pos);     /* N = 20 */
    RCC->PLLCFGR |= (0 << RCC_PLLCFGR_PLLR_Pos);      /* R = 2 (encoded as 0) */
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    /* 5. Switch SYSCLK source to the PLL, confirm the switch completed. */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    /* AHB/APB1/APB2 prescalers remain /1, so APB1 = SYSCLK = 80MHz. */
}
