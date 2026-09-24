/*
 * uart.c
 *
 *  Created on: Sep 18, 2026
 *      Author: chuch
 */

#include "stm32l476xx.h"
#include "uart.h"

void UART2_Init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

    /* PA2 = TX, PA3 = RX, AF7 = USART2 */
    GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
    GPIOA->MODER |=  (2U << GPIO_MODER_MODE2_Pos) | (2U << GPIO_MODER_MODE3_Pos);
    GPIOA->AFR[0] &= ~((0xFU << (2*4)) | (0xFU << (3*4)));
    GPIOA->AFR[0] |=  ((7U  << (2*4)) | (7U  << (3*4)));

    /* Baud = 115200 @ APB1 = 80MHz, oversampling x16 (default) */
    USART2->BRR = 80000000UL / 115200UL;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void UART2_SendChar(char c)
{
    while (!(USART2->ISR & USART_ISR_TXE));
    USART2->TDR = c;
}

void UART2_SendString(const char *s)
{
    while (*s) UART2_SendChar(*s++);
}

void UART2_SendInt(int32_t val)
{
    char buf[12];
    int i = 0;
    uint32_t uval;
    if (val < 0) { UART2_SendChar('-'); uval = (uint32_t)(-val); }
    else          { uval = (uint32_t)val; }
    if (uval == 0) { UART2_SendChar('0'); return; }
    while (uval > 0) { buf[i++] = '0' + (uval % 10); uval /= 10; }
    while (i > 0) UART2_SendChar(buf[--i]);
}
