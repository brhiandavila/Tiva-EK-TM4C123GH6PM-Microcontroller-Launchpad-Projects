/*
 * can1.c
 *
 *  Created on: Sep 18, 2026
 *      Author: chuch
 */

#include "stm32l476xx.h"
#include "can1.h"
#include "uart.h"

#define TELEMETRY_ID 0x001U
#define HEARTBEAT_ID 0x002U

void CAN1_Init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_CAN1EN;

    /* PA11 = CAN1_RX, PA12 = CAN1_TX, AF9 */
    GPIOA->MODER &= ~(GPIO_MODER_MODE11 | GPIO_MODER_MODE12);
    GPIOA->MODER |=  (2U << GPIO_MODER_MODE11_Pos) | (2U << GPIO_MODER_MODE12_Pos);
    GPIOA->AFR[1] &= ~((0xFU << ((11-8)*4)) | (0xFU << ((12-8)*4)));
    GPIOA->AFR[1] |=  ((9U  << ((11-8)*4)) | (9U  << ((12-8)*4)));

    /* Enter init mode */
    CAN1->MCR |= CAN_MCR_INRQ;
    while (!(CAN1->MSR & CAN_MSR_INAK));
    CAN1->MCR &= ~CAN_MCR_SLEEP;

    /* Bit timing for 500kbit/s @ APB1=80MHz: BRP=10, TS1=13tq, TS2=2tq, SJW=1tq
     * (matches the TM4C's CANBitRateSet(..., 500000) exactly) */
    CAN1->BTR = (0U << 24)          /* SJW = 1 (encoded -1) */
              | (1U << 20)          /* TS2 = 2 (encoded -1) */
              | (12U << 16)         /* TS1 = 13 (encoded -1) */
              | (9U << 0);          /* BRP = 10 (encoded -1) */

    /* Filter bank 0: accept ONLY standard ID 0x001 (telemetry) into FIFO0 */
    CAN1->FMR |= CAN_FMR_FINIT;
    CAN1->FA1R &= ~(1U << 0);
    CAN1->FS1R |= (1U << 0);              /* 32-bit scale */
    CAN1->FM1R &= ~(1U << 0);             /* mask mode */
    CAN1->sFilterRegister[0].FR1 = (TELEMETRY_ID << 21);
    CAN1->sFilterRegister[0].FR2 = (0x7FFU << 21); /* exact-match mask */
    CAN1->FFA1R &= ~(1U << 0);            /* assign to FIFO0 */
    CAN1->FA1R |= (1U << 0);              /* activate */
    CAN1->FMR &= ~CAN_FMR_FINIT;

    /* Leave init mode -> normal mode */
    CAN1->MCR &= ~CAN_MCR_INRQ;
    while (CAN1->MSR & CAN_MSR_INAK);

    /* RX FIFO0 message-pending interrupt */
    CAN1->IER |= CAN_IER_FMPIE0;
    NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

void CAN1_SendHeartbeat(void)
{
    while (!(CAN1->TSR & CAN_TSR_TME0)); /* wait for mailbox 0 free */
    CAN1->sTxMailBox[0].TIR = (HEARTBEAT_ID << 21);
    CAN1->sTxMailBox[0].TDTR = 0; /* DLC = 0, empty payload */
    CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;
}

void CAN1_RX0_IRQHandler(void)
{
    if (CAN1->RF0R & CAN_RF0R_FMP0)
    {
        uint32_t data_lo = CAN1->sFIFOMailBox[0].RDLR;

        /* TM4C packs big-endian: byte0=speed_hi, byte1=speed_lo,
         * byte2=current_hi, byte3=current_lo */
        int16_t speed   = (int16_t)(((data_lo & 0xFF) << 8) | ((data_lo >> 8) & 0xFF));
        int16_t current = (int16_t)((((data_lo >> 16) & 0xFF) << 8) | ((data_lo >> 24) & 0xFF));

        UART2_SendString("RX Speed: ");
        UART2_SendInt(speed);
        UART2_SendString(" | Current: ");
        UART2_SendInt(current);
        UART2_SendString(" mA\r\n");

        CAN1->RF0R |= CAN_RF0R_RFOM0; /* release FIFO */
    }
}
