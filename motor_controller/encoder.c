#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/qei.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "encoder.h"

void Encoder_Init(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));

    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= GPIO_PIN_7;

    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);

    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    // 4294967295 is the maximum uint32 value since I am using velocity mode, not position mode
    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 4294967295);

    // window duration = ticks / system clock
    //                 = 800,000 / 80,000,000
    //                 = 10ms or 0.010s
    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, 800000);
    QEIVelocityEnable(QEI0_BASE);
    QEIEnable(QEI0_BASE);
}

int32_t Encoder_GetSpeed(void){
    int32_t speed;
    int32_t dir;

    speed = QEIVelocityGet(QEI0_BASE);
    dir = QEIDirectionGet(QEI0_BASE);

    return (speed * dir);
}
