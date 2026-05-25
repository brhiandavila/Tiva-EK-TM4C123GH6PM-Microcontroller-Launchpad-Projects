/******************************************************************************
 * PIN MAPPING
 *
 * DRV8833 Motor Driver (PWM0)
 * - PE4 -> AIN1 (PWM0, Generator 2, Output 4)
 * - PE5 -> AIN2 (PWM0, Generator 2, Output 5)
 *
 * PWM Configuration:
 * - PWM Clock: 80 MHz (no divider)
 * - PWM Frequency: 1000 Hz
 * - Period Ticks: 80,000
 * - Duty Cycle Range: 0% to 100%
 *
 * Motor Control:
 * - Positive speed -> Forward  (AIN1 = PWM, AIN2 = idle)
 * - Negative speed -> Reverse  (AIN1 = idle, AIN2 = PWM)
 * - Zero speed     -> Stop     (AIN1 = idle, AIN2 = idle)
 *
 *****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "drv8833.h"

void DRV8833_Init(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    GPIOPinConfigure(GPIO_PE4_M0PWM4);
    GPIOPinConfigure(GPIO_PE5_M0PWM5);
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    // frequency = system clock / period ticks
    //           = 80,000,000 / 80,000
    //           = 1,000Hz
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 80000);

    // duty cycle % = (pulse width ticks / 80,000) * 100
    // EXAMPLE:     = (8,000 / 80,000) * 100
    //              = 10% duty cycle
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
}

void DRV8833_SetMotor(int32_t speed){
    uint32_t pulse;

    if(speed > 0){
        // forward
        pulse = (speed * 80000) / 100;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, pulse);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 1);
    }
    else if(speed < 0){
        // reverse
        pulse = (-speed * 80000) / 100;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 1);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, pulse);
    }
    else{
        // stop
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 1);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 1);
    }
}
