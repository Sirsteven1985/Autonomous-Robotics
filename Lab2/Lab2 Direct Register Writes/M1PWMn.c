/*
 * M1PWMn.c
 *
 *  Created on: Jan 30, 2018
 *      Author: vuong
 */

#include <stdint.h>
#include "SYSCTL.h"
#include "MnPWMn.h"

void initM1PWMn(void) {

    //Enable clock to M1PWMn
    SYSCTL[SYSCTL_RCGCPWM] |= SYSCTL_RCGC_M1PWM;
    SYSCTL[SYSCTL_RCGCPWM] |= SYSCTL_RCGC_M1PWM;

    for(uint32_t i = 0; i < 0xFFFF; i++);      // Need extra delay for PWM clock to set.

    //6. Configure the PWM generator for countdown mode with immediate updates to the parameters.
    //■ Write the PWM0CTL register with a value of 0x0000.0000.
    M1PWM[PWM0_CTL] &= 0x00000000;
    M1PWM[PWM1_CTL] &= 0x00000000;
    //■ Write the PWM0GENA register with a value of 0x0000.008C.
    //M1PWM[PWM0_GENA] |= (0x08C << 0);
    //M1PWM[PWM1_GENA] |= (0x08C << 0);
    //■ Write the PWM0GENB register with a value of 0x0000.080C.
    M1PWM[PWM1_GENB] |= (0x80C << 0);
    M1PWM[PWM0_GENB] |= (0x80C << 0);

    //7. Set the period. For a 25-KHz frequency, the period = 1/25,000, or 40 microseconds. The PWM
    //clock source is 10 MHz; the system clock divided by 2. Thus there are 400 clock ticks per period.
    //Use this value to set the PWM0LOAD register. In Count-Down mode, set the LOAD field in the
    //PWM0LOAD register to the requested period minus one.
    //■ Write the PWM0LOAD register with a value of 0x0000.018F.
    //Generator 0
    M1PWM[PWM0_LOAD] |= (0x18F << 0);                                 //399 decimal
    //Generator 1
    M1PWM[PWM1_LOAD] |= (0x18F << 0);

    //8. Set the pulse width of the MnPWM0 pin for a 25% duty cycle.
    //■ Write the PWM0CMPA register with a value of 0x0000.012B.
    //Generator 0
    //M1PWM[PWM0_CMPA] |= (0x12B << 0);                                 //299 decimal
    //Generator 1
    //M1PWM[PWM1_CMPA] |= (0x12B << 0);

    //9. Set the pulse width of the MnPWM1 pin for a 75% duty cycle.
    //■ Write the PWM0CMPB register with a value of 0x0000.0063.
    //Generator 0
    M1PWM[PWM0_CMPB] |= (0x12B << 0);
    //Generator 1
    M1PWM[PWM1_CMPB] |= (0x12B << 0);

    //10. Start the timers in PWM generator 0.
    //■ Write the PWM0CTL register with a value of 0x0000.0001.
    //Generator 0
    M1PWM[PWM0_CTL] |= (1 << 0);
    //Generator 1
    M1PWM[PWM1_CTL] |= (1 << 0);

    //11. Enable PWM outputs.
    //■ Write the PWMENABLE register with a value of 0x0000.0002.
    M1PWM[PWM_ENABLE] |= (0x3 << 0);

}
