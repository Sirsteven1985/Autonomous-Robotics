/*
 * M1PWMn.c
 *
 *  Created on: Jan 30, 2018
 *      Author: vuong
 */

#include <MnPWMn.h>
#include <stdint.h>
#include "SYSCTL.h"

void initM1PWMn(void) {

    //Enable clock to M1PWMn
    SYSCTL[SYSCTL_RCGCPWM] |= SYSCTL_RCGC_M1PWMn;
    SYSCTL[SYSCTL_RCGCPWM] |= SYSCTL_RCGC_M1PWMn;

//    6. Configure the PWM generator for countdown mode with immediate updates to the parameters.
//    ■ Write the PWM0CTL register with a value of 0x0000.0000.
        M1PWMn[PWM1_CTL] = 0x00000000;
//    ■ Write the PWM0GENA register with a value of 0x0000.008C.
        //M1PWMn[PWM1_GENA] = 0x000000B0;                                 //
//    ■ Write the PWM0GENB register with a value of 0x0000.080C.
        M1PWMn[PWM1_GENB] = 0x000000B0;
//    7. Set the period. For a 25-KHz frequency, the period = 1/25,000, or 40 microseconds. The PWM
//    clock source is 10 MHz; the system clock divided by 2. Thus there are 400 clock ticks per period.
//    Use this value to set the PWM0LOAD register. In Count-Down mode, set the LOAD field in the
//    PWM0LOAD register to the requested period minus one.
//    ■ Write the PWM0LOAD register with a value of 0x0000.018F.
        M1PWMn[PWM1_LOAD] = 0x0000018F;                                 //399 decimal
//    8. Set the pulse width of the MnPWM0 pin for a 25% duty cycle.
//    ■ Write the PWM0CMPA register with a value of 0x0000.012B.
        //M1PWMn[PWM1_CMPA] = 0x0000012B;                                 //299 decimal
//    9. Set the pulse width of the MnPWM1 pin for a 75% duty cycle.
//    ■ Write the PWM0CMPB register with a value of 0x0000.0063.
        M1PWMn[PWM1_CMPB] = 0x0000012B;
//    10. Start the timers in PWM generator 0.
//    ■ Write the PWM0CTL register with a value of 0x0000.0001.
        M1PWMn[PWM1_CTL] |= (0x7 << 0);

        M1PWMn[PWM_CTL] |= (1 << 0);

//    11. Enable PWM outputs.
//    ■ Write the PWMENABLE register with a value of 0x0000.0003.
        M1PWMn[PWM_ENABLE] |= (1 << 1);

}
