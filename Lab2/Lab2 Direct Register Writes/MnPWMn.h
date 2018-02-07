/*
 * PWM.h
 *
 *  Created on: Jan 30, 2018
 *      Author: vuong
 */


#ifndef MNPWMN_H_
#define MNPWMN_H_

#include <stdint.h>

#define M1PWM ((volatile uint32_t *)0x40029000)

enum {

    PWM_CTL      =   (0x000 >> 2), // PWM Master Control
    PWM_ENABLE   =   (0x008 >> 2), // Enable

    PWM1_CTL     =   (0x080 >> 2), // Generator 1 Control
    PWM1_LOAD    =   (0x090 >> 2), // Load Value
    PWM1_COUNT   =   (0x094 >> 2), // Count
    PWM1_CMPA    =   (0x098 >> 2), // Comparator A
    PWM1_CMPB    =   (0x09C >> 2), // Comparator B
    PWM1_GENA    =   (0x0A0 >> 2), // Generator A
    PWM1_GENB    =   (0x0A4 >> 2), // Generator B


    PWM0_CTL     =   (0x040 >> 2), // Generator 0 Control
    PWM0_LOAD    =   (0x050 >> 2), // Load Value
    PWM0_COUNT   =   (0x054 >> 2), // Count
    PWM0_CMPA    =   (0x058 >> 2), // Comparator A
    PWM0_CMPB    =   (0x05C >> 2), // Comparator B
    PWM0_GENA    =   (0x060 >> 2), // Generator A
    PWM0_GENB    =   (0x064 >> 2), // Generator B

};

#endif /* MNPWMN_H_ */
