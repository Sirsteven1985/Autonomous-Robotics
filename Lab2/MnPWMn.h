/*
 * PWM.h
 *
 *  Created on: Jan 30, 2018
 *      Author: vuong
 */


#ifndef MNPWMN_H_
#define MNPWMN_H_

#include <stdint.h>

#define M1PWMn ((volatile uint32_t *)0x40029000)

enum {

    PWM_CTL      =   (0x000 >> 2), // PWM Master Control
    PWM_ENABLE   =   (0x008 >> 2), // Enable
    PWM1_CTL     =   (0x080 >> 2), // PWM0 Control
    PWM1_LOAD    =   (0x090 >> 2), // Load Value
    PWM1_COUNT   =   (0x094 >> 2), // PWM0 Count
    PWM1_CMPA    =   (0x098 >> 2), // PWM0 Comperator A
    PWM1_CMPB    =   (0x09C >> 2), // PWM0 Comperator B
    PWM1_GENA    =   (0x0A0 >> 2), // PWM Generator A
    PWM1_GENB    =   (0x0A4 >> 2), // PWM Generator B
    PWM1_DBCTL    =   (0x0A8 >> 2), // PWM Generator B
    PWM1_DBRISE    =   (0x0AC >> 2), // PWM Generator B
    PWM1_DBFALL    =   (0x0B0 >> 2), // PWM Generator B


    PWM2_CTL     =   (0x0C0 >> 2), // PWM0 Control
    PWM2_LOAD    =   (0x0D0 >> 2), // Load Value
    PWM2_COUNT   =   (0x0D4 >> 2), // PWM0 Count
    PWM2_CMPA    =   (0x0D8 >> 2), // PWM0 Comperator A
    PWM2_CMPB    =   (0x0DC >> 2), // PWM0 Comperator B
    PWM2_GENA    =   (0x0E0 >> 2), // PWM Generator A
    PWM2_GENB    =   (0x0E4 >> 2), // PWM Generator B
    PWM2_DBCTL    =   (0x0E8 >> 2), // PWM Generator B
    PWM2_DBRISE    =   (0x0EC >> 2), // PWM Generator B
    PWM2_DBFALL    =   (0x0F0 >> 2), // PWM Generator B

};

#endif /* MNPWMN_H_ */
