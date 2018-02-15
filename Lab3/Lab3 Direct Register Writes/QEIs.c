/*
 * QEI0.c
 *
 *  Created on: Feb 10, 2018
 *      Author: vuong
 */


#include <stdint.h>
#include <stdbool.h>
#include "SYSCTL.h"
#include "PPB.h"
#include "QEI.h"

void initQEIs(void) {

    //Enable clock to QEI0
    SYSCTL[SYSCTL_RCGCQEI] |= SYSCTL_RCGCQEI_QEI0 | SYSCTL_RCGCQEI_QEI1;
    SYSCTL[SYSCTL_RCGCQEI] |= SYSCTL_RCGCQEI_QEI0 | SYSCTL_RCGCQEI_QEI1;

    //Disable QEIs
    QEI0[QEI_CTL] &= ~(QEI_EN);
    QEI1[QEI_CTL] &= ~(QEI_EN);

    //Enable PhA and PhB and Velocity Capture
    QEI0[QEI_CTL] |= QEI_VEL_EN | QEI_CAP_MODE;
    QEI1[QEI_CTL] |= QEI_VEL_EN | QEI_CAP_MODE;

    //Set the maximum position value 63 decimal
    QEI0[QEI_MAXPOS] |= (0x03F << 0);
    QEI1[QEI_MAXPOS] |= (0x03F << 0);

    //Enable interrupts
    QEI0[QEI_INT_EN] |= QEI_INT_DIR | QEI_INT_TIMER;
    QEI1[QEI_INT_EN] |= QEI_INT_DIR | QEI_INT_TIMER;

    //Clear interrupt flags
    QEI0[QEI_ISC] |= 0xF;
    QEI1[QEI_ISC] |= 0xF;

    //Enable interrupt handlers
    PPB[PPB_NVIC_ENO] |= PPB_NVIC_QEI0;
    PPB[PPB_NVIC_EN1] |= PPB_NVIC_QEI1;

    //Enable QEI
    QEI0[QEI_CTL] |= QEI_EN;
    QEI1[QEI_CTL] |= QEI_EN;

    //Delay for sometime
    for(uint32_t m=0; m<1000; m++);


}
