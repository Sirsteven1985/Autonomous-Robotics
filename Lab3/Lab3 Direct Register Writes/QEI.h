/*
 * QEI.h
 *
 *  Created on: Feb 10, 2018
 *      Author: vuong
 */

#ifndef QEI_H_
#define QEI_H_

#include <stdint.h>

#define QEI0                            ((volatile uint32_t *)0x4002C000)
#define QEI1                            ((volatile uint32_t *)0x4002D000)

enum{

    QEI_CTL         =   (0x000 >> 2),   //QEI control register
    QEI_STAT        =   (0x004 >> 2),   //QEI status register
    QEI_MAXPOS      =   (0x00C >> 2),   //Maximum position register
    QEI_POS         =   (0x008 >> 2),   //Position register
    QEI_LOAD        =   (0x010 >> 2),   //Load register

    QEI_TIME        =   (0x014 >> 2),   //Timer register                READ ONLY
    QEI_COUNT       =   (0x018 >> 2),   //Velocity counter register     READ ONLY
                                        //Note* The QEISPEED register should be
                                        //used to determine the actual encoder
                                        //velocity; this register is provided for
                                        //information purposes only. This counter
                                        //does not increment when the VELEN
                                        //bit in the QEICTL register is clear.
    QEI_SPEED       =   (0x01C >> 2),   //Velocity register             READ ONLY

    QEI_INT_EN      =   (0x020 >> 2),   //Interrupt enable register
    QEI_RIS         =   (0x024 >> 2),   //Raw interrupt status register READ ONLY
    QEI_ISC         =   (0x028 >> 2),   //Interrupt status and clear register

};

enum{

    QEI_EN          =   (1 << 0),       //Enable QEI
    QEI_VEL_DIV     =   (0x0 << 6),     //Predivide velocity (Page 1313 for additional settings)
    QEI_VEL_EN      =   (1 << 5),       //Enables capture of the velocity of the quadrature encoder
    QEI_CAP_MODE    =   (1 << 3),       //PhA and PhB edges are counted
    QEI_STAT_DIR    =   (1 << 1),       //encoder rotation direction

    QEI_INT_DIR     =   (1 << 2),       //Interrupt Direction
    QEI_INT_TIMER   =   (1 << 1),       //Interrupt timer
    QEI_INT_INDEX   =   (1 << 0),       //Interrupt Index

};


#endif /* QEI_H_ */
