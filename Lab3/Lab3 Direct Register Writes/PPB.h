/*
 * PPB.h
 *
 *  Created on: Feb 10, 2018
 *      Author: vuong
 */

#ifndef PPB_H_
#define PPB_H_

#define       PPB               ((volatile uint32_t *)0xE000E000)

enum{

        PPB_NVIC_ENO          = (0x100 >> 2), //interrupt #s 0-31
#define PPB_NVIC_QEI0           (1 << 13)     //QEI0 interrupt #13
        PPB_NVIC_EN1          = (0x104 >> 2), //interrupt #s 32-63
#define PPB_NVIC_QEI1           (1 << 6)      //QEI0 interrupt #38

};

#endif /* PPB_H_ */
