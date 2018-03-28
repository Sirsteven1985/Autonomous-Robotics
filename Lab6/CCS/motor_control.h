/*
 * motor_control.h
 *
 *  Created on: Mar 7, 2018
 *      Author: vuong
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

//System prototypes
void initSYSCTL(void);
void initGPIO(void);
void initPWM0(void);
void initQEI(void);
void initADC(void);
void initUART5(void);
void initSysTick(void);
void init_timerA(void);
void SysTick_Wait(uint32_t delay);


// Motor Control Prototypes
void ADC_function(void);
void IR_functions(void);
void QEI0_handler(void);
void QEI1_handler(void);
void bumpSensor_handler(void);
void bumper_function(void);
void PD_control(void);

void right_FWD(void);
void right_REV(void);
void right_brake(void);
void right_standby(void);

void left_FWD(void);
void left_REV(void);
void left_brake(void);
void left_standby(void);

void all_REV(void);
void all_FWD(void);

void turn_angle(int angle);
void CW_90(void);
void CCW_90(void);

void FWD_cent(int cm);
void FWD_1_foot(void);


#endif /* MOTOR_CONTROL_H_ */
