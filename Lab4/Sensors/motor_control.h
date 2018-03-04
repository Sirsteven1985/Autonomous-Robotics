/*
 * motor_control.h
 *
 *  Created on: Mar 3, 2018
 *      Author: vuong
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

//System prototypes
void initSYSCTL(void);
void initGPIO(void);
void initPWM0(void);
void initQEI(void);
void initUART0(void);
void initSysTick(void);
void SysTick_Wait(uint32_t delay);

// Motor Control Prototypes
void bumper_function(void);
void PD_control(void);
void QEI0_handler(void);
void QEI1_handler(void);
void bumpSensor_handler(void);
void right_wheel_FWD(void);
void right_wheel_REV(void);
void right_brake(void);
void right_standby(void);
void left_wheel_FWD(void);
void left_wheel_REV(void);
void all_wheel_REV(void);
void left_brake(void);
void left_standby(void);
void CW_rotate_90(void);
void CCW_rotate_90(void);
void robot_FWD(void);
void FWD_1_foot(void);

#endif /* MOTOR_CONTROL_H_ */
