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
void initI2C(void);
void initUART5(void);
void initUART0(void);
void initSysTick(void);
void init_timer0A(void);
void init_timer1A(void);
//void init_timer2A(void);
void SysTick_Wait(uint32_t delay);


// Motor Control Prototypes
void bumpSensor_handler(void);
void bumper_function(void);
//void FKM(void);
void RCV_Packet(void);
//void send_timeout(void);
void Send_Packet(void);
void PD_control(void);
void I2C_handler(void);

void right_FWD(void);
void right_REV(void);
void right_brake(void);
void right_standby(void);

void left_FWD(void);
void left_REV(void);
void left_brake(void);
void left_standby(void);

void Drive_FWD(void);
void all_REV(void);
void all_FWD(void);
void all_Brake(void);

void turn_angle(int angle);

void FWD_cent(int cm);
void REV_cent(int cm);


#endif /* MOTOR_CONTROL_H_ */
