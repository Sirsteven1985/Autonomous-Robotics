

/**
 * main.c
 * ECPE 155 Autonomous Robotics
 * Spring 2018
 * Lab 3 - Quadrature Encoder Interface (QEI), wheel speed measurement and motor control
 * Lab Associates:  Paul Vuong
 *                  Steve Guerro
 *
 * 1. The QEI module interprets the two-bit gray code produced by a quadrature encoder wheel to integrate position over time and determine direction of rotation.
 *    In addition, it can capture a running estimate of the velocity of the encoder wheel.
 *
 * 2. The position integrator and velocity capture can be independently enabled, though the position integrator must be enabled before the velocity capture can be enabled.
 *
 * 3. The robot will accept parameters from the programmer such as current position and final destination. These parameters along with parameters measured from the qei
 *    will develop the wheel speeds and direction to reach a destination. Additionally, the programmer will need to input dimensions for the robot.
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include "SYSCTL.h"
#include "GPIO.h"
#include "OSC.h"
#include "PPB.h"
#include "MnPWMn.h"
#include "M1PWMn.h"
#include "QEI.h"
#include "QEIs.h"

//Function declarations
void        initGPIO(void);
//Moves robot forward and then backwards
void        move(uint8_t IN1, uint8_t IN2);
//timer to have the wheels run for a specified interval
void        wait(void);
//Returns revolutions per minute, wheel speed
uint32_t    getRPM_Wh1(void);
uint32_t    getRPM_Wh2(void);
//Indicate rotational direction
void        directionIndicate(void);
//Allow robot to turn left
void        turnLeft(void);
//Allow robot to turn right
void        turnRight(void);

//global declarations
volatile    uint32_t clock_Wh1, clock_Wh2;
volatile    uint32_t speed_Wh1, speed_Wh2;
volatile    uint32_t direction_Wh1, direction_Wh2;
volatile    uint32_t RPM_Wh1, RPM_Wh2;
const       uint32_t ppr = 63;
const       uint32_t load = 50;

const       uint32_t L = 3;
const       uint32_t r = 2;

int main(void)
{

    //initialize system clock source
    initOSC();

    //initialize GPIO ports as needed
    initGPIO();

    //initialize QEI
    initQEIs();

    //initialize PWM
    initM1PWMn();

    while(1){

        move(0, 1); //forward
        directionIndicate();
        RPM_Wh1 = getRPM_Wh1();
        RPM_Wh2 = getRPM_Wh2();
        move(1, 1); //stop

        move(1, 0); //backwards
        directionIndicate();
        RPM_Wh1 = getRPM_Wh1();
        RPM_Wh2 = getRPM_Wh2();
        move(1, 1); //stop



    }




}

void QEI0_Handler(void){

    if((QEI0[QEI_RIS] & QEI_INT_TIMER) != 0){
        QEI0[QEI_ISC] |= QEI_INT_TIMER;
        clock_Wh1 = QEI0[QEI_TIME];
        speed_Wh1 = QEI0[QEI_SPEED];
    }

    if((QEI0[QEI_RIS] & QEI_INT_DIR) != 0){
        QEI0[QEI_ISC] |= QEI_INT_DIR;
        direction_Wh1 = (QEI0[QEI_STAT] & QEI_STAT_DIR);
    }

}

void QEI1_Handler(void){

    if((QEI1[QEI_RIS] & QEI_INT_TIMER) != 0){
        QEI1[QEI_ISC] |= QEI_INT_TIMER;
        clock_Wh2 = QEI1[QEI_TIME];
        speed_Wh2 = QEI1[QEI_SPEED];
    }

    if((QEI1[QEI_RIS] & QEI_INT_DIR) != 0){
        QEI1[QEI_ISC] |= QEI_INT_DIR;
        direction_Wh2 = (QEI1[QEI_STAT] & QEI_STAT_DIR);
    }

}

void turnLeft(void){



}

void turnRight(void){



}

uint32_t getRPM_Wh1(void){

    return((clock_Wh1 * speed_Wh1 * 60)/(load * ppr * 2));

}

uint32_t getRPM_Wh2(void){

    return((clock_Wh2 * speed_Wh2 * 60)/(load * ppr * 2));

}

void directionIndicate(void){

    if((direction_Wh1 == 1) && (direction_Wh2 == 1)){
        GPIO_PORTF[(GPIO_PIN_2 | GPIO_PIN_3)] |= GPIO_PIN_2 | GPIO_PIN_3;
        for(uint32_t i=0; i<10000; i++){

        }
    }else{
        GPIO_PORTF[(GPIO_PIN_2 | GPIO_PIN_3)] &= ~(GPIO_PIN_2 | GPIO_PIN_3);
        for(uint32_t i=0; i<10000; i++){

        }
    }

}

void move(uint8_t IN1, uint8_t IN2){

    if(IN1==0 && IN2==1){
        //                                                                    PE1    |     PE3     |     PE2     |     PE5
        //                                                                RWheel IN1 |  LWheel In1 |  RWheel IN2 | LWheel IN2
        GPIO_PORTE[GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5] = ~(GPIO_PIN_1 | GPIO_PIN_5) | (GPIO_PIN_2 | GPIO_PIN_3);
        wait();

    }//move forward

    if(IN1==1 && IN2==0){

        GPIO_PORTE[GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5] = (GPIO_PIN_1 | GPIO_PIN_5) | ~(GPIO_PIN_2 | GPIO_PIN_3);
        wait();

    }//move backwards

    if(IN1==1 && IN2==1){

        GPIO_PORTE[GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5] = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5;

    }//stop

}

void wait(void){

    for(uint32_t i=0; i<2000000; i++){

    }

}



void initGPIO(void) {

    //Enable clocks to GPIO ports D and E for M1PWM1 and M1PWM2
    //PORTD PD1 for M1PWM1 and PORTE PE4 for M1PWM2
    //Additionally, PORTE PE1, PE2, PE3, and PE5, are to be configured for the digital control signals
    //Add PORTC and PORTF for QEI control
    //Pins PD3 and PC4 will read INDEX values into the QEI modules IDX0 and IDX1 respectively.
    //Pins PD6, PC5, PF1, and PC6 will read the pulses produced from the wheel encoders.
    //Respectively they are PhA0, PhA1 and PhB0, PhB1
    SYSCTL[SYSCTL_RCGCGPIO] |= SYSCTL_RCGCGPIO_PORTC | SYSCTL_RCGCGPIO_PORTD | SYSCTL_RCGCGPIO_PORTE | SYSCTL_RCGCGPIO_PORTF;
    SYSCTL[SYSCTL_RCGCGPIO] |= SYSCTL_RCGCGPIO_PORTC | SYSCTL_RCGCGPIO_PORTD | SYSCTL_RCGCGPIO_PORTE | SYSCTL_RCGCGPIO_PORTF;

    //Set direction for the input ports
    //                            PC4    |    PC5     |     PC6
    //                           IDX1    |    PhA1    |     PhB1
    GPIO_PORTC[GPIO_DIR] &= ~(GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);
    //                            PD3    |    PD6
    //                           IDX0    |    PhA0
    GPIO_PORTD[GPIO_DIR] &= ~(GPIO_PIN_3 | GPIO_PIN_6);
    //                            PF1
    //                           PhB0
    GPIO_PORTF[GPIO_DIR] &= ~GPIO_PIN_1;

    //Set direction for the output ports
    //                         PD1
    //                        M1PWM0
    GPIO_PORTD[GPIO_DIR] |= GPIO_PIN_1;
    //                          PE1    |    PE2     |     PE3    |    PE4     |     PE5
    //                      RWheel IN1 | RWheel In2 | LWheel IN1 |   M1PWM2   | LWheel IN2
    GPIO_PORTE[GPIO_DIR] |= GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    //                         PF2     |     PF3
    //                       Blue LED  |  Green LED
    GPIO_PORTF[GPIO_DIR] |= GPIO_PIN_2 | GPIO_PIN_3;

    //Set GPIO alternate function selections
    GPIO_PORTC[GPIO_AFSEL] |= GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_PORTD[GPIO_AFSEL] |= GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_6;
    GPIO_PORTE[GPIO_AFSEL] |= GPIO_PIN_4;
    GPIO_PORTF[GPIO_AFSEL] |= GPIO_PIN_1;

    //Enable digital signals to PE[1:5] and PD1 and PF[2:3]
    GPIO_PORTD[GPIO_DEN] |= GPIO_PIN_1;
    GPIO_PORTE[GPIO_DEN] |= GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_PORTF[GPIO_DEN] |= GPIO_PIN_2 | GPIO_PIN_3;

    //Configure the PMCn fields in the GPIOPCTL register to assign the PWM signals to the appropriate
    //pins.
    GPIO_PORTC[GPIO_PCTL] &= ~((0xF<<(4*4)) | (0xF<<(4*5)) | (0xF<<(4*6)));
    GPIO_PORTD[GPIO_PCTL] &= ~((0xF<<(4*1)) | (0xF<<(4*3)) | (0xF<<(4*6)));
    GPIO_PORTE[GPIO_PCTL] &= ~(0xF<<(4*4));
    GPIO_PORTF[GPIO_PCTL] &= ~(0xF<<(4*1));
    GPIO_PORTC[GPIO_PCTL] |= (0x6<<(4*4)) | (0x6<<(4*5)) | (0x6<<(4*6));
    GPIO_PORTD[GPIO_PCTL] |= (0x5<<(4*1)) | (0x6<<(4*3)) | (0x6<<(4*6));
    GPIO_PORTE[GPIO_PCTL] |= (0x5<<(4*4));
    GPIO_PORTF[GPIO_PCTL] |= (0x6<<(4*1));

}
