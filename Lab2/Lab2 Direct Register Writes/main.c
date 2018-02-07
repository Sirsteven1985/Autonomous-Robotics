

/**
 * ECPE 155 Autonomous Robotics
 * Spring 2018
 * Lab 2 - Pulse Width Modulation (PWM)
 * Lab Associates:  Paul Vuong
 *                  Steve Guerro
 * 1.   To drive the HUB-ee motors/wheels, a PWM signal supplies an average potential to the motors.
 *      The higher the average potential (the greater the pulse width) the faster the motor spins.
 * 2.   PWM module 1 Generator 0 (M1PWM1 Load/Generator/Comparator B,register bitfields) will supply a signal to the right wheel from PD1.
 * 3.   PWM module 1 Generator 1 (M1PWM1 Generator/Comparator B) will supply a signal to the left wheel.
 * 4.   In addition to the PWM dead-band signals, two additional digital control signals
 *      will give added functionality to the wheels.
 * 5.   Digital signals will be sourced from the many GPIO ports as a digital output
 * 6.   The additional digital inputs will allow the motor to move forwards, backwards and stop.
 */


#include <stdbool.h>
#include <stdint.h>
#include "SYSCTL.h"
#include "GPIO.h"
#include "OSC.h"
#include "M1PWMn.h"
#include "MnPWMn.h"

// Control interface system prototypes initialization
void initGPIO(void);
void move(uint32_t IN1, uint32_t IN2); //moves robot forward and then backwards
void wait(void); //wait timer

int main(void)
{
    //Initiate Oscillator. Direct register writes to RCC and RCC2 register.
    //Establish system clock and enable both system clocks and PWM clock.
    //Use of PWM module 1 will require register writes to RCGCPWM register.
    initOSC();
    //Initiate GPIOs. Enable clocks to ports D and E (RCGCGPIO). PD1 and PE4 will
    //be used for the PWM output pins while PE1, PE2, PE3, and PE5 are the digital
    //signals out to the HUBee wheels.
    initGPIO();
    //Initiate PWM module. Assign clock signal to PWM module (RCGCPWM).
    //1. Load start value into PWMnCTL register
    //2. Set pulse width values in PWMnGENA and PWMnGENB registers
    //3. From PWM clock signal determine the amount of clock ticks and load value
    //   into PWMnLOAD register.
    //4. Set duty cycles in PWMnCMPA and PWMnCMPB register
    //5. Start the timers in PWMnCTL register.
    //6. Enable PWM outputs PWMENABLE register.
    initM1PWMn();

    while(1){

//        move(0, 1); //forward
//        wait();
//        move(1, 1); //stop
//        wait();
//        move(1, 0); //backwards
//        wait();
//        move(1, 1); //stop
//        wait();

    }//end main while loop

}

void move(uint32_t IN1, uint32_t IN2){

    if(IN1==0 && IN2==1){
        GPIO_PORTE[GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5] = ~GPIO_PIN_1 | GPIO_PIN_2 | ~GPIO_PIN_3 | GPIO_PIN_5;
    }//move forward

    if(IN1==1 && IN2==0){
        GPIO_PORTE[GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5] = GPIO_PIN_1 | ~GPIO_PIN_2 | GPIO_PIN_3 | ~GPIO_PIN_5;
    }//move backwards

    if(IN1==1 && IN2==1){
        GPIO_PORTE[GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5] = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5;
    }//stop

}

void wait(void){

    for(uint32_t i=10000; i<10000; i++){

    }

}

void initGPIO(void) {

    //Enable clocks to GPIO ports D and E for M1PWM1 and M1PWM2
    //PORTD PD1 for M1PWM1 and PORTE PE4 for M1PWM2
    //Additionally, PORTE PE1, PE2, PE3, and PE5, are to be configured for the digital control signals
    SYSCTL[SYSCTL_RCGCGPIO] |= SYSCTL_RCGCGPIO_PORTD | SYSCTL_RCGCGPIO_PORTE;
    SYSCTL[SYSCTL_RCGCGPIO] |= SYSCTL_RCGCGPIO_PORTD | SYSCTL_RCGCGPIO_PORTE;

    //Set direction for the ports, all ports will serve as outputs
    //                         PD1
    //                        M1PWM0
    GPIO_PORTD[GPIO_DIR] |= GPIO_PIN_1;
    //                          PE1    |    PE2     |     PE3    |    PE4     |     PE5
    //                      RWheel IN1 | RWheel In2 | LWheel IN1 |   M1PWM2   | LWheel IN2
    GPIO_PORTE[GPIO_DIR] |= GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;

    //Set GPIO alternate function selections
    GPIO_PORTD[GPIO_AFSEL] |= GPIO_PIN_1;       //PD1 M1PWM1, Generator 0
    GPIO_PORTE[GPIO_AFSEL] |= GPIO_PIN_5;       //PE4 M1PWM2, Generator 1

    //Enable digital signals to PE[1:5] and PD1
    GPIO_PORTD[GPIO_DEN] |= GPIO_PIN_1;
    GPIO_PORTE[GPIO_DEN] |= GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;

    //Configure the PMCn fields in the GPIOPCTL register to assign the PWM signals to the appropriate
    //pins. PMC5 to both M1PWM1 and M1PWM2.
    GPIO_PORTD[GPIO_PCTL] &= ~(0xF<<(4*1));
    GPIO_PORTE[GPIO_PCTL] &= ~(0xF<<(4*5));
    GPIO_PORTD[GPIO_PCTL] |= (0x5<<(4*1));
    GPIO_PORTE[GPIO_PCTL] |= (0x5<<(4*5));

}

