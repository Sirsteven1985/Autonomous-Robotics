

/**
 * ECPE 155 Autonomous Robotics
 * Spring 2018
 * Lab 2 - Pulse Width Modulation (PWM)
 * Lab Associates:  Paul Vuong
 *                  Steve Guerro
 * 1.   To drive the HUB-ee motors/wheels, the PWM modules need to be configured for
 *      dead-band generation.
 * 2.   PWM Generator 1 will supply a signal to the right wheel.
 * 3.   PWM Generator 2 will supply a signal to the left wheel.
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
#include "PWM.h"
#include "M1PWMn.h"

// Control interface system prototypes initialization
void initGPIO(void);
void moveForward(uint32_t in1, uint32_t in2);

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

        moveForward(0, 1);
        wait();
        moveForward(1, 1);
        wait();

    }//end main while loop

}


void initGPIO(void) {

    //Enable clocks to GPIO ports D and E for M1PWM1 and M1PWM2
    //PORTD PD1 for M1PWM1 and PORTE PE4 for M1PWM2
    //Additionally, PORTE PE1, PE2, PE3, and PE5, are to be configured for the digital control signals
    SYSCTL[SYSCTL_RCGCGPIO] |= SYSCTL_RCGCGPIO_PORTD | SYSCTL_RCGCGPIO_PORTE;
    SYSCTL[SYSCTL_RCGCGPIO] |= SYSCTL_RCGCGPIO_PORTD | SYSCTL_RCGCGPIO_PORTE;

    //Set direction for the ports, all ports will serve as outputs
    GPIO_PORTD[GPIO_DIR] |= GPIO_PIN_1;
    GPIO_PORTE[GPIO_DIR] |= GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;

    //Set GPIO alternate function selections
    GPIO_PORTD[GPIO_AFSEL] |= GPIO_PIN_1;
    GPIO_PORTE[GPIO_AFSEL] |= GPIO_PIN_4;

    //Enable digital signals to PE1, PE2, PE3, and PE5
    GPIO_PORTE[GPIO_DEN] |= GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5;

    //Configure the PMCn fields in the GPIOPCTL register to assign the PWM signals to the appropriate
    //pins
    GPIO_PORTD[GPIO_PCTL] = (GPIO_PORTD[GPIO_PCTL] &= ~(0xF<<(4*1))) | (0x5<<(4*1));
    GPIO_PORTE[GPIO_PCTL] = (GPIO_PORTE[GPIO_PCTL] &= ~(0xF<<(4*4))) | (0x5<<(4*4));

}
