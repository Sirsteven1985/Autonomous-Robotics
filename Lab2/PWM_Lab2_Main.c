/*  ECPE 155 Lab 2
    PWM Motor movement API version
    Authors: Steve Guerrero and Paul Vuong
    Feb 6, 2018
*/
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "driverlib\sysctl.h"
#include "driverlib\gpio.h"
#include "driverlib\pwm.h"
#include "driverlib\pin_map.h"
#include "inc/hw_memmap.h"
void right_drive_FWD(void);
void right_drive_Back(void);
void right_brake(void);
void right_standby(void);
void left_drive_FWD(void);
void left_drive_Back(void);
void left_brake(void);
void left_standby(void);


/**
 * main.c
 */
int main(void)
{

//Set system clock to 16MHz, Utilize main oscillator
SysCtlClockSet(SYSCTL_OSC_MAIN | SYSCTL_USE_OSC | SYSCTL_XTAL_16MHZ);


SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);


//
GPIOPinTypePWM(GPIO_PORTB_BASE, ( (GPIO_PIN_6) | (GPIO_PIN_7)));


//
SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
// Wait for the PWM0 module to be ready.
GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,( (GPIO_PIN_0) | (GPIO_PIN_1) | (GPIO_PIN_2) | (GPIO_PIN_3)  | (GPIO_PIN_4) | (GPIO_PIN_5) ) );
// Enable the PWM0 peripheral
GPIOIntEnable(GPIO_PORTB_BASE, ( (GPIO_PIN_0) | (GPIO_PIN_1) | (GPIO_PIN_2) | (GPIO_PIN_3)  | (GPIO_PIN_4) | (GPIO_PIN_5) | (GPIO_PIN_6) | (GPIO_PIN_7) ) );
//
GPIOPinConfigure( GPIO_PB6_M0PWM0 );   // Located in PinMap.h
GPIOPinConfigure( GPIO_PB7_M0PWM1 ); // Located in PinMap.h

while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))
{ }
//
// Configure the PWM generator for count down mode with immediate updates
// to the parameters.
//PB_6_m0
PWMGenConfigure(PWM_BASE_0, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
PWMGenConfigure(PWM_BASE_0, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
//
// Set the period. For a 50 KHz frequency, the period = 1/50,000, or 20
// microseconds. For a 20 MHz clock, this translates to 400 clock ticks.
// Use this value to set the period.
//
PWMGenPeriodSet(PWM_BASE_0, PWM_GEN_0, 400);
//
// Set the pulse width of PWM0 for a 25% duty cycle.
//
PWMPulseWidthSet(PWM_BASE_0, PWM_OUT_0, 300);
//
// Set the pulse width of PWM1 for a 75% duty cycle.
//
//PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, 300);

PWMGenPeriodSet(PWM_BASE_0, PWM_GEN_1, 400);
//
// Set the pulse width of PWM0 for a 25% duty cycle.
//
PWMPulseWidthSet(PWM_BASE_0, PWM_OUT_1, 300);
//
// Set the pulse width of PWM1 for a 75% duty cycle.
//
// Start the timers in generator 0.
//
PWMGenEnable(PWM_BASE_0, PWM_GEN_0);
//
// Enable the outputs.
//
PWMOutputState(PWM_BASE_0, (PWM_OUT_0_BIT | PWM_OUT_1_BIT), true);



//////////////////////      Drive forward      ///////////////////////

right_drive_FWD();

left_drive_Back();

for( uint32_t ii = 0; ii < 5000000 ; ii++){}

right_brake();
left_brake();
//left_drive_FWD();
//
//for( uint32_t ii = 0; ii < 2000000 ; ii++){}

//right_brake();
//left_brake();
//
for( uint32_t ii = 0; ii < 2000000 ; ii++){}
right_drive_Back();
left_drive_FWD();
//right_drive_FWD();
	while(1)
	{


	}

}

void right_drive_FWD(void)
{
    /*      REFERENCE TABLE     */
    //  IN1 --> PIN_1
    //  IN2 --> PIN_2
    //  STANDBY --> PIN_0

    // Setting the proper pins to drive the motor forward [ IN1 = 0 ; IN2 = 1 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_0) | (GPIO_PIN_2) ), ( (GPIO_PIN_0) | (GPIO_PIN_2) ));
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, ~(GPIO_PIN_1));
}

void right_drive_Back(void)
{
    // Setting the proper pins to drive the motor in reverse [ IN1 = 1 ; IN2 = 0 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_0) | (GPIO_PIN_1) ), ( (GPIO_PIN_0) | (GPIO_PIN_1) ));
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, ~(GPIO_PIN_2));
}

void right_brake(void)
{
    // Setting the proper pins to stop the motors  [ IN1 = 1 ; IN2 = 1 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_0) | (GPIO_PIN_1)| (GPIO_PIN_2) ), ( (GPIO_PIN_0) | (GPIO_PIN_1)| (GPIO_PIN_2)));

}

void right_standby(void)
{
    // Setting the proper pins to stop the motors  [ IN1 = 0 ; IN2 = 0 ; SB = 0 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_0) | (GPIO_PIN_1)| (GPIO_PIN_2) ), ( ~(GPIO_PIN_0) | (GPIO_PIN_1)| (GPIO_PIN_2)));

}

void left_drive_Back(void)
{
    /*      REFERENCE TABLE     */
    //  IN1 --> PIN_5
    //  IN2 --> PIN_4
    //  STANDBY --> PIN_3

    // Setting the proper pins to drive the motor in reverse [ IN1 = 0 ; IN2 = 1 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_3) | (GPIO_PIN_5) ), ( (GPIO_PIN_3) | (GPIO_PIN_5) ));
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, ~(GPIO_PIN_4));
}

void left_drive_FWD(void)
{
    // Setting the proper pins to drive the motor forward [ IN1 = 1 ; IN2 = 0 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_3) | (GPIO_PIN_4) ), ( (GPIO_PIN_3) | (GPIO_PIN_4) ));
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, ~(GPIO_PIN_5));
}

void left_brake(void)
{
    // Setting the proper pins to stop the motors  [ IN1 = 1 ; IN2 = 1 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_3) | (GPIO_PIN_4)| (GPIO_PIN_5) ), ( (GPIO_PIN_3) | (GPIO_PIN_4)| (GPIO_PIN_5)));

}

void left_standby(void)
{
    // Setting the proper pins to stop the motors  [ IN1 = 0 ; IN2 = 0 ; SB = 0 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_3) | (GPIO_PIN_4)| (GPIO_PIN_5) ), ( ~(GPIO_PIN_3) | (GPIO_PIN_4)| (GPIO_PIN_5)));

}

