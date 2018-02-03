
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "driverlib\sysctl.h"
#include "driverlib\gpio.h"
#include "driverlib\pwm.h"
#include "driverlib\pin_map.h"
#include "inc/hw_memmap.h"


/**
 * main.c
 */
int main(void)
{

//Set system clock to 20MHz, Utilize main oscillator
SysCtlClockSet(SYSCTL_OSC_MAIN | SYSCTL_USE_OSC | SYSCTL_XTAL_16MHZ);

//Set GPIO
// sysctlpwmclockset


SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
//
GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

//
SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
// Wait for the PWM0 module to be ready.
//GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_6);
// Enable the PWM0 peripheral
///GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_6);
//
GPIOPinConfigure(GPIO_PB6_M0PWM0);


while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))
{ }
//
// Configure the PWM generator for count down mode with immediate updates
// to the parameters.
//PB_6_m0
PWMGenConfigure(PWM_BASE_0, PWM_GEN_0,
PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
//
// Set the period. For a 50 KHz frequency, the period = 1/50,000, or 20
// microseconds. For a 20 MHz clock, this translates to 400 clock ticks.
// Use this value to set the period.
//
PWMGenPeriodSet(PWM_BASE_0, PWM_GEN_0, 400);
//
// Set the pulse width of PWM0 for a 25% duty cycle.
//
PWMPulseWidthSet(PWM_BASE_0, PWM_OUT_0, 200);
//
// Set the pulse width of PWM1 for a 75% duty cycle.
//
//PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, 300);
//
// Start the timers in generator 0.
//
PWMGenEnable(PWM_BASE_0, PWM_GEN_0);
//
// Enable the outputs.
//
PWMOutputState(PWM_BASE_0, (PWM_OUT_0_BIT | PWM_OUT_1_BIT), true);

	while(1)
	{

	}

}
