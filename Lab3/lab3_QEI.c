// 
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "driverlib\sysctl.h"
#include "driverlib\gpio.h"
#include "driverlib\pwm.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib\pin_map.h"
#include "inc/hw_memmap.h"
#include "driverlib\qei.h"


/**
 * main.c
 */
int main(void)
{

//Set system clock to 16MHz, Utilize main oscillator
SysCtlClockSet(SYSCTL_OSC_MAIN | SYSCTL_USE_OSC | SYSCTL_XTAL_16MHZ);


SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
//SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

//
//GPIOPinTypePWM(GPIO_PORTB_BASE, ( (GPIO_PIN_6) | (GPIO_PIN_7)));
GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

//
SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
// Wait for the PWM0 module to be ready.
GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,( (GPIO_PIN_0) | (GPIO_PIN_1) | (GPIO_PIN_2)));
// Enable the PWM0 peripheral
GPIOIntEnable(GPIO_PORTB_BASE, ( (GPIO_PIN_0) | (GPIO_PIN_1) | (GPIO_PIN_2) | (GPIO_PIN_3) | (GPIO_PIN_6) | (GPIO_PIN_7)));
//
//GPIOPinConfigure( (GPIO_PB6_M0PWM0)| (GPIO_PB7_M0PWM1));
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
// microseconds. For a 16 MHz clock, this translates to 400 clock ticks.
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


////////////////////////////////////////////////////////////////////////////
//QEI

// Set the clocking to run directly from the crystal.
SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

// Enable QEI Peripherals
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);

//Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

//Set Pins to be PHA0 and PHB0
GPIOPinConfigure(GPIO_PD6_PHA0);
GPIOPinConfigure(GPIO_PD7_PHB0);

//Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7. I believe this sets the pull up and makes them inputs
GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 |  GPIO_PIN_7);

//DISable peripheral and int before configuration
QEIDisable(QEI0_BASE);
QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

// Configure quadrature encoder, use an arbitrary top limit of 1000
QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 1000);

// Enable the quadrature encoder.
QEIEnable(QEI0_BASE);

//Set position to a middle value so we can see if things are working
QEIPositionSet(QEI0_BASE, 500);





/// set up interrupt function pointer
QEIIntRegister(uint32_t ui32Base, *QEIDirectionGet(QEI0_BASE));


QEIIntEnable(QEI0_BASE, QEI_INTDIR);






///////////////////////////////















//////////////////////      Drive forward       ///////////////////////
GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_0) | (GPIO_PIN_2) ), ( (GPIO_PIN_0) | (GPIO_PIN_2) ));
GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, ~(GPIO_PIN_1));




	while(1)
	{


	}

}


