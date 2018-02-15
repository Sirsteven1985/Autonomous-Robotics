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
#include "driverlib\interrupt.h"
#include "driverlib\uart.h"
#include "driverlib\qei.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

void right_drive_FWD(void);
void right_drive_Back(void);
void right_brake(void);
void right_standby(void);
void left_drive_FWD(void);
void left_drive_Back(void);
void left_brake(void);
void left_standby(void);
void QEI0_handler(void);


volatile    uint32_t stat = 0;
volatile    uint32_t clock_Wh1, clock_Wh2;
volatile    uint32_t speed_Wh1, speed_Wh2;
volatile    uint32_t direction_Wh1, direction_Wh2;
volatile    uint32_t RPM_Wh1, RPM_Wh2;
const       uint32_t ppr = 63;
const       uint32_t load = 50;

const       uint32_t L = 3;
const       uint32_t r = 2;

/**
 * main.c
 */
int main(void)
{

//Set system clock to 16MHz, Utilize main oscillator
SysCtlClockSet(SYSCTL_OSC_MAIN | SYSCTL_USE_OSC | SYSCTL_XTAL_16MHZ);

// Enable PWM peripherals
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

// Enable UART Peripherals
SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

// Enable QEI Peripherals
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);


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

while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0))
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







////////////////////////////////////////////////////////////////////////////
//QEI

//Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;
HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= 0x80;
HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

IntMasterEnable();


//Set Pins to be PHA0 and PHB0
GPIOPinConfigure(GPIO_PD6_PHA0);
GPIOPinConfigure(GPIO_PD7_PHB0);

//Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7. I believe this sets the pull up and makes them inputs
GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 |  GPIO_PIN_7);

//DISable peripheral and int before configuration
QEIDisable(QEI0_BASE);
QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

// Configure quadrature encoder, use an arbitrary top limit of 1000
QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 64);

// Enable the quadrature encoder.
QEIEnable(QEI0_BASE);

//Set position to a middle value so we can see if things are working
QEIPositionSet(QEI0_BASE, 32);
QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, 1000);
QEIVelocityEnable(QEI0_BASE);

/// set up interrupt function pointer
QEIIntRegister(QEI0_BASE, QEI0_handler);


QEIIntEnable(QEI0_BASE, QEI_INTDIR);



////////////////////////////////////////////////////////////////////
//UART

GPIOPinConfigure(GPIO_PA0_U0RX);
GPIOPinConfigure(GPIO_PA1_U0TX);
GPIOPinTypeUART(GPIO_PORTA_BASE, (GPIO_PIN_0 | GPIO_PIN_1));
UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600,
     UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
     UART_CONFIG_PAR_NONE);


// Finally, we can send data.  To send a single character, such as the letter 'e,' we use:

// UARTCharPut(UART0_BASE,'e');

// To send multiple characters, such as numbers, we need to send multiple characters.  We can do this using a string and a for loop:

// sprintf(strToSend,"%d\r\n",ui32value);
// for(i = 0; (strToSend[i] != '\0'); i++)
     // UARTCharPut(UART0_BASE,strToSend[i]);

//////////////////////////////////////////////////////




    uint32_t cnt = 0;
    while(1)
    {
        //////////////////////      Drive forward      ///////////////////////
        clock_Wh1 = QEIVelocityGet(QEI0_BASE);
        right_drive_FWD();

        left_drive_Back();
       // cnt = clock_Wh1;
        for( uint32_t ii = 0; ii < 5000000 ; ii++){}
        clock_Wh1 = QEIVelocityGet(QEI0_BASE);
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
        for( uint32_t ii = 0; ii < 5000000 ; ii++){}

    }

}

void QEI0_handler(void)
{
    /* if((QEI0[QEI_RIS] & QEI_INT_TIMER) != 0){
        QEI0[QEI_ISC] |= QEI_INT_TIMER;
        clock_Wh1 = QEI0[QEI_TIME];
        speed_Wh1 = QEI0[QEI_SPEED];
    }

    if((QEI0[QEI_RIS] & QEI_INT_DIR) != 0){
        QEI0[QEI_ISC] |= QEI_INT_DIR;
        direction_Wh1 = (QEI0[QEI_STAT] & QEI_STAT_DIR);
    } */

   // stat = ( QEI_INTTIMER & QEIIntStatus(QEI0_BASE, true));

    QEIIntClear(QEI0_BASE, QEI_INTTIMER | QEI_INTDIR | QEI_INTERROR | QEI_INTINDEX);


    clock_Wh2 = QEIPositionGet(QEI0_BASE);
    speed_Wh1 = QEIDirectionGet(QEI0_BASE);


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

    // Setting the proper pins to drive the motor forward [ IN1 = 0 ; IN2 = 1 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_3) | (GPIO_PIN_5) ), ( (GPIO_PIN_3) | (GPIO_PIN_5) ));
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, ~(GPIO_PIN_4));
}

void left_drive_FWD(void)
{
    // Setting the proper pins to drive the motor in reverse [ IN1 = 1 ; IN2 = 0 ; SB = 1 ]
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

