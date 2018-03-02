/*  ECPE 155 Lab 3
    QEI API version
    Authors: Steve Guerrero and Paul Vuong
    Feb 6, 2018
*/

// Libraries Used
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "inc/tm4c123gh6pm.h"
#include "C:\ti\TivaWare_C_Series-2.1.3.156\driverlib\sysctl.h"
#include "C:\ti\TivaWare_C_Series-2.1.3.156\driverlib\gpio.h"
#include "C:\ti\TivaWare_C_Series-2.1.3.156\driverlib\pwm.h"
#include "C:\ti\TivaWare_C_Series-2.1.3.156\driverlib\pin_map.h"
#include "C:\ti\TivaWare_C_Series-2.1.3.156\driverlib\interrupt.h"
#include "C:\ti\TivaWare_C_Series-2.1.3.156\driverlib\uart.h"
#include "C:\ti\TivaWare_C_Series-2.1.3.156\driverlib\qei.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

// Function Prototypes
void right_wheel_FWD(void);
void right_wheel_REV(void);
void right_brake(void);
void right_standby(void);
void left_wheel_FWD(void);
void left_wheel_REV(void);
void left_brake(void);
void left_standby(void);
void QEI0_handler(void);
void QEI1_handler(void);
void bumpSensor_handler(void);
void CW_rotate_90(void);
void robot_FWD(void);
void FWD_1_foot(void);

// Global Variables
volatile    uint32_t cnt = 0;
volatile    uint32_t stat = 0;
volatile    uint32_t clock_Wh1, clock_Wh2;
volatile    uint32_t speed_Wh1, speed_Wh2;
volatile    int32_t direction_Wh1, direction_Wh2;
volatile    uint32_t POS_Wh1, POS_Wh2;
volatile    uint32_t IND_Wh1, IND_Wh2;
volatile    uint32_t R_SPD, L_SPD;
const       uint32_t ppr = 63;
const       uint32_t load = 50;
const       float L = 4.65;
const       float r = 2;

//Sensors
volatile    uint32_t r_n_l_bumpSensors;

/**
* main.c
*/
int main(void)
{
    //Set system clock to 16MHz, Utilize main oscillator
    SysCtlClockSet(SYSCTL_OSC_MAIN | SYSCTL_USE_OSC | SYSCTL_XTAL_16MHZ);

    // Enable GPIO peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Enable PWM peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    // Enable UART Peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Enable QEI Peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
    //

    /////////////////////////////////    GPIO     ///////////////////////////////////////////
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,(GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3)); // LEDS
    //GPIOIntEnable(GPIO_PORTF_BASE, (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3));
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,(GPIO_PIN_1 | GPIO_PIN_2)); //bump switches
    // Make pins 2 and 4 rising edge triggered interrupts.
    //
    GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2, GPIO_FALLING_EDGE);
    // Enable the pin interrupts.
    //
    GPIOIntRegister(GPIO_PORTD_BASE, bumpSensor_handler);
    GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2);
    // Read some pins.
    //
    r_n_l_bumpSensors = GPIOPinRead(GPIO_PORTD_BASE,(GPIO_PIN_1 | GPIO_PIN_2));


    /////////////////////////////////    PWM     ///////////////////////////////////////////
    GPIOPinTypePWM(GPIO_PORTB_BASE, ( (GPIO_PIN_6) | (GPIO_PIN_7)));

    SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
    // Wait for the PWM0 module to be ready.
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,( (GPIO_PIN_0) | (GPIO_PIN_1) | (GPIO_PIN_2) | (GPIO_PIN_3)  | (GPIO_PIN_4) | (GPIO_PIN_5) ) );
    // Enable the PWM0 peripheral
    GPIOIntEnable(GPIO_PORTB_BASE, ( (GPIO_PIN_0) | (GPIO_PIN_1) | (GPIO_PIN_2) | (GPIO_PIN_3)  | (GPIO_PIN_4) | (GPIO_PIN_5) | (GPIO_PIN_6) | (GPIO_PIN_7) ) );
    //
    GPIOPinConfigure( GPIO_PB6_M0PWM0 );   // Located in PinMap.h
    GPIOPinConfigure( GPIO_PB7_M0PWM1 );   // Located in PinMap.h

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

    // initializing the speed to 300/400 = 75%
    R_SPD = 250;
    L_SPD = 250;
    // Set the load value of PWM0 generator
    PWMGenPeriodSet(PWM_BASE_0, PWM_GEN_0, 400);
    //
    // Set the pulse width of PWM0
    PWMPulseWidthSet(PWM_BASE_0, PWM_OUT_0, R_SPD);   // Right Wheel Speed Control
    //
    // Set the load value of PWM1 generator
    PWMGenPeriodSet(PWM_BASE_0, PWM_GEN_1, 400);
    //
    // Set the pulse width of PWM1
    PWMPulseWidthSet(PWM_BASE_0, PWM_OUT_1, L_SPD);   // Left Wheel Speed Control
    //
    // Enable Gen 1 and Gen 0
    PWMGenEnable(PWM_BASE_0, PWM_GEN_0);
    //PWMGenEnable(PWM_BASE_0, PWM_GEN_0 | PWM_GEN_1);
    //
    // Enable the outputs
    PWMOutputState(PWM_BASE_0, (PWM_OUT_0_BIT | PWM_OUT_1_BIT), true);

    /////////////////////////////////    QEI     /////////////////////////////////////////////

    //Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    //Set Pins to be PHA0 and PHB0 (Module 0 Phase A and Phase B)
    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);

    //Set Pins to be PHA1 and PHB1 (Module 1 Phase A and Phase B)
    GPIOPinConfigure(GPIO_PC5_PHA1);
    GPIOPinConfigure(GPIO_PC6_PHB1);

    //Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7. I believe this sets the pull up and makes them inputs
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 |  GPIO_PIN_7);  // Left wheel QEI
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 |  GPIO_PIN_6);  // Right wheel QEI

    //Disable peripheral and interrupts before configuration
    QEIDisable(QEI0_BASE);
    QEIDisable(QEI1_BASE);

    QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
    QEIIntDisable(QEI1_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

    IntMasterEnable();  // Allows for interrupts to occur

    // Configure quadrature encoder, use an arbitrary top limit of 1000
    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 127);
    QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 127);

    // Enable the quadrature encoder.
    QEIEnable(QEI0_BASE);
    QEIEnable(QEI1_BASE);

    //Set position to a middle value so we can see if things are working
    QEIPositionSet(QEI0_BASE, 0);
    QEIPositionSet(QEI1_BASE, 0);

    uint32_t Clk_period = SysCtlClockGet();

    // Using SYSTEM Clock to get a 1 second period for velocity
    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_2, Clk_period);
    QEIVelocityEnable(QEI0_BASE);

    // Using SYSTEM Clock to get a 1 second period for velocity
    QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_2, Clk_period);
    QEIVelocityEnable(QEI1_BASE);

    /// set up interrupt function pointer
    QEIIntRegister(QEI0_BASE, QEI0_handler);
    QEIIntEnable(QEI0_BASE, QEI_INTINDEX);

    /// set up interrupt function pointer
    QEIIntRegister(QEI1_BASE, QEI1_handler);
    QEIIntEnable(QEI1_BASE, QEI_INTINDEX);

    ////////////////////////////////////////////////////////////////////
    //UART

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, (GPIO_PIN_0 | GPIO_PIN_1));
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);


    // Drive straight forward
    //robot_FWD();


    // To send multiple characters, such as numbers, we need to send multiple characters.  We can do this using a string and a for loop:
     // UART EXAMPLE CODE FOR POSITION
     char strToSend[8];
     uint32_t i = 0;
//     POS_Wh1 = QEIPositionGet(QEI0_BASE);
//     sprintf(strToSend,"%d\r\n",POS_Wh1);
//     for(i = 0; (strToSend[i] != '\0'); i++)
//     UARTCharPut(UART0_BASE,strToSend[i]);
     IND_Wh1 = 0;
     IND_Wh2 = 0;
    //////////////////////////////////////////////////////
    //////////////////////      MAIN LOOP      ///////////////////////
    while(1)
    {
        uint32_t ii;

        //CW_rotate_90();
        //right_brake();
        //left_brake();
        //for(ii = 0; ii < 1000000; ii++){}
        //FWD_1_foot();
        //right_brake();
        //left_brake();
        //for(ii = 0; ii < 1000000; ii++){}
        if(!r_n_l_bumpSensors){
            //blink leds when bumpers make contact
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1, GPIO_PIN_1);
            for(i = 0; i < 150000; i++)
            {}
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1, ~GPIO_PIN_1);
            for(i = 0; i < 150000; i++)
            {}
        }else{
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1, ~GPIO_PIN_1);
        }





    }

}
//////////////////////      PROCEDURES / HANDLERS     ///////////////////////
//volatile uint8_t QEI_INT_INDEX_Flag1;
//volatile uint8_t QEI_INT_INDEX_Flag0;

void QEI0_handler(void) // Left Wheel
{
    QEIIntClear(QEI0_BASE, QEI_INTTIMER | QEI_INTDIR | QEI_INTERROR | QEI_INTINDEX);

    //POS_Wh1 = QEIPositionGet(QEI0_BASE);
    //direction_Wh1 = QEIDirectionGet(QEI0_BASE);
    IND_Wh1 = IND_Wh1 + 1;
}

void QEI1_handler(void) // Left Wheel
{
    QEIIntClear(QEI1_BASE, QEI_INTTIMER | QEI_INTDIR | QEI_INTERROR | QEI_INTINDEX);

    //POS_Wh1 = QEIPositionGet(QEI0_BASE);
    //direction_Wh1 = QEIDirectionGet(QEI0_BASE);
    IND_Wh2 = IND_Wh2 + 1;
}

void bumpSensor_handler(void){

    GPIOIntClear(GPIO_PORTD_BASE,(GPIO_PIN_1 | GPIO_PIN_2));

}

void CW_rotate_90(void)
{
    cnt = 0;
    QEIPositionSet(QEI0_BASE, 0);
    QEIPositionSet(QEI1_BASE, 0);
    left_wheel_FWD();
    right_wheel_REV();

    while((POS_Wh1 != 50) && (POS_Wh2 != 50))
    {
        POS_Wh1 = QEIPositionGet(QEI0_BASE);
        POS_Wh2 = QEIPositionGet(QEI1_BASE);

        if(POS_Wh1 == 50)
        {
           left_brake();
           QEIPositionSet(QEI0_BASE, 0);
           cnt = cnt + 1;
        }
        if(POS_Wh2 == 50)
        {
           right_brake();
           QEIPositionSet(QEI1_BASE, 0);
           cnt = cnt + 1;
        }
        if (cnt == 2)
        {
           left_brake();
           right_brake();
           break;
        }
    }
}
void CCW_rotate_90(void)
{
    cnt = 0;
    QEIPositionSet(QEI0_BASE, 0);
    QEIPositionSet(QEI1_BASE, 0);
    left_wheel_REV();
    right_wheel_FWD();
    while((POS_Wh1 != 50) && (POS_Wh2 != 50))
    {
        POS_Wh1 = QEIPositionGet(QEI0_BASE);
        POS_Wh2 = QEIPositionGet(QEI1_BASE);

        if(POS_Wh1 == 50)
        {
           left_brake();
           QEIPositionSet(QEI0_BASE, 0);
           cnt = cnt + 1;
        }
        if(POS_Wh2 == 50)
        {
           right_brake();
           QEIPositionSet(QEI1_BASE, 0);
           cnt = cnt + 1;
        }
        if (cnt == 2)
        {
           cnt = 0;
           break;
        }
    }
}
void FWD_1_foot(void)
{
    cnt = 0;
    QEIPositionSet(QEI0_BASE, 0);
    QEIPositionSet(QEI1_BASE, 0);
    left_wheel_FWD();
    right_wheel_FWD();
    IND_Wh1 = 0;
    IND_Wh2 = 0;
    POS_Wh1 = QEIPositionGet(QEI0_BASE);
    POS_Wh2 = QEIPositionGet(QEI1_BASE);

    while(cnt != 2)
    {
        POS_Wh1 = QEIPositionGet(QEI0_BASE);
        POS_Wh2 = QEIPositionGet(QEI1_BASE);

        if(POS_Wh1 == 127)
        {
            IND_Wh1 = 1;
        }
        if(POS_Wh2 == 1)
        {
            IND_Wh2 = 1;
        }

        if((POS_Wh1 == 90) && (IND_Wh1 == 1))
        {
            left_brake();
            QEIPositionSet(QEI0_BASE, 0);
            cnt = cnt + 1;
        }
        if((POS_Wh2 == (127-90)) && (IND_Wh2 == 1))
        {
            right_brake();
            QEIPositionSet(QEI1_BASE, 0);
            cnt = cnt + 1;
        }

    }
}
void robot_FWD(void)
{
    //////////////////////      Drive straight forward      ///////////////////////
    right_wheel_FWD();
    left_wheel_FWD();
}

void right_wheel_FWD(void)
{
    /*      REFERENCE TABLE     */
    //  IN1 --> PIN_1
    //  IN2 --> PIN_2
    //  STANDBY --> PIN_0

    // Setting the proper pins to drive the motor forward [ IN1 = 0 ; IN2 = 1 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_0) | (GPIO_PIN_2) ), ( (GPIO_PIN_0) | (GPIO_PIN_2) ));
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, ~(GPIO_PIN_1));
}

void right_wheel_REV(void)
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

void left_wheel_FWD(void)
{   /*      REFERENCE TABLE     */
    //  IN1 --> PIN_5
    //  IN2 --> PIN_4
    //  STANDBY --> PIN_3

    // Setting the proper pins to drive the motor in reverse [ IN1 = 1 ; IN2 = 0 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_3) | (GPIO_PIN_4) ), ( (GPIO_PIN_3) | (GPIO_PIN_4) ));
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, ~(GPIO_PIN_5));
}

void left_wheel_REV(void)
{
    // Setting the proper pins to drive the motor forward [ IN1 = 0 ; IN2 = 1 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_3) | (GPIO_PIN_5) ), ( (GPIO_PIN_3) | (GPIO_PIN_5) ));
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, ~(GPIO_PIN_4));
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

//////////////////////      Recycle Bin      ///////////////////////

//      for( uint32_t ii = 0; ii < 8000000 ; ii++){}
//        //////////////////////      Brake      ///////////////////////
//        speed_Wh1 = QEIVelocityGet(QEI0_BASE);  //  measure speed

        //for( uint32_t ii = 0; ii < 1000000 ; ii++){}


//        sprintf(strToSend,"%d\r\n",speed_Wh1);
//        for(i = 0; (strToSend[i] != '\0'); i++)
//        UARTCharPut(UART0_BASE,strToSend[i]);
    //    for( uint32_t ii = 0; ii < 1000000 ; ii++){}

//      right_brake();
//      left_brake();
//
//      for( uint32_t ii = 0; ii < 2000000 ; ii++){}
//        //////////////////////      Drive Backwards      ///////////////////////
//      right_wheel_REV();
//      left_wheel_REV();
//
//      for( uint32_t ii = 0; ii < 5000000 ; ii++){}
//
//      speed_Wh1 = QEIVelocityGet(QEI0_BASE); //   measure speed
