

/**
 * main.c
 * ECPE 155 Autonomous Robotics
 * Spring 2018
 * Lab 4 - Bump Sensor
 * Lab Associates:  Paul Vuong
 *                  Steve Guerrero
 * 1.   Install rocker switches on robot for use as a bump sensor.
 * 2.   Design and implement basic obstacle avoidance.
 * 3.   After attaching the bump sensors, create source code to have LEDs blink in a particular pattern
 *      for each engagement of the sensors with the use of ISRs.
 * 4.   Once the sensors are functional with the results of the LEDs, create source code to have the
 *      robot adjust its direction once it impacts a surface with the bumper.
 */

// Libraries Used
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
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
#include "motor_control.h"

// Global Variables
volatile    uint32_t cnt = 0;
//volatile    uint32_t stat = 0;
volatile    uint32_t speed_Wh1, speed_Wh2;
volatile    int32_t direction_Wh1, direction_Wh2;
volatile    uint32_t POS_Wh1, POS_Wh2;
volatile    uint32_t IND_Wh1, IND_Wh2;
volatile    uint32_t L_SPD = 7500;
volatile    uint32_t R_SPD = 7500;

// Constant
const       float L = 4.65;
const       float L_rad = 2.93;
const       float R_rad = 2.9;


// Bump Switches
volatile    uint32_t r_bumpSensors = 0x4;
volatile    uint32_t l_bumpSensors = 0x2;

uint32_t ii = 0;
uint32_t i = 0;

int main(void)
{
    initSYSCTL();
    initPWM0();
    initQEI();
    initUART0();
    initGPIO();
    initSysTick();

    //To send multiple characters, such as numbers, we need to send multiple characters.  We can do this using a string and a for loop:
    //UART EXAMPLE CODE FOR POSITION
    //char strToSend[8];
    //uint32_t i = 0;
    //POS_Wh1 = QEIPositionGet(QEI0_BASE);
    //sprintf(strToSend,"%d\r\n",POS_Wh1);
    //for(i = 0; (strToSend[i] != '\0'); i++)
    //UARTCharPut(UART0_BASE,strToSend[i]);

    IND_Wh1 = 0;
    IND_Wh2 = 0;

    speed_Wh1 = QEIVelocityGet(QEI0_BASE);
    speed_Wh2 = QEIVelocityGet(QEI1_BASE);
    ////////////////////////////////////////////////////////////////////////////////

    while(1)
    {
        speed_Wh1 = QEIVelocityGet(QEI0_BASE);
        speed_Wh2 = QEIVelocityGet(QEI1_BASE);
        //PD_control();
        //CW_90();

        //FWD_1_foot();

        all_FWD();
        //all_wheel_REV();


    }
}
/**************************************HANDLERS**************************************/
void QEI0_handler(void){ // Left Wheel

    QEIIntClear(QEI0_BASE, (QEI_INTTIMER | QEI_INTDIR | QEI_INTERROR | QEI_INTINDEX));

    //POS_Wh1 = QEIPositionGet(QEI0_BASE);
    //direction_Wh1 = QEIDirectionGet(QEI0_BASE);
    IND_Wh1 = IND_Wh1 + 1;
}

void QEI1_handler(void){ // Left Wheel

    QEIIntClear(QEI1_BASE, (QEI_INTTIMER | QEI_INTDIR | QEI_INTERROR | QEI_INTINDEX));

    //POS_Wh1 = QEIPositionGet(QEI0_BASE);
    //direction_Wh1 = QEIDirectionGet(QEI0_BASE);
    IND_Wh2 = IND_Wh2 + 1;
}

void bumpSensor_handler(void){
    GPIOIntClear(GPIO_PORTD_BASE,(GPIO_PIN_2 | GPIO_PIN_3));
    l_bumpSensors = GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_2);
    r_bumpSensors = GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_3);
    GPIOIntDisable(GPIO_PORTD_BASE, (GPIO_PIN_2 | GPIO_PIN_3));
    bumper_function();
    //SysTick_Wait(100);
}

void bumper_function(void){

    if(r_bumpSensors == 0){
        //blink leds when bumpers make contact PD2 right switch
        GPIOPinWrite(GPIO_PORTF_BASE,(GPIO_PIN_1 | GPIO_PIN_2), (GPIO_PIN_1 | ~GPIO_PIN_2));
        //reverse
        all_REV();
        SysTick_Wait(16000000);
        SysTick_Wait(16000000);
        //ccw rotate
        CCW_90();
    }
    if(l_bumpSensors == 0){
        //blink leds when bumpers make contact PD1 left switch
        GPIOPinWrite(GPIO_PORTF_BASE,(GPIO_PIN_1 | GPIO_PIN_2), (~GPIO_PIN_1 | GPIO_PIN_2));
        //reverse
        all_REV();
        SysTick_Wait(16000000);
        SysTick_Wait(16000000);
        //cw rotate
        CW_90();
    }else if((r_bumpSensors == 0) && (l_bumpSensors == 0)){
        //blink leds when bumpers make contact
        GPIOPinWrite(GPIO_PORTF_BASE,(GPIO_PIN_1 | GPIO_PIN_2), (GPIO_PIN_1 | GPIO_PIN_2));
        // Reverse
        all_REV();
        SysTick_Wait(16000000);
        SysTick_Wait(16000000);
        //cw rotate
        CW_90();
        CW_90();
    }else{

        GPIOPinWrite(GPIO_PORTF_BASE,(GPIO_PIN_1 | GPIO_PIN_2), ~(GPIO_PIN_1 | GPIO_PIN_2));
    }
    GPIOIntEnable(GPIO_PORTD_BASE, (GPIO_PIN_2 | GPIO_PIN_3));
    GPIOPinWrite(GPIO_PORTF_BASE,(GPIO_PIN_1 | GPIO_PIN_2), ~(GPIO_PIN_1 | GPIO_PIN_2));

}
// SYSCTL initialization
void initSYSCTL(void){

    //SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_25MHZ);
    //Set system clock to 80MHz, Utilize main oscillator
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

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
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI1))
     { }

}
// GPIO initialization
void initGPIO(void){

    // LEDS
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,(GPIO_PIN_1 | GPIO_PIN_2));

    // bump switches
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,(GPIO_PIN_2 | GPIO_PIN_3));

    // Make pins 1 and 2 LOW LEVEL edge triggered interrupts.
    GPIOIntTypeSet(GPIO_PORTD_BASE, (GPIO_PIN_2 | GPIO_PIN_3), GPIO_LOW_LEVEL);

    // Make pins
    GPIOPadConfigSet(GPIO_PORTD_BASE, (GPIO_PIN_2 | GPIO_PIN_3), GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIO_PORTD_AMSEL_R &= ~0x6;      // 3) disable analog on PD1 & PD2

    // Clear interrupts
    GPIOIntClear(GPIO_PORTD_BASE,(GPIO_PIN_2 | GPIO_PIN_3));

    // Enable the pin interrupts.
    GPIOIntRegister(GPIO_PORTD_BASE, bumpSensor_handler);

    // Enable interrupts
    GPIOIntEnable(GPIO_PORTD_BASE, (GPIO_PIN_2 | GPIO_PIN_3));
    // Allows for interrupts to occur
    IntMasterEnable();

}
// PWM0 initialization
void initPWM0(void){

    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    // Wait for the PWM0 module to be ready.
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,( (GPIO_PIN_0) | (GPIO_PIN_1) | (GPIO_PIN_2) | (GPIO_PIN_3)  | (GPIO_PIN_4) | (GPIO_PIN_5) ) );
    // Enable the PWM0 peripheral
    //
    GPIOPinConfigure( GPIO_PB6_M0PWM0 );   // Located in PinMap.h
    GPIOPinConfigure( GPIO_PB7_M0PWM1 );   // Located in PinMap.h
    GPIOPinTypePWM(GPIO_PORTB_BASE, ( (GPIO_PIN_6) | (GPIO_PIN_7)));
    //
    // Configure the PWM generator for count down mode with immediate updates
    // to the parameters.
    //PB_6_m0
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the period. For a 50 KHz frequency, the period = 1/50,000, or 20
    // microseconds. For a 20 MHz clock, this translates to 400 clock ticks.

    // Set the load value of PWM0 generator
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 10000);
    // Set the pulse width of PWM0
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, R_SPD);   // Right Wheel Speed Control
    // Set the load value of PWM1 generator
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 10000);
    // Set the pulse width of PWM1
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, L_SPD);   // Left Wheel Speed Control
    // Enable Gen 1 and Gen 0
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);

    // Enable the outputs
    PWMOutputState(PWM0_BASE, (PWM_OUT_0_BIT | PWM_OUT_1_BIT), true);
}

// QEI initialization
void initQEI(void){

    // Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
    // In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    // Set Pins to be PHA0 and PHB0 (Module 0 Phase A and Phase B)
    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);

    // Set Pins to be PHA1 and PHB1 (Module 1 Phase A and Phase B)
    GPIOPinConfigure(GPIO_PC5_PHA1);
    GPIOPinConfigure(GPIO_PC6_PHB1);

    // Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7. I believe this sets the pull up and makes them inputs
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 |  GPIO_PIN_7);  // Left wheel QEI
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 |  GPIO_PIN_6);  // Right wheel QEI

    // Disable peripheral and interrupts before configuration
    QEIDisable(QEI0_BASE);
    QEIDisable(QEI1_BASE);

    QEIIntClear(QEI0_BASE, (QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX));
    QEIIntClear(QEI1_BASE, (QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX));


    QEIIntDisable(QEI0_BASE,(QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX));
    QEIIntDisable(QEI1_BASE,(QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX));

    // Configure quadrature encoder, use an arbitrary top limit of 1000
    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 127);
    QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 127);

    // Enable the quadrature encoder.
    QEIEnable(QEI0_BASE);
    QEIEnable(QEI1_BASE);

    // Set position to a middle value so we can see if things are working
    QEIPositionSet(QEI0_BASE, 0);
    QEIPositionSet(QEI1_BASE, 0);

    uint32_t Clk_period = SysCtlClockGet();

    // Using SYSTEM Clock to get a 1 second period for velocity
    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_2, Clk_period);
    QEIVelocityEnable(QEI0_BASE);

    // Using SYSTEM Clock to get a 1 second period for velocity
    QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_2, Clk_period);
    QEIVelocityEnable(QEI1_BASE);

    // Set up interrupt function pointer
    QEIIntRegister(QEI0_BASE, QEI0_handler);
    QEIIntEnable(QEI0_BASE, QEI_INTINDEX);

    // Set up interrupt function pointer
    QEIIntRegister(QEI1_BASE, QEI1_handler);
    QEIIntEnable(QEI1_BASE, QEI_INTINDEX);
}

// UART0 initialization
void initUART0(void){

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, (GPIO_PIN_0 | GPIO_PIN_1));
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);

}
/**************************************PROCEDURES**************************************/
void PD_control(void)
{
    // this function corrects the errors in this wheel speed and position

    POS_Wh1 = QEIPositionGet(QEI0_BASE);
    POS_Wh2 = QEIPositionGet(QEI1_BASE);
    int32_t error;
    error = (POS_Wh1 - POS_Wh2);
    if(error < 0){
        error = -error;
    }
    if(POS_Wh1 > POS_Wh2){
        error = 0;
    }

}
void CW_90(void){
    cnt = 0;
    right_brake();
    left_brake();
    SysTick_Wait(16000000); // wait to prevent carrying momentum from previous movement
    // slow down for turning
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 6500);   // Right Wheel Speed Control
    // Set the pulse width of PWM1
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 6500);   // Left Wheel Speed Control
    QEIPositionSet(QEI0_BASE, 0);
    QEIPositionSet(QEI1_BASE, 0);
    POS_Wh1 = QEIPositionGet(QEI0_BASE);
    POS_Wh2 = QEIPositionGet(QEI1_BASE);
    left_FWD();
    right_REV();

    //while((POS_Wh1 != 51) && (POS_Wh2 != 51))
    while(1){

        POS_Wh1 = QEIPositionGet(QEI0_BASE);
        POS_Wh2 = QEIPositionGet(QEI1_BASE);

        if(POS_Wh1 == 52){
           left_brake();
           PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, L_SPD);   // Resume left speed
           QEIPositionSet(QEI0_BASE, 0);
           cnt = cnt + 1;
        }
        if(POS_Wh2 == (127-52)){
           right_brake();
           PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, R_SPD);   // Resume right speed
           QEIPositionSet(QEI1_BASE, 0);
           cnt = cnt + 1;
        }
        if(cnt == 2){
            break;
        }
    }
    SysTick_Wait(16000000); // wait to prevent carrying momentum on to next movement
    SysTick_Wait(16000000); // wait to prevent carrying momentum on to next movement
}
void CCW_90(void){
    cnt = 0;
    right_brake();
    left_brake();
    SysTick_Wait(16000000); // wait to prevent carrying momentum from previous movement
    // slow down for turning
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 6500);   // Right Wheel Speed Control
    // Set the pulse width of PWM1
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 6500);   // Left Wheel Speed Control
    QEIPositionSet(QEI0_BASE, 0);
    QEIPositionSet(QEI1_BASE, 0);

    left_REV();
    right_FWD();

    POS_Wh1 = QEIPositionGet(QEI0_BASE);
    POS_Wh2 = QEIPositionGet(QEI1_BASE);

    while(1){

        POS_Wh1 = QEIPositionGet(QEI0_BASE);
        POS_Wh2 = QEIPositionGet(QEI1_BASE);

        if(POS_Wh1 == (127-52)){
            left_brake();
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, L_SPD);   // Resume left speed
            QEIPositionSet(QEI0_BASE, 0);
            cnt = cnt + 1;
        }
        if(POS_Wh2 == 52){
            right_brake();
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, R_SPD);   // Resume right speed
            QEIPositionSet(QEI1_BASE, 0);
            cnt = cnt + 1;
        }
        if(cnt == 2){
            break;
        }
    }
    SysTick_Wait(16000000); // wait to prevent carrying momentum on to next movement
}
void FWD_1_foot(void){

    cnt = 0;
    QEIPositionSet(QEI0_BASE, 0);
    QEIPositionSet(QEI1_BASE, 0);

    all_FWD();

    IND_Wh1 = 0;
    IND_Wh2 = 0;

    POS_Wh1 = QEIPositionGet(QEI0_BASE);
    POS_Wh2 = QEIPositionGet(QEI1_BASE);

    while(cnt != 2){

        POS_Wh1 = QEIPositionGet(QEI0_BASE);
        POS_Wh2 = QEIPositionGet(QEI1_BASE);

        if(POS_Wh1 == 127){
            IND_Wh1 = 1;
        }
        if(POS_Wh2 == 127){
            IND_Wh2 = 1;
        }
        if((POS_Wh1 == 90) && (IND_Wh1 == 1)){
            left_brake();
            QEIPositionSet(QEI0_BASE, 0);
            cnt = cnt + 1;
        }
        if((POS_Wh2 == 90) && (IND_Wh2 == 1)){
            right_brake();
            QEIPositionSet(QEI1_BASE, 0);
            cnt = cnt + 1;
        }
    }
    SysTick_Wait(16000000); // wait to prevent carrying momentum on to next movement
    SysTick_Wait(16000000); // wait to prevent carrying momentum on to next movement
}

//////////////////////      Drive straight forward      ///////////////////////
void all_FWD(void){

    right_FWD();
    left_FWD();

}
void right_FWD(void){

    /*      REFERENCE TABLE     */
    //  IN1 --> PIN_1
    //  IN2 --> PIN_2
    //  STANDBY --> PIN_0

    // Setting the proper pins to drive the motor forward [ IN1 = 0 ; IN2 = 1 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_0) | (GPIO_PIN_2) ), ( (GPIO_PIN_0) | (GPIO_PIN_2) ));
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, ~(GPIO_PIN_1));

}
void right_REV(void){

    // Setting the proper pins to drive the motor in reverse [ IN1 = 1 ; IN2 = 0 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_0) | (GPIO_PIN_1) ), ( (GPIO_PIN_0) | (GPIO_PIN_1) ));
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, ~(GPIO_PIN_2));
}
void right_brake(void){

    // Setting the proper pins to stop the motors  [ IN1 = 1 ; IN2 = 1 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_0) | (GPIO_PIN_1)| (GPIO_PIN_2) ), ( (GPIO_PIN_0) | (GPIO_PIN_1)| (GPIO_PIN_2)));
}
void right_standby(void){

    // Setting the proper pins to stop the motors  [ IN1 = 0 ; IN2 = 0 ; SB = 0 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_0) | (GPIO_PIN_1)| (GPIO_PIN_2) ), ( ~(GPIO_PIN_0) | (GPIO_PIN_1)| (GPIO_PIN_2)));
}
void left_FWD(void){

    /*      REFERENCE TABLE     */
    //  IN1 --> PIN_5
    //  IN2 --> PIN_4
    //  STANDBY --> PIN_3

    // Setting the proper pins to drive the motor in reverse [ IN1 = 1 ; IN2 = 0 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_3) | (GPIO_PIN_4) ), ( (GPIO_PIN_3) | (GPIO_PIN_4) ));
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, ~(GPIO_PIN_5));
}
void left_REV(void){

    // Setting the proper pins to drive the motor forward [ IN1 = 0 ; IN2 = 1 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_3) | (GPIO_PIN_5) ), ( (GPIO_PIN_3) | (GPIO_PIN_5) ));
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, ~(GPIO_PIN_4));
}
void all_REV(void){

    // Setting the proper pins to drive the motor in reverse [ IN1 = 1 ; IN2 = 0 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_0) | (GPIO_PIN_1) ), ( (GPIO_PIN_0) | (GPIO_PIN_1) ));
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, ~(GPIO_PIN_2));
    // Setting the proper pins to drive the motor forward [ IN1 = 0 ; IN2 = 1 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_3) | (GPIO_PIN_5) ), ( (GPIO_PIN_3) | (GPIO_PIN_5) ));
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, ~(GPIO_PIN_4));

}
void left_brake(void){

    // Setting the proper pins to stop the motors  [ IN1 = 1 ; IN2 = 1 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_3) | (GPIO_PIN_4)| (GPIO_PIN_5) ), ( (GPIO_PIN_3) | (GPIO_PIN_4)| (GPIO_PIN_5)));

}
void left_standby(void){

    // Setting the proper pins to stop the motors  [ IN1 = 0 ; IN2 = 0 ; SB = 0 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_3) | (GPIO_PIN_4)| (GPIO_PIN_5) ), ( ~(GPIO_PIN_3) | (GPIO_PIN_4)| (GPIO_PIN_5)));

}
void initSysTick(void)
{
  NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_RELOAD_R = NVIC_ST_RELOAD_M;  // maximum reload value
  NVIC_ST_CURRENT_R = 0;
  NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE+NVIC_ST_CTRL_CLK_SRC;
}

// The delay parameter is in units of the core clock. ( 1/120000000 sec )
void SysTick_Wait(uint32_t delay)
{
  volatile uint32_t elapsedTime;
  uint32_t startTime = NVIC_ST_CURRENT_R;
  do{
    elapsedTime = (startTime-NVIC_ST_CURRENT_R)&0x00FFFFFF;
  }
  while(elapsedTime <= delay);
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

//IND_Wh1 = GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_1);
//GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1, (GPIO_PIN_1 ^ IND_Wh1));
//SysTick_Wait(16000000); // wait to prevent carrying momentum on to next movement
