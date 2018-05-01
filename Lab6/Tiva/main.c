/**
 * main.c
 * ECPE 155 Autonomous Robotics
 * Spring 2018
 * Lab 6 - Obstacle Avoidance with the use of IR and bump sensors
 * Lab Associates:  Paul Vuong
 *                  Steve Guerrero
 * 1.   In addition to the bump switches, install the IR sensors (3).
 * 2.   Re-design obstacle avoidance from lab4.
 * 3.   Attach the IR sensors and connect to the TM4C.
 * 4.   The IR sensors output an analog signal, the use of the ADC will be needed to convert analog samples.
 * 5.   Design and implement a low level obstacle avoidance warning system based on the IR and bump sensor readings.
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
#include "driverlib\timer.h"
#include "driverlib\pin_map.h"
#include "driverlib\interrupt.h"
#include "driverlib\uart.h"
#include "driverlib\qei.h"
#include "driverlib\adc.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "motor_control.h"

// Global Variables
volatile    uint32_t cnt = 0;
volatile    uint32_t Clk_period;
volatile    uint32_t speed_Wh1, speed_Wh2;
volatile    int32_t direction_Wh1, direction_Wh2;
volatile    uint32_t POS_Wh1, POS_Wh2;
volatile    uint32_t IND_Wh1, IND_Wh2;
volatile    uint32_t ADC_F, ADC_R, ADC_L;
            uint32_t ADC_Data[3];
volatile    uint32_t prev_error = 0;
volatile    uint32_t L_SPD = 7000;
volatile    uint32_t R_SPD = 7200;

// Constants
const       float L = 4.65;
const       float L_rad = 2.93; // Left Wheel Radius
const       float R_rad = 2.9;  // Right Wheel Radius

volatile    uint8_t Kp = 30;    // Kp proportional const
volatile    uint8_t Kd = 2;     // Kd derivative const

// Boolean variables
_Bool da_wae = false;
_Bool IR_func = true;
_Bool bumped = true;
_Bool ADC_threshold = false;
_Bool ADC_flag = false;

// Bump Switches
volatile    uint32_t r_bumpSensors = 0x4;
volatile    uint32_t l_bumpSensors = 0x2;

uint32_t ii = 0;
uint32_t i = 0;
char strToSend[8];

int32_t choice;
int mc_data;
int mc_data_2;
int mc_data_3;
int mc_data_4;
int main(void){

    initSYSCTL();
    initPWM0();
    initQEI();
    initADC();
    initGPIO();
    initUART5();
    initSysTick();
    init_timerA();

    //To send multiple characters, such as numbers, we need to send multiple characters.  We can do this using a string and a for loop:
    //UART EXAMPLE CODE FOR POSITION
    //char strToSend[8] = "Testing";
//    uint32_t i = 0;

    IND_Wh1 = 0;
    IND_Wh2 = 0;
    while(1){

        if(ADC_flag){
            IR_functions();
        }
        if(bumped == true){
            bumper_function();
            bumped = false;
        }
        if(choice == 0x1){    // turn the input angle
            choice = 0;
            while(!UARTCharsAvail(UART5_BASE)){  // clean up anything left in FIFO
            }
            mc_data = UARTCharGet(UART5_BASE);
            mc_data = (mc_data << 8);
            while(!UARTCharsAvail(UART5_BASE)){  // clean up anything left in FIFO
            }
            mc_data_2 = UARTCharGet(UART5_BASE);
            mc_data |= mc_data_2;
            if(mc_data > 32767){
                mc_data = (mc_data - 65536);
            }
            turn_angle(mc_data);
            UARTIntEnable(UART5_BASE,UART_INT_RX);     // re-enable UART interrupt
        }
        if(choice == 0x2){    // drive forward for input cm
            choice = 0;
            mc_data = UARTCharGet(UART5_BASE);
            mc_data = (mc_data << 8);
            while(!UARTCharsAvail(UART5_BASE)){  // clean up anything left in FIFO
            }
            mc_data_2 = UARTCharGet(UART5_BASE);
            mc_data |= mc_data_2;
            FWD_cent(mc_data);
            UARTIntEnable(UART5_BASE,UART_INT_RX);     // re-enable UART interrupt
         }
        if (choice == 0x03)   // toggle obstacle avoidance
        {   choice = 0;
            IR_func = (IR_func ^ 1);
            if(IR_func == false){
                ADCSequenceDisable(ADC0_BASE, 1);   // disables ADC peripheral
            }
            else{
                ADCSequenceEnable(ADC0_BASE, 1);    // enables ADC peripheral
            }
            UARTIntEnable(UART5_BASE,UART_INT_RX);     // re-enable UART interrupt
        }
        if(choice == 5){    // drive forward for input cm
            choice = 0;
            mc_data = UARTCharGet(UART5_BASE);
            mc_data = (mc_data << 8);
            mc_data_2 = UARTCharGet(UART5_BASE);
            mc_data |= mc_data_2;
            mc_data_3 = UARTCharGet(UART5_BASE);
            mc_data_3 = (mc_data_3 << 8);
            while(!UARTCharsAvail(UART5_BASE)){  // clean up anything left in FIFO
            }
            mc_data_4 = UARTCharGet(UART5_BASE);
            mc_data_3 |= mc_data_4;
            if(mc_data > 32767){
                mc_data = (mc_data - 65536);
            }
            turn_angle(mc_data);

            FWD_cent(mc_data_3);
            UARTIntEnable(UART5_BASE,UART_INT_RX);     // re-enable UART interrupt
         }
    }
}

/**************************************HANDLERS**************************************/
void QEI0_handler(void){ // Left Wheel

    QEIIntClear(QEI0_BASE, (QEI_INTTIMER | QEI_INTDIR | QEI_INTERROR | QEI_INTINDEX));

    //POS_Wh1 = QEIPositionGet(QEI0_BASE);
    //direction_Wh1 = QEIDirectionGet(QEI0_BASE);
    IND_Wh1 = IND_Wh1 + 1;

}
void QEI1_handler(void){ // Right Wheel
    QEIIntClear(QEI1_BASE, (QEI_INTTIMER | QEI_INTDIR | QEI_INTERROR | QEI_INTINDEX));

    //POS_Wh1 = QEIPositionGet(QEI0_BASE);
    //direction_Wh1 = QEIDirectionGet(QEI0_BASE);
    IND_Wh2 = IND_Wh2 + 1;
}
void UART_handler(void){

    int32_t check;
    UARTIntDisable(UART5_BASE,UART_INT_RX); // disable so that it doesn't reenter the handler upon user input
    // check if something in FIFO
    check = UARTCharGet(UART5_BASE);    // garbage variable;
    if(UARTCharsAvail(UART5_BASE)){
        choice = UARTCharGet(UART5_BASE);
    }
    if((check != 0) && (choice == 0))
        choice = check;

//    if (choice == 0x01){  // turn the input angle
//        mc_data = UARTCharGet(UART5_BASE);
//        if(UARTCharsAvail(UART5_BASE))
//            mc_data_2 = UARTCharGet(UART5_BASE);
//
//        dis_wae = (mc_data << 8);
//        dis_wae |= mc_data_2;
//        mc_data = dis_wae;
//
//        if(dis_wae > 32767){
//            mc_data = (dis_wae - 65536);
//        }
//    }
//    if (choice == 0x02){  // drive forward for input cm
//        mc_data = UARTCharGet(UART5_BASE);
//        if(UARTCharsAvail(UART5_BASE))
//            mc_data_2 = UARTCharGet(UART5_BASE);
//
//        dis_wae = (mc_data << 8);
//        dis_wae |= mc_data_2;
//        mc_data = dis_wae;
//
//    }
    if (choice == 0x04)   // Self-Destruct
    {
        for(int k = 0;k < 4 ; k++){
            turn_angle(-10);
            turn_angle(10);
        }
        choice = 0;
    }


   // UARTIntEnable(UART5_BASE,UART_INT_RX);     // re-enable UART interrupt
}
void ADC_function(void){

  //  ADCProcessorTrigger(ADC0_BASE, 1);
  //  ADCIntDisable(ADC0_BASE, 1);
    ADCIntClear(ADC0_BASE, 1);

    // Wait for conversion to be completed.
    //while(!ADCIntStatus(ADC0_BASE, 1, false)){}
    //
    // Clear the ADC interrupt flag.

    //
    // Read ADC Value.
    ADCSequenceDataGet(ADC0_BASE, 1, ADC_Data);

    ADC_F = ADC_Data[0];
    ADC_R = ADC_Data[1];
    ADC_L = ADC_Data[2];

    ADC_flag = true;
    if((ADC_F > 1900) || (ADC_L > 2850) || (ADC_R > 3100))
        ADC_threshold = true;
    ADCIntEnable(ADC0_BASE, 1);

}

void IR_functions(void){

    ////////////////////////////   LAB5    /////////////////////
    //Robot will operate in IR mode when distances detected on all three sensors are below an ADC reading of 650
    //Robot will operate in bumper mode when distances are at a maximum value. I.E. an object is too close to the
    //robot for accurate measurements. In which case rely on the bump sensors instead.

    //Conditional statements for ADC_F = 1000, ADC_R = 2100, and ADC_L = 1950 indicate robot is able to move freely
    //and there are no obstacles w/in a 15cm radius in front of it.
    if(ADC_F > 1900){
        //Front sensor detects an obstacle, stop bust a U and drive in opposite direction
        turn_angle(45);

    }else if(ADC_L > 2850){
        //Front and Left IR sensor detect obstacle, rotate CW
        turn_angle(45);

    }else if(ADC_R>3100){
        //Robot is driving too close to an obstacle on the Right, rotate CCW
        turn_angle(-45);
    }

    ADC_flag = false;
    all_Brake();

}
void bumpSensor_handler(void){

    GPIOIntClear(GPIO_PORTD_BASE,(GPIO_PIN_2 | GPIO_PIN_3));
    l_bumpSensors = GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_2);
    r_bumpSensors = GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_3);
    GPIOIntDisable(GPIO_PORTD_BASE, (GPIO_PIN_2 | GPIO_PIN_3));
    bumped = true;

}
void bumper_function(void){

    if(r_bumpSensors == 0){
        //blink leds when bumpers make contact PD2 right switch
        GPIOPinWrite(GPIO_PORTF_BASE,(GPIO_PIN_1 | GPIO_PIN_2), (GPIO_PIN_1 | ~GPIO_PIN_2));
        //reverse
        REV_cent(8);
        //ccw rotate
        turn_angle(-90);
    }

    if(l_bumpSensors == 0){
        //blink leds when bumpers make contact PD1 left switch
        GPIOPinWrite(GPIO_PORTF_BASE,(GPIO_PIN_1 | GPIO_PIN_2), (~GPIO_PIN_1 | GPIO_PIN_2));
        //reverse
        REV_cent(8);
        //cw rotate
        turn_angle(90);
    }
    else if((r_bumpSensors == 0) && (l_bumpSensors == 0)){
        //blink leds when bumpers make contact
        GPIOPinWrite(GPIO_PORTF_BASE,(GPIO_PIN_1 | GPIO_PIN_2), (GPIO_PIN_1 | GPIO_PIN_2));
        // Reverse
        REV_cent(8);
        //cw rotate
        turn_angle(180);
    }
    else{
        // Turn the bumper interrupts back on
        GPIOIntEnable(GPIO_PORTD_BASE, (GPIO_PIN_2 | GPIO_PIN_3));
        // Turn all lights off
        GPIOPinWrite(GPIO_PORTF_BASE,(GPIO_PIN_1 | GPIO_PIN_2), ~(GPIO_PIN_1 | GPIO_PIN_2));
    }
        // Turn the bumper interrupts back on
    GPIOIntEnable(GPIO_PORTD_BASE, (GPIO_PIN_2 | GPIO_PIN_3));
    // Turn all lights off
    GPIOPinWrite(GPIO_PORTF_BASE,(GPIO_PIN_1 | GPIO_PIN_2), ~(GPIO_PIN_1 | GPIO_PIN_2));

}

// SYSCTL initialization
void initSYSCTL(void){

    //Set system clock to 80MHz, Utilize main oscillator
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    // Enable GPIO peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);    // UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);    // Driver control PWM0 (B6 & B7)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);    // QEI
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);    // Bumpers & QEI
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);    // ADC (E1 & E2 & E3)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);    // LED Lights (F1 & F2)

    // Enable PWM peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);     // PWM Module 0

    // Enable Timer for PD_control Interrupt
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // Enable UART Peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);    // UART5 Module

    // Enable QEI Peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);     // QEI0 Module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);     // QEI1 Module
    //
    // The ADC0 peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);     // ADC0 Module

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)){}

}

// GPIO initialization
void initGPIO(void){

    // LEDS
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,(GPIO_PIN_1 | GPIO_PIN_2));
    //UART5
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,(GPIO_PIN_4));
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE,(GPIO_PIN_5));
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

    SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
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

    Clk_period = SysCtlClockGet();

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
// Initialize ADC
void initADC(void){

    // Change the pin types to use the ADC peripheral
    GPIOPinTypeADC(GPIO_PORTE_BASE, ( GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3));

    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.  Each ADC module has 4 programmable sequences, sequence 0
    // to sequence 3.  This example is arbitrarily using sequence 3.
    //
    //ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_ALWAYS, 0);
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // information on the ADC sequences and steps, reference the datasheet.
    //
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH2| ADC_CTL_IE | ADC_CTL_END);   // enable interrupt after ch2 read

    ADCHardwareOversampleConfigure(ADC0_BASE, 8);
    //
    // Since sample sequence 3 is now configured, it must be enabled.
    //
    ADCIntRegister (ADC0_BASE, 0x1,ADC_function);
    //ADCIntEnableEx (ADC0_BASE, ADC_INT_SS1);
    ADCIntEnable (ADC0_BASE, 1);

    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    //
    ADCSequenceEnable(ADC0_BASE, 1);
    ADCIntClear(ADC0_BASE, 1);
    //
    // Sample AIN0 forever.  Display the value on the console.
}

// UART5 initialization
void initUART5(void){

    GPIOPinConfigure(GPIO_PE4_U5RX);
    GPIOPinConfigure(GPIO_PE5_U5TX);
    GPIOPinTypeUART(GPIO_PORTE_BASE, (GPIO_PIN_4 | GPIO_PIN_5));
    UARTConfigSetExpClk(UART5_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTIntRegister(UART5_BASE,UART_handler);
    UARTFIFOLevelSet(UART5_BASE,UART_FIFO_TX7_8,UART_FIFO_RX1_8);
    UARTEnable(UART5_BASE);
    UARTIntEnable(UART5_BASE,UART_INT_RX);
}

/**************************************PROCEDURES**************************************/

void PD_control(void){

    // this function corrects the errors in the wheel speed every 0.5 seconds
    // this will allow robot to travel a straighter path with less error in position

    TimerDisable(TIMER0_BASE, TIMER_A);
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    if(da_wae == false){
        TimerEnable(TIMER0_BASE, TIMER_A);
        return;
    }

    speed_Wh1 = QEIVelocityGet(QEI0_BASE);  // RPM of Left wheel
    speed_Wh2 = QEIVelocityGet(QEI1_BASE);  // RPM of Right wheel

    int32_t P;      // Proportional error
    int32_t D;      // Derivative error
    int32_t error;
    error = (speed_Wh1 - speed_Wh2);

    if(error != 0){

        if(error < 0){  //Wheel 2 (Right) speed greater than Wheel 1 (left)
            error = -(error);
            P = Kp*(error);
            D = Kd * (error - prev_error);
            R_SPD = (R_SPD - (P + D));        // decrease right speed
            L_SPD = (L_SPD + (P + D));        // increase left speed
        }
        else{       // Left speed greater than Right
            P = Kp*(error);
            D = Kd * (error - prev_error);
            R_SPD = (R_SPD + (P + D));        // increase right speed
            L_SPD = (L_SPD - (P + D));        // decrease left speed
        }
    }
    if(R_SPD < 5200){
        R_SPD = (R_SPD + 700);   // if below 50% duty cycle increase duty cycle
    }
    if(L_SPD < 5000){
        L_SPD = (L_SPD + 700);
    }
    if(R_SPD > 9400){   // if above 93% duty cycle decrease duty cycle
        R_SPD = (R_SPD - 700);
    }
    if(L_SPD > 9500){
        L_SPD = (L_SPD - 700);
    }

    prev_error = error;

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, L_SPD);   // Set left speed
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, R_SPD);   // Set right speed

    TimerEnable(TIMER0_BASE, TIMER_A);
}
void turn_angle(int angle){
/* This function takes in an angle in degrees and converts it to robot movement based on current pose*/
    // Angle inputs must be greater than 10 degrees and no greater than 180 degrees absolute value
    // if conditions not met the function returns
    da_wae = false;

    int check = 0;
    check = angle;
    if(angle < 0)
        check = -angle;

    if(check > 180)
        return;

    int index = 0;
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

    if(angle < 0){      // Turn CCW
        // convert the angle into wheel index
        angle = -angle;
        index = 0.5*( (angle * 1.1) - 1 ); // result goes to floor because of integers
        left_REV();
        right_FWD();
        while(1){
            if((bumped == true) || (ADC_threshold == true)){
                ADC_threshold = false;
                break;
            }
            POS_Wh1 = QEIPositionGet(QEI0_BASE);
            POS_Wh2 = QEIPositionGet(QEI1_BASE);

            if(POS_Wh1 == (127-index)){
                left_brake();
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, L_SPD);   // Resume left speed
                QEIPositionSet(QEI0_BASE, 0);
                cnt = cnt + 1;
            }
            if(POS_Wh2 == index){
                right_brake();
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, R_SPD);   // Resume right speed
                QEIPositionSet(QEI1_BASE, 0);
                cnt = cnt + 1;
            }
            if(cnt == 2){
                break;
            }
        }
    }
    else{       // Turn CW
        // convert the angle into wheel index
        index = 0.5*( (angle * 1.1) + 1 ); // result goes to floor because of integers
        left_FWD();
        right_REV();

        while(1){
            if((bumped == true) || (ADC_threshold == true)){
                ADC_threshold = false;
                break;
            }
            POS_Wh1 = QEIPositionGet(QEI0_BASE);
            POS_Wh2 = QEIPositionGet(QEI1_BASE);

            if(POS_Wh1 == index){
               left_brake();
               PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, L_SPD);   // Resume left speed
               QEIPositionSet(QEI0_BASE, 0);
               cnt = cnt + 1;
            }
            if(POS_Wh2 == (127-index)){
               right_brake();
               PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, R_SPD);   // Resume right speed
               QEIPositionSet(QEI1_BASE, 0);
               cnt = cnt + 1;
            }
            if(cnt == 2){
                break;
            }
        }
    }
      SysTick_Wait(16000000); // wait to prevent carrying momentum on to next movement
//    sprintf(strToSend,"%d\r\n",0xAA);
//    for(i = 0; (strToSend[i] != '\0'); i++){
//    UARTCharPut(UART5_BASE,strToSend[i]);
//    }
//    UARTCharPutNonBlocking(UART5_BASE, 0xAA);
}
void FWD_cent(int cm){  // This function moves the robot forward by given amount of centimeters
    /* this function takes in an integer and converts it to movement in centimeter*/
    // The input must be greater or equal to 2 cm ( for accuracy of movement )
    // if conditions not met the function returns
    if(cm > 512)
        return;

    int index = (7 * cm) + 1;
    int rev = (index/128); // how many revolutions
    int mod = (index%128); // how many indices after revolution
    cnt = 0;
    QEIPositionSet(QEI0_BASE, 0); // Left wheel reset
    QEIPositionSet(QEI1_BASE, 0); // right wheel reset
    all_FWD();

    IND_Wh1 = 0;
    IND_Wh2 = 0;

    while(cnt != 2){
        if((bumped == true) || (ADC_threshold == true)){
            ADC_threshold = false;
            break;
        }
        POS_Wh1 = QEIPositionGet(QEI0_BASE);
        POS_Wh2 = QEIPositionGet(QEI1_BASE);

        if(POS_Wh1 == 127){ // left wheel completed 1 revolution
            IND_Wh1 = IND_Wh1 + 1;
            QEIPositionSet(QEI0_BASE, 0); // Left wheel reset
        }
        if(POS_Wh2 == 127){ // right wheel completed 1 revolution
            IND_Wh2 = IND_Wh2 + 1;
            QEIPositionSet(QEI1_BASE, 0); // right wheel reset
        }
        if((POS_Wh1 == mod) && (IND_Wh1 >= rev)){
            left_brake();
            QEIPositionSet(QEI0_BASE, 0);
            cnt = cnt + 1;
        }
        if((POS_Wh2 == mod) && (IND_Wh2 >= rev)){
            right_brake();
            QEIPositionSet(QEI1_BASE, 0);
            cnt = cnt + 1;
        }
    }
    SysTick_Wait(16000000); // wait to prevent carrying momentum on to next movement
}
void REV_cent(int cm){  // This function moves the robot forward by given amount of centimeters
    /* this function takes in an integer and converts it to movement in centimeter*/
    // The input must be greater or equal to 2 cm ( for accuracy of movement )
    // if conditions not met the function returns
    da_wae = false;
    if(cm > 512)
        return;

    int index = (7 * cm) + 1;
    int rev = (index/128); // how many revolutions
    int mod = (index%128); // how many indices after revolution
    mod = (127 - mod);
    cnt = 0;
    QEIPositionSet(QEI0_BASE, 0); // Left wheel reset
    QEIPositionSet(QEI1_BASE, 0); // right wheel reset
    all_REV();

    IND_Wh1 = 0;
    IND_Wh2 = 0;

    while(cnt != 2){

        POS_Wh1 = QEIPositionGet(QEI0_BASE);
        POS_Wh2 = QEIPositionGet(QEI1_BASE);

        if(POS_Wh1 == 1){ // left wheel completed 1 revolution
            IND_Wh1 = IND_Wh1 + 1;
            QEIPositionSet(QEI0_BASE, 0); // Left wheel reset
        }
        if(POS_Wh2 == 1){ // right wheel completed 1 revolution
            IND_Wh2 = IND_Wh2 + 1;
            QEIPositionSet(QEI1_BASE, 0); // right wheel reset
        }
        if((POS_Wh1 == mod) && (IND_Wh1 >= rev)){
            left_brake();
            QEIPositionSet(QEI0_BASE, 0);
            cnt = cnt + 1;
        }
        if((POS_Wh2 == mod) && (IND_Wh2 >= rev)){
            right_brake();
            QEIPositionSet(QEI1_BASE, 0);
            cnt = cnt + 1;
        }
    }
    SysTick_Wait(16000000); // wait to prevent carrying momentum on to next movement
}
//////////////////////      Drive straight forward      ///////////////////////

void all_FWD(void){

    right_FWD();
    left_FWD();

    da_wae = true;
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
    da_wae = false;
    // Setting the proper pins to drive the motor in reverse [ IN1 = 1 ; IN2 = 0 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_0) | (GPIO_PIN_1) ), ( (GPIO_PIN_0) | (GPIO_PIN_1) ));
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, ~(GPIO_PIN_2));
}
void right_brake(void){
    da_wae = false;
    // Setting the proper pins to stop the motors  [ IN1 = 1 ; IN2 = 1 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_0) | (GPIO_PIN_1)| (GPIO_PIN_2) ), ( (GPIO_PIN_0) | (GPIO_PIN_1)| (GPIO_PIN_2)));
}
void right_standby(void){
    da_wae = false;
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
    da_wae = false;
    // Setting the proper pins to drive the motor forward [ IN1 = 0 ; IN2 = 1 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_3) | (GPIO_PIN_5) ), ( (GPIO_PIN_3) | (GPIO_PIN_5) ));
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, ~(GPIO_PIN_4));
}
void all_REV(void){
    da_wae = false;
    // Setting the proper pins to drive the motor in reverse [ IN1 = 1 ; IN2 = 0 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_0) | (GPIO_PIN_1) ), ( (GPIO_PIN_0) | (GPIO_PIN_1) ));
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, ~(GPIO_PIN_2));
    // Setting the proper pins to drive the motor forward [ IN1 = 0 ; IN2 = 1 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_3) | (GPIO_PIN_5) ), ( (GPIO_PIN_3) | (GPIO_PIN_5) ));
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, ~(GPIO_PIN_4));

}
void left_brake(void){
    da_wae = false;
    // Setting the proper pins to stop the motors  [ IN1 = 1 ; IN2 = 1 ; SB = 1 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_3) | (GPIO_PIN_4)| (GPIO_PIN_5) ), ( (GPIO_PIN_3) | (GPIO_PIN_4)| (GPIO_PIN_5)));

}
void left_standby(void){
    da_wae = false;
    // Setting the proper pins to stop the motors  [ IN1 = 0 ; IN2 = 0 ; SB = 0 ]
    GPIOPinWrite(GPIO_PORTB_BASE,( (GPIO_PIN_3) | (GPIO_PIN_4)| (GPIO_PIN_5) ), ( ~(GPIO_PIN_3) | (GPIO_PIN_4)| (GPIO_PIN_5)));

}
void all_Brake(void){
    da_wae = false;
    left_brake();
    right_brake();
}
void initSysTick(void)
{
    NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
    NVIC_ST_RELOAD_R = NVIC_ST_RELOAD_M;  // maximum reload value
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE+NVIC_ST_CTRL_CLK_SRC;
}
void init_timerA(void)
{
    TimerDisable(TIMER0_BASE, TIMER_A);
    TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);   // 32 bits Timer periodic
    TimerIntRegister(TIMER0_BASE, TIMER_A, PD_control);    // Registering;
    TimerLoadSet(TIMER0_BASE, TIMER_A, (Clk_period/3) ); // 0.25 second interrupt
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);

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
