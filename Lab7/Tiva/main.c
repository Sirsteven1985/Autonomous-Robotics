
/**
 * main.c
 * ECPE293 - Embedded Systems Project
 * Spring 2018
 * University of the Pacific, SOEC
 * Authors: Paul Vuong
 *          Steve Guerro
 * Demonstrate RTOS/Kernel on TivaC Launchpad microcontroller, TM4C1294NCPDT
 * 1. Task 1 - Polisher task:
 * 2. Task 2 - Scrubber task:
 * 3. Task 3 - Metrology task:
 * 4. Scheduler is comprised of an FSM and TIMER0A interrupt
 * 5.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "sysctl.h"
#include "osc.h"
#include "gpio.h"
#include "ppb.h"
#include "uart.h"
#include "uart0.h"
#include "timer.h"
#include "timer0a.h"

//        Global variables for Systick and TIMER0A clock rate
uint32_t  clock = 120000000;
uint32_t  delay = 12000000;

//       Global variables UART0...
uint32_t i = 0;
uint8_t  m = 0;
char     strToSend[8];

//      Flags for FSM
bool    waferPolish = true;
bool    waferScrub = false;
bool    waferInspect = false;
bool    tool_Idle = false;

//      Wafer Tracking
int     unPolishedWafers = 5;
int     unScrubbedWafers = 0;
int     unMeasuredWafers = 0;

int     PolishedWafers = 0;
int     ScrubbedWafers = 0;
int     MeasuredWafers = 0;

int     polishWaferInProcess = 0;
int     scrubWaferInProcess = 0;
int     measureWaferInProcess = 0;

//     Struct status queues
struct QUEUES{



};

//   System Protocols
void initGPIO(void);
void initSYSTCK(void);
void sysTickWait(uint32_t delay);
void MessageCenter(uint32_t state);

//   Task Prototypes
void T1(void);              // T1, polish wafer
void T2(void);              // T2, scrub wafer
void T3(void);              // T3, post-polish measurement

// Create states for FSM
enum{

    Polish, Scrub, Measure, Idle

};

//   System message: Wafers in process and Tasks in process
void MessageCenter(uint32_t state){

    sprintf(strToSend,"%i in process\r\n", state);
    for(m = 0; (strToSend[m] != '\0'); m++)
    txUART0(strToSend[m]);

    sprintf(strToSend,"Unpolished wafers: %d\t Unscrubbed wafers: %d\t Unmeasured wafers: %d\t\r\n",unPolishedWafers,unScrubbedWafers,unMeasuredWafers);
    for(m = 0; (strToSend[m] != '\0'); m++)
    txUART0(strToSend[m]);

    sprintf(strToSend,"Unpolished wafers in process: %d\t Unscrubbed wafers in process: %d\t Unmeasured wafers in process: %d\t\r\n",polishWaferInProcess,scrubWaferInProcess,measureWaferInProcess);
    for(m = 0; (strToSend[m] != '\0'); m++)
    txUART0(strToSend[m]);

    sprintf(strToSend,"Polished wafers: %d\t Scrubbed wafers: %d\t Measured wafers: %d\t\r\n",PolishedWafers,ScrubbedWafers,MeasuredWafers);
    for(m = 0; (strToSend[m] != '\0'); m++)
    txUART0(strToSend[m]);

}

int main(void)
{
    // Local Variables
    uint32_t state = Polish;

    // Initialize system
    initOsc();
    initGPIO();
    initTimer0A();
    initSYSTCK();
    initUART0();

    while(1){

        switch(state){

            case Polish:
                //Polisher is active, check scrubber and measurement tool idle
                //Check if there are unpolished wafers to polish
                //Deduct unpolished wafers if there are any to consume
                //Set TIMER0A for 15 seconds and enable TIMER0A
                //Flash LED2 for duration at 3/s frequency
                if(!waferScrub && !waferInspect && !){
                    if(unPolishedWafers > 0){
                        unPolishedWafers--;
                        polishWaferInProcess++;
                        setTimer0ARate(15*clock);
                        enableTimer0A(true);
                        T1();
                    }
                }//end polish

            case Scrub:
                //Scrubber is active, check polisher and measurement tool idle
                //Check if there are polished wafers to scrub
                //Deduct polished wafers if there are any to consume
                //Set TIMER0A for 12 seconds and enable TIMER0A
                //Flash LED3 for duration at 2/s frequency
                if(!waferPolish && !waferInspect){
                    if(unScrubbedWafers > 0){
                        unScrubbedWafers--;
                        scrubWaferInProcess++;
                        setTimer0ARate(12*clock);
                        enableTimer0A(true);
                        T2();
                    }
                }//end scrubber

            case Measure:
                //Metrology tool is active, check polisher and scrubber idle
                //Check if there are unmeasured wafers to analyze
                //Deduct unmeasured wafers if there are any to consume
                //Set TIMER0A for 10 seconds and enable TIMER0A
                //Flash LED4 for duration at 1/s frequency
                if(!waferPolish && !waferScrub){
                    if(unMeasuredWafers >= 0){
                        unMeasuredWafers--;
                        measureWaferInProcess++;
                        setTimer0ARate(10*clock);
                        enableTimer0A(true);
                        T3();
                    }
                }//end measure

            case Idle:
                if(!waferPolish && !waferScrub && !waferInspect){
                    ++MeasuredWafers;
                    MessageCenter(Measure);
                }

        }//end switch

    }//end main while

}//end main


//   Timer0A handler - Scheduler is comprised of HW
//   timer interrupt and finite state machine
void timer0A_handler(void){

    if(waferPolish){
        //Polisher has completed, enable scrubber.
        //Increase the number of unscrubbed wafers.
        polishWaferInProcess--;
        unScrubbedWafers++;
        PolishedWafers++;
        waferPolish = false;
        waferScrub = true;
        waferInspect = false;
        tool_Idle = false;
    }else if(waferScrub){
        //Scrubber has completed, enable measurement tool.
        //Increase the number of unmeasured wafers.
        scrubWaferInProcess--;
        unMeasuredWafers++;
        ScrubbedWafers++;
        waferPolish = false;
        waferScrub = false;
        waferInspect = true;
        tool_Idle = false;
    }else if(waferInspect){
        //measurement has completed, enable polisher.
        //Increase the number of unpolished wafers.
        measureWaferInProcess--;
        ++MeasuredWafers;
        waferPolish = true;
        waferScrub = false;
        waferInspect = false;
    }else if(tool_Idle){
        waferPolish = true;
        waferScrub = false;
        waferInspect = false;
        tool_Idle = false;
    }

    enableTimer0A(false);
    clearTimer0A();

}

//   T1, polish the wafer
//   Determine if there are any wafers to polish
//   Decrement the number of unpolished wafers
//   Polisher will run for 15 seconds
//   Use systick to have an LED blink at 3Hz for the Duration
//   After which increment the number of polished wafers
void T1(void){

    MessageCenter(Polish);

    while(waferPolish){
        //Blink LED2
        GPIO_PORTN[GPIO_PIN_0] = ~GPIO_PORTN[GPIO_PIN_0];
        for(i=0; i<3; i++){
            sysTickWait(delay);
        }
    }

}

//   T2, scrub the wafer
//   Determine if there are any wafers to scrub
//   Decrement the number of polished wafers
//   Scrubber will run for 12 seconds
//   Use systick to have an LED blink at 2Hz for the duration
//   After which increment the number of scrubbed wafers
void T2(void){

    MessageCenter(Scrub);

    while(waferScrub){
        //Blink LED3
        GPIO_PORTF[GPIO_PIN_4] = ~GPIO_PORTF[GPIO_PIN_4];
        for(i=0; i<5; i++){
            sysTickWait(delay);
        }
    }

}

//   T3, post polish analysis
//   Determine if there are any wafers to analyze
//   Decrement the number of scrubbed wafers
//   Analysis tool will run for 10 seconds
//   Use systick to have an LED blink at 1Hz for the duration
//   After which the wafer leaves the planar step
void T3(void){

    MessageCenter(Measure);

    while(waferInspect){
        //Blink LED4
        GPIO_PORTF[GPIO_PIN_0] = ~GPIO_PORTF[GPIO_PIN_0];
        for(i=0; i<10; i++){
            sysTickWait(delay);
        }
    }

}

//   Load a delay value to
void sysTickWait(uint32_t delay){

    volatile uint32_t elapsedTime;
    volatile uint32_t startTime = PPB[SYSCTL_ST_CURRENT];

    do{
      elapsedTime = ((startTime - PPB[SYSCTL_ST_CURRENT])&0x00FFFFFF);
    }
    while(elapsedTime <= delay);

}

//   Initialize systick timer
void initSYSTCK(void){

    PPB[SYSCTL_ST_CTRL] = 0;
    PPB[SYSCTL_ST_RELOAD] = 0x00FFFFFF;
    PPB[SYSCTL_ST_CURRENT] = 0x00FFFFFF;
    PPB[SYSCTL_ST_CTRL] |= ST_CTRL_EN | ST_CTRL_CLK_SRC;

}

//   Initialize GPIO ports
void initGPIO(void){

    SYSCTL[SYSCTL_RCGCGPIO] |= SYSCTL_RCGCGPIO_PORTA | SYSCTL_RCGCGPIO_PORTD | SYSCTL_RCGCGPIO_PORTE | SYSCTL_RCGCGPIO_PORTF | SYSCTL_RCGCGPIO_PORTN;
    SYSCTL[SYSCTL_RCGCGPIO] |= SYSCTL_RCGCGPIO_PORTA | SYSCTL_RCGCGPIO_PORTD | SYSCTL_RCGCGPIO_PORTE | SYSCTL_RCGCGPIO_PORTF | SYSCTL_RCGCGPIO_PORTN;

    //UART0: GPIO port direction. Input/Output.
    //                          U0Rx
    GPIO_PORTA[GPIO_DIR] &= ~(GPIO_PIN_0);                          //UART0 data input
    //                          U0Tx
    GPIO_PORTA[GPIO_DIR] |= GPIO_PIN_1;                             //UART0 data output
    //
    //Set the GPIO AFSEL bits for the input pins.
    //                          U0Rx        U0Tx
    GPIO_PORTA[GPIO_AFSEL] |= GPIO_PIN_0 | GPIO_PIN_1;              //UART0 RX and TX transmission lines
    //
    //Configure the PMCn fields in the GPIOPCTL register
    //Need PMC1 enabled for U0Rx(PA0) and U0Tx(PA1)
    GPIO_PORTA[GPIO_PCTL] &= ~((0xF<<(4*0)) | (0xF<<(4*1)));
    GPIO_PORTA[GPIO_PCTL] |= (0x1<<(4*0)) | (0x1<<(4*1));
    //
    //Enable the digital signals
    //                          U0Rx        U0Tx
    GPIO_PORTA[GPIO_DEN] |= GPIO_PIN_0 | GPIO_PIN_1;                //UART0


    //GPIOs to turn on LEDs
    //GPIO port A D E F direction, set as outputs
    GPIO_PORTA[GPIO_DIR] |= GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_PORTD[GPIO_DIR] |= GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_PORTE[GPIO_DIR] |= GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    //Port F - PF0 and PF4 is output for LED3 and LED4
    GPIO_PORTF[GPIO_DIR] |= GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
    //Port N is output for LED1 and LED2
    GPIO_PORTN[GPIO_DIR] |= GPIO_PIN_0 | GPIO_PIN_1;


    //Extend GPIO drive strength
    GPIO_PORTA[GPIO_PP] |= GPIO_PP_EDE;
    GPIO_PORTD[GPIO_PP] |= GPIO_PP_EDE;
    GPIO_PORTE[GPIO_PP] |= GPIO_PP_EDE;
    GPIO_PORTF[GPIO_PP] |= GPIO_PP_EDE;
    //
    //Extend to 12mA drive strength
    GPIO_PORTA[GPIO_PC] |= GPIO_PC_EDM;
    GPIO_PORTD[GPIO_PC] |= GPIO_PC_EDM;
    GPIO_PORTE[GPIO_PC] |= GPIO_PC_EDM;
    GPIO_PORTF[GPIO_PC] |= GPIO_PC_EDM;
    //
    //Set GPIODR4R, DR8R and DR12R corresponding bits to increase drive strength to 12mA
    GPIO_PORTA[GPIO_DR4R] |= GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_PORTD[GPIO_DR4R] |= GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_PORTE[GPIO_DR4R] |= GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_PORTF[GPIO_DR4R] |= GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    //
    GPIO_PORTA[GPIO_DR8R] |= GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_PORTD[GPIO_DR8R] |= GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_PORTE[GPIO_DR8R] |= GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_PORTF[GPIO_DR8R] |= GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    //
    GPIO_PORTA[GPIO_DR12R] |= GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_PORTD[GPIO_DR12R] |= GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_PORTE[GPIO_DR12R] |= GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_PORTF[GPIO_DR12R] |= GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;


    //GPIO port A D E F N digital enable
    GPIO_PORTA[GPIO_DEN] |= GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_PORTD[GPIO_DEN] |= GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_PORTE[GPIO_DEN] |= GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    //Port F - PF0 and PF4 is output for LED3 and LED4
    GPIO_PORTF[GPIO_DEN] |= GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
    //Port N is output for LED1 and LED2
    GPIO_PORTN[GPIO_DEN] |= GPIO_PIN_0 | GPIO_PIN_1;

}
