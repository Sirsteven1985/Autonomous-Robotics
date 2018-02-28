

/**
 * Blinky_DirectRegisterWrites.c
 * ECPE 155 Autonomous Robotics
 * Spring 2018
 * Lab 1 - Code Composer 7 Primer
 * Lab Associates:  Paul Vuong
 *                  Steve Guerro
 *
 * 1.   This lab serves as a refresher to Code Composer Studios IDE and the Tiva C Series
 *      family of microcontrollers. Specifically, the TM4C123GH6PM.
 * 2.   Create source code to make one or all three of the LED's blink on the test-board using direct
 *      register writes.
 */

#include <stdint.h>
#include <stdbool.h>

//System control base address
#define SYSCTL     (((volatile uint32_t *) 0x400FE000))
//Port F base address
#define GPIO_PORTF (((volatile uint32_t *) 0x40025000))



enum {
        SYSCTL_RCGCGPIO         = (0x608 >> 2),   //RCGC GPIO offset
        SYSCTL_RCGCGPIO_PORTF   = (1 << 5)        //Enable run mode gating clock control
};


enum {
        SYSCTL_RCC_R            = (0x060 >> 2),   //Run-mode clock configuration register
        SYSCTL_RCC_USESYSDIV    = (1 << 22),      //System clock is used undivided
        SYSCTL_RCC_XTAL         = (0x15 << 6),    //16MHz crystal value
        SYSCTL_RCC2_R           = (0x070 >> 2),   //Run-mode clock 2 configuration register
        SYSCTL_RCC2_BYPASS2     = (1 << 11),      //Disable the PLL Bypass2, the system clock is the PLL output clock
                                                  //divided by the divisor specified by SYSDIV2.
                                                  //See Table 5-5 on page 223 for programming guidelines.
                                                  //Note: The ADC must be clocked from the PLL or directly from a
                                                  //16-MHz clock source to operate properly.
        SYSCTL_RCC2_USERCC2     = ((uint32_t)1 << 31),//The RCC2 register fields override the RCC register fields.
        SYSCTL_RCC2_OSCSRC2     = (0x0 << 4),     //Select main oscillator.
        SYSCTL_RCC2_PWRDN2      = (1 << 13),      //Clear this bit to power and enable PLL and output
        SYSCTL_RCC2_SYSDIV2     = (0x02 << 23),   //System Clock Divisor 2
        SYSCTL_RCC2_SYSDIV2LSB  = (1 << 22),      //Additional LSB for SYSDIV2
        SYSCTL_RCC2_SYSDIV400   = (1 << 30),      //Enabling, Append the SYSDIV2LSB bit to the SYSDIV2 field to create a
                                                  //7 bit divisor using the 400 MHz PLL output, see Table
                                                  //5-6 on page 224.
        SYSCTL_RIS_R            = (0x050 >> 2),   //Raw interrupt status register
        SYSCTL_RIS_PLLLRIS      = (1 << 6)        //PLL Lock Raw Interrupt Status
};

enum {
        GPIO_DATA               = 0,   //GPIO data register
        GPIO_DIR                = (0x400 >> 2),   //GPIO direction register
        GPIO_DEN                = (0x51C >> 2),   //GPIO digital enable register
        GPIO_PIN_1              = (1 << 1),       //Port pins
        GPIO_PIN_2              = (1 << 2),
        GPIO_PIN_3              = (1 << 3)
};

/* Function Declarations for system control and GPIO port configurations
 * 1. initOSC(): increase microcontroller CPU clock speed to 80MHz.
 * 2. initGPIO(): Initiate gpio pin configurations.
 * 3. wait(): count up timer.
 * */
void initOSC(void);
void initGPIO(void);
void wait(void);

int main(void){

    //Initialize oscillator file (system clock)
    initOSC();
    //Enable desired GPIO ports
    initGPIO();

    while(1){

        //turn red led on then off
        GPIO_PORTF[GPIO_PIN_1] = 0x2;
        wait();
        GPIO_PORTF[GPIO_PIN_1] = ~(0x2);
        wait();
        //turn blue led on then off
        GPIO_PORTF[GPIO_PIN_2] = 0x4;
        wait();
        GPIO_PORTF[GPIO_PIN_2] = ~(0x4);
        wait();
        //turn green led on then off
        GPIO_PORTF[GPIO_PIN_3] = 0x8;
        wait();
        GPIO_PORTF[GPIO_PIN_3] = ~(0x8);
        wait();

    }//end main while loop

}//end main

void initOSC(void){

    //The PLL is configured using direct register writes to the RCC/RCC2 register. If the RCC2 register
    //is being used, the USERCC2 bit must be set and the appropriate RCC2 bit/field is used. The steps
    //required to successfully change the PLL-based system clock are (pp.231):

    //1. Bypass the PLL and system clock divider by setting the BYPASS bit and clearing the USESYS
    //bit in the RCC register, thereby configuring the microcontroller to run off a "raw" clock source
    //and allowing for the new PLL configuration to be validated before switching the system clock
    //to the PLL.

    //Bypass PLL while configuring
    SYSCTL[SYSCTL_RCC_R]   |= SYSCTL_RCC2_BYPASS2;
    SYSCTL[SYSCTL_RCC2_R]  |= SYSCTL_RCC2_BYPASS2;
    //Raw clock source used while configuring PLL.
    SYSCTL[SYSCTL_RCC_R]   &= ~(SYSCTL_RCC_USESYSDIV);
    //Extend RCC register fields to RCC2.
    //Bitfields in RCC2 with similar functions to
    //RCC will take precedence if USERCC2 bit is set.
    SYSCTL[SYSCTL_RCC2_R]  |= SYSCTL_RCC2_USERCC2;

    //2. Select the crystal value (XTAL) and oscillator source (OSCSRC), and clear the PWRDN bit in
    //RCC/RCC2. Setting the XTAL field automatically pulls valid PLL configuration data for the
    //appropriate crystal, and clearing the PWRDN bit powers and enables the PLL and its output.

    //16.384Mhz crystal oscillator chosen for PLL
    SYSCTL[SYSCTL_RCC_R]   |= SYSCTL_RCC_XTAL;
    //Main oscillator source selected
    SYSCTL[SYSCTL_RCC_R]   |= SYSCTL_RCC2_OSCSRC2;
    SYSCTL[SYSCTL_RCC2_R]  |= SYSCTL_RCC2_OSCSRC2;
    //Enable PLL and output
    SYSCTL[SYSCTL_RCC_R]   &= ~(SYSCTL_RCC2_PWRDN2);
    SYSCTL[SYSCTL_RCC2_R]  &= ~(SYSCTL_RCC2_PWRDN2);

    //3. Select the desired system divider (SYSDIV) in RCC/RCC2 and set the USESYS bit in RCC. The
    //SYSDIV field determines the system frequency for the microcontroller.

    //SYSDIV2 divided the 400MHz PLL down to 200MHz
    SYSCTL[SYSCTL_RCC2_R] |= SYSCTL_RCC2_SYSDIV2 | SYSCTL_RCC2_SYSDIV400;
    //SYSDIV400 and SYSDIV2LSB offer a divisor of 2.5 and brings the 200MHz signal to 80MHz
    SYSCTL[SYSCTL_RCC2_R] &= ~(SYSCTL_RCC2_SYSDIV2LSB);
    //80MHz signal is used undivided, system clock
    SYSCTL[SYSCTL_RCC_R]  |= SYSCTL_RCC_USESYSDIV;

    //4. Wait for the PLL to lock by polling the PLLLRIS bit in the Raw Interrupt Status (RIS) register.
    while(!(SYSCTL[SYSCTL_RIS_R] & SYSCTL_RIS_PLLLRIS));

    //5. Enable use of the PLL by clearing the BYPASS bit in RCC/RCC2.
    SYSCTL[SYSCTL_RCC_R]   &= ~(SYSCTL_RCC2_BYPASS2);
    SYSCTL[SYSCTL_RCC2_R]  &= ~(SYSCTL_RCC2_BYPASS2);

}//end initOSC()

void initGPIO(void){

    //1. Enable the clock to the port by setting the appropriate bits in the RCGCGPIO register (see
    //page 340).
    SYSCTL[SYSCTL_RCGCGPIO] |= SYSCTL_RCGCGPIO_PORTF;
    SYSCTL[SYSCTL_RCGCGPIO] |= SYSCTL_RCGCGPIO_PORTF; //Delay

    //2. Set the direction of the GPIO port pins by programming the GPIODIR register. A write of a 1
    //indicates output and a write of a 0 indicates input.
    GPIO_PORTF[GPIO_DIR] |= GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;

    //6. To enable GPIO pins as digital I/Os, set the appropriate DEN bit in the GPIODEN register. To
    //enable GPIO pins to their analog function (if available), set the GPIOAMSEL bit in the
    //GPIOAMSEL register.
    GPIO_PORTF[GPIO_DEN] |= GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;

}//end initGPIO()

void wait(void){

    for(uint32_t i=0; i<150000; i++){

    }

}//end wait()




