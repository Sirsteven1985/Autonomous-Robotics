

/**
 * Blinky_API.c
 * ECPE 155 Autonomous Robotics
 * Spring 2018
 * Lab 1 - Code Composer 7 Primer
 * Lab Associates:  Steve Guerro
 *                  Paul Vuong
 *
 * 1.   This lab serves as a refresher to Code Composer Studios IDE and the Tiva C Series
 *      family of microcontrollers. Specifically, the TM4C123GH6PM.
 * 2.   Create source code to make one or all three of the LED's blink on the test-board
 *      using the API driver library.
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "C:\ti\TivaWare_C_Series-2.1.3.156\driverlib\sysctl.h"
#include "C:\ti\TivaWare_C_Series-2.1.3.156\driverlib\gpio.h"
#include "inc/hw_memmap.h"

int main(void)
{
    uint32_t i = 0;

    // Enable the GPIOF peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Wait for the GPIOF module to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    // Initialize the GPIO pin configuration.
    // Set pins 1, 2, and 3 as outputs, SW controlled.
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,(GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3));

    // Make pin 1, 2 and 3 high level triggered interrupts.
    GPIOIntEnable(GPIO_PORTF_BASE, (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3));

    while (1)
    {
        //turn red led on and then off
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1, GPIO_PIN_1);
        for(i = 0; i < 150000; i++)
        {}
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1, ~GPIO_PIN_1);
        for(i = 0; i < 150000; i++)
        {}

        //turn blue led on and then off
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, GPIO_PIN_2);
        for(i = 0; i < 150000; i++)
        {}
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, ~GPIO_PIN_2);
        for(i = 0; i < 150000; i++)
        {}

        //turn green led on and then off
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3, GPIO_PIN_3);
        for(i = 0; i < 150000; i++)
        {}
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3, ~GPIO_PIN_3);
        for(i = 0; i < 150000; i++)
        {}
    }

}
