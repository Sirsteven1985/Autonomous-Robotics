
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "C:\TivaWare\driverlib\sysctl.h"
#include "C:\TivaWare\driverlib\gpio.h"
#include "inc/hw_memmap.h"


/**
 * main.c
 */
int main(void)
{
    uint32_t i = 0;
    //
    // Enable the GPIOA peripheral
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    //
    // Wait for the GPIOA module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }
    //
    // Register the port-level interrupt handler. This handler is the first
    // level interrupt handler for all the pin interrupts
    //
    // Initialize the GPIO pin configuration.
    //
    // Set pins 2, 4, and 5 as input, SW controlled.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,(GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3));
    //
    // Set pins 0 and 3 as output, SW controlled.
    //
    //
    //
    // Make pin 5 high level triggered interrupts.
    //
    // Read some pins.
    //
    //
    GPIOIntEnable(GPIO_PORTF_BASE, (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3));
    //
    // Write some pins. Even though pins 2, 4, and 5 are specified, those pins
    // are unaffected by this write because they are configured as inputs. At
    // the end of this write, pin 0 is low, and pin 3 is high.
    //
    while (1)
    {
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1, GPIO_PIN_1);
        for(i = 0; i < 100000; i++)
        {}
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1, ~GPIO_PIN_1);
        for(i = 0; i < 100000; i++)
        {}


        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, GPIO_PIN_2);
        for(i = 0; i < 100000; i++)
        {}
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, ~GPIO_PIN_2);
        for(i = 0; i < 100000; i++)
        {}


        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3, GPIO_PIN_3);
        for(i = 0; i < 100000; i++)
        {}
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3, ~GPIO_PIN_3);
        for(i = 0; i < 100000; i++)
        {}
    }
    //
    // Enable the pin interrupts.

}
