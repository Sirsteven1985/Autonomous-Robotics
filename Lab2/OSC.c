/*
 * OSC.c
 *
 *  Created on: Jan 30, 2018
 *      Author: vuong
 */

#include <stdint.h>
#include "SYSCTL.h"
#include "OSC.h"

void initOSC(void) {

    //The PLL is configured using direct register writes to the RCC/RCC2 register. If the RCC2 register
    //is being used, the USERCC2 bit must be set and the appropriate RCC2 bit/field is used. The steps
    //required to successfully change the PLL-based system clock are (pp.231):

    //1. Bypass the PLL and system clock divider by setting the BYPASS bit and clearing the USESYS
    //bit in the RCC register, thereby configuring the microcontroller to run off a "raw" clock source
    //and allowing for the new PLL configuration to be validated before switching the system clock
    //to the PLL.

    //Extend RCC register fields to RCC2.
    //Bitfields in RCC2 with similar functions to
    //RCC2 will take precedence if USERCC2 bit is set.
    SYSCTL[SYSCTL_RCC2_R]  |= SYSCTL_RCC2_USERCC2;
    //Bypass PLL while configuring
    //SYSCTL[SYSCTL_RCC_R]   |= SYSCTL_RCC2_BYPASS2;
    SYSCTL[SYSCTL_RCC2_R]  |= SYSCTL_RCC2_BYPASS2;
    //Raw clock source used while configuring PLL.
    SYSCTL[SYSCTL_RCC_R]   &= ~(SYSCTL_RCC_USESYSDIV);

    //2. Select the crystal value (XTAL) and oscillator source (OSCSRC), and clear the PWRDN bit in
    //RCC/RCC2. Setting the XTAL field automatically pulls valid PLL configuration data for the
    //appropriate crystal, and clearing the PWRDN bit powers and enables the PLL and its output.

    //16Mhz crystal oscillator chosen for PLL
    SYSCTL[SYSCTL_RCC_R]   |= SYSCTL_RCC_XTAL;
    //Main oscillator source selected
    //SYSCTL[SYSCTL_RCC_R]   |= SYSCTL_RCC2_OSCSRC2;
    SYSCTL[SYSCTL_RCC2_R]  |= SYSCTL_RCC2_OSCSRC2;
    //Enable PLL and output
    //SYSCTL[SYSCTL_RCC_R]   &= ~(SYSCTL_RCC2_PWRDN2);
    SYSCTL[SYSCTL_RCC2_R]  &= ~(SYSCTL_RCC2_PWRDN2);

    //3. Select the desired system divider (SYSDIV) in RCC/RCC2 and set the USESYS bit in RCC. The
    //SYSDIV field determines the system frequency for the microcontroller.

    //SYSDIV2 divided the 400MHz PLL down to 200MHz
    SYSCTL[SYSCTL_RCC2_R] |= SYSCTL_RCC2_SYSDIV2 | SYSCTL_RCC2_SYSDIV400;
    //SYSDIV400 and SYSDIV2LSB offer a divisor of 2.5 and brings the 200MHz signal to 80MHz
    SYSCTL[SYSCTL_RCC2_R] &= ~(SYSCTL_RCC2_SYSDIV2LSB);
    //80MHz signal is used undivided, system clock
    SYSCTL[SYSCTL_RCC_R]  |= SYSCTL_RCC_USESYSDIV;
    //Enable functionality the PWM modules, 10MHz clock enabled for PWM modules
    SYSCTL[SYSCTL_RCC_R]  |= SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV;

    //4. Wait for the PLL to lock by polling the PLLLRIS bit in the Raw Interrupt Status (RIS) register.
    while(!(SYSCTL[SYSCTL_RIS_R] & SYSCTL_RIS_PLLLRIS));

    //5. Enable use of the PLL by clearing the BYPASS bit in RCC/RCC2.
    //SYSCTL[SYSCTL_RCC_R]   &= ~(SYSCTL_RCC2_BYPASS2);
    SYSCTL[SYSCTL_RCC2_R]  &= ~(SYSCTL_RCC2_BYPASS2);

}

//*****************************************************************************
//
//! Gets the processor clock rate.
//!
//! This function determines the clock rate of the processor clock, which is
//! also the clock rate of the peripheral modules (with the exception of
//! PWM, which has its own clock divider; other peripherals may have different
//! clocking, see the device data sheet for details).
//!
//! \note This cannot return accurate results if SysCtlClockSet() has not
//! been called to configure the clocking of the device, or if the device is
//! directly clocked from a crystal (or a clock source) that is not one of the
//! supported crystal frequencies.  In the latter case, this function should be
//! modified to directly return the correct system clock rate.
//!
//! \note This function can only be called on TM4C123 devices.  For TM4C129
//! devices, the return value from SysCtlClockFreqSet() indicates the system
//! clock frequency.
//!
//! \return The processor clock rate for TM4C123 devices only.
//
//*****************************************************************************
/* uint32_t
SysCtlClockGet(void)
{
    uint32_t ui32RCC, ui32RCC2, ui32PLL, ui32Clk, ui32Max;
    uint32_t ui32PLL1;

    //
    // This function is only valid on TM4C123 devices.
    //
    //ASSERT(CLASS_IS_TM4C123);

    //
    // Read RCC and RCC2.
    //
    ui32RCC = HWREG(SYSCTL_RCC_R);
    ui32RCC2 = HWREG(SYSCTL_RCC2_R);

    //
    // Get the base clock rate.
    //
    switch((ui32RCC2 & SYSCTL_RCC2_USERCC2) ?
           (ui32RCC2 & SYSCTL_RCC2_OSCSRC2) :
           (ui32RCC & SYSCTL_RCC_OSCSRC_M))
    {
        //
        // The main oscillator is the clock source.  Determine its rate from
        // the crystal setting field.
        //
        case SYSCTL_RCC_OSCSRC_MAIN:
        {
            ui32Clk = g_pui32Xtals[(ui32RCC & SYSCTL_RCC_XTAL_M) >>
                                   SYSCTL_RCC_XTAL_S];
            break;
        }

        //
        // The internal oscillator is the source clock.
        //
        case SYSCTL_RCC_OSCSRC_INT:
        {
            //
            // The internal oscillator on all devices is 16 MHz.
            //
            ui32Clk = 16000000;
            break;
        }

        //
        // The internal oscillator divided by four is the source clock.
        //
        case SYSCTL_RCC_OSCSRC_INT4:
        {
            //
            // The internal oscillator on all devices is 16 MHz.
            //
            ui32Clk = 16000000 / 4;
            break;
        }

        //
        // The internal 30-KHz oscillator is the source clock.
        //
        case SYSCTL_RCC_OSCSRC_30:
        {
            //
            // The internal 30-KHz oscillator has an accuracy of +/- 30%.
            //
            ui32Clk = 30000;
            break;
        }

        //
        // The 32.768-KHz clock from the hibernate module is the source clock.
        //
        case SYSCTL_RCC2_OSCSRC2_32:
        {
            ui32Clk = 32768;
            break;
        }

        //
        // An unknown setting, so return a zero clock (that is, an unknown
        // clock rate).
        //
        default:
        {
            return(0);
        }
    }

    //
    // Default the maximum frequency to the maximum 32-bit unsigned value.
    //
    ui32Max = 0xffffffff;

    //
    // See if the PLL is being used.
    //
    if(((ui32RCC2 & SYSCTL_RCC2_USERCC2) &&
        !(ui32RCC2 & SYSCTL_RCC2_BYPASS2)) ||
       (!(ui32RCC2 & SYSCTL_RCC2_USERCC2) && !(ui32RCC & SYSCTL_RCC_BYPASS)))
    {
        //
        // Read the two PLL frequency registers.  The formula for a
        // TM4C123 device is "(xtal * m) / ((q + 1) * (n + 1))".
        //
        ui32PLL = HWREG(SYSCTL_PLLFREQ0);
        ui32PLL1 = HWREG(SYSCTL_PLLFREQ1);

        //
        // Divide the input clock by the dividers.
        //
        ui32Clk /= ((((ui32PLL1 & SYSCTL_PLLFREQ1_Q_M) >>
                      SYSCTL_PLLFREQ1_Q_S) + 1) *
                    (((ui32PLL1 & SYSCTL_PLLFREQ1_N_M) >>
                      SYSCTL_PLLFREQ1_N_S) + 1) * 2);

        //
        // Multiply the clock by the multiplier, which is split into an
        // integer part and a fractional part.
        //
        ui32Clk = ((ui32Clk * ((ui32PLL & SYSCTL_PLLFREQ0_MINT_M) >>
                               SYSCTL_PLLFREQ0_MINT_S)) +
                   ((ui32Clk * ((ui32PLL & SYSCTL_PLLFREQ0_MFRAC_M) >>
                                SYSCTL_PLLFREQ0_MFRAC_S)) >> 10));

        //
        // Force the system divider to be enabled.  It is always used when
        // using the PLL, but in some cases it does not read as being enabled.
        //
        ui32RCC |= SYSCTL_RCC_USESYSDIV;

        //
        // Calculate the maximum system frequency.
        //
        switch(HWREG(SYSCTL_DC1) & SYSCTL_DC1_MINSYSDIV_M)
        {
            case SYSCTL_DC1_MINSYSDIV_80:
            {
                ui32Max = 80000000;
                break;
            }
            case SYSCTL_DC1_MINSYSDIV_50:
            {
                ui32Max = 50000000;
                break;
            }
            case SYSCTL_DC1_MINSYSDIV_40:
            {
                ui32Max = 40000000;
                break;
            }
            case SYSCTL_DC1_MINSYSDIV_25:
            {
                ui32Max = 25000000;
                break;
            }
            case SYSCTL_DC1_MINSYSDIV_20:
            {
                ui32Max = 20000000;
                break;
            }
            default:
            {
                break;
            }
        }
    }

    //
    // See if the system divider is being used.
    //
    if(ui32RCC & SYSCTL_RCC_USESYSDIV)
    {
        //
        // Adjust the clock rate by the system clock divider.
        //
        if(ui32RCC2 & SYSCTL_RCC2_USERCC2)
        {
            if((ui32RCC2 & SYSCTL_RCC2_DIV400) &&
               (((ui32RCC2 & SYSCTL_RCC2_USERCC2) &&
                 !(ui32RCC2 & SYSCTL_RCC2_BYPASS2)) ||
                (!(ui32RCC2 & SYSCTL_RCC2_USERCC2) &&
                 !(ui32RCC & SYSCTL_RCC_BYPASS))))

            {
                ui32Clk = ((ui32Clk * 2) / (((ui32RCC2 &
                                              (SYSCTL_RCC2_SYSDIV2_M |
                                               SYSCTL_RCC2_SYSDIV2LSB)) >>
                                             (SYSCTL_RCC2_SYSDIV2_S - 1)) +
                                            1));
            }
            else
            {
                ui32Clk /= (((ui32RCC2 & SYSCTL_RCC2_SYSDIV2_M) >>
                             SYSCTL_RCC2_SYSDIV2_S) + 1);
            }
        }
        else
        {
            ui32Clk /= (((ui32RCC & SYSCTL_RCC_SYSDIV_M) >>
                         SYSCTL_RCC_SYSDIV_S) + 1);
        }
    }

    //
    // Limit the maximum clock to the maximum clock frequency.
    //
    if(ui32Max < ui32Clk)
    {
        ui32Clk = ui32Max;
    }

    //
    // Return the computed clock rate.
    //
    return(ui32Clk);
}*/
