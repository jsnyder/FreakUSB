/*******************************************************************
    Copyright (C) 2009 FreakLabs
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
    3. Neither the name of the the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
    OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.

    Originally written by Christopher Wang aka Akiba.
    Please post support questions to the FreakLabs forum.
*******************************************************************/
/*!
    \defgroup hw_at90usb Hardware - AT90USB
    \file hw.c
    \ingroup hw_at90usb
*/
/*******************************************************************/
#include "sim3u1xx.h"
#include "sim3u1xx_Types.h"
#include "freakusb.h"
#include "types.h"

/**************************************************************************/
/*!
    Initialize the hardware.
    1) Turn off the watchdog timer (in AVR MCUs)
    2) Configure the PLL
    3) Enable the USB and relevant peripherals
    4) Enable the global interrupt.
*/
/**************************************************************************/

void mySystemInit()
{
   // oscillators
   // pmu
   // vreg

   // set Priority for Cortex-M0 System Interrupts.
   NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
   // TBD the rest of them

#if si32BuildOption_enable_logging
   // enable ITM
   // Enable APB clock to the Port Bank module.
   SI32_CLKCTRL_0->APBCLKG0_SET = SI32_CLKCTRL_A_APBCLKG0_PB0CEN_MASK;
   // Make the SWO pin (PB1.3) push-pull to enable SWV printf.
   SI32_PBSTD_1->PBOUTMD_SET = 0x00000008;
#endif // si32BuildOption_enable_logging

  // disable the watchdog.
  SI32_WDTIMER_A_stop_counter(SI32_WDTIMER_0);
}

void hw_init()
{
  SI32_CLKCTRL_0->APBCLKG0_SET = SI32_CLKCTRL_A_APBCLKG0_PLL0CEN_ENABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_PB0CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_USART0CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_USART1CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_UART0CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_UART1CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_SPI0CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_SPI1CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_SPI2CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_I2C0CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_I2C1CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_EPCA0CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_PCA0CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_PCA1CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_SSG0CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_TIMER0CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_TIMER1CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_ADC0CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_ADC1CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_CMP0CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_CMP1CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_CS0CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_AES0CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_CRC0CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_IDAC0CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_IDAC1CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_LPT0CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_I2S0CEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_USB0CEN_ENABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_EVREGCEN_DISABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_FLCTRLCEN_ENABLED_U32;

  SI32_CLKCTRL_0->AHBCLKG_SET = SI32_CLKCTRL_A_AHBCLKG_RAMCEN_ENABLED_U32 |
                                SI32_CLKCTRL_A_AHBCLKG_DMACEN_DISABLED_U32 |
                                SI32_CLKCTRL_A_AHBCLKG_FLASHCEN_ENABLED_U32 |
                                SI32_CLKCTRL_A_AHBCLKG_EMIF0CEN_DISABLED_U32 |
                                SI32_CLKCTRL_A_AHBCLKG_USB0BCEN_ENABLED_U32;
                                

  /* --------------------- */
  /* Initialize USB Module */
  /* --------------------- */
  SI32_USB_A_enable_usb_oscillator(SI32_USB_0);

  // Perform asynchronous reset of the USB module
  SI32_USB_A_reset_module(SI32_USB_0);
  // Wait for reset to complete
  while (SI32_USB_0->CLKSEL.RESET == SI32_USB_A_CLKSEL_RESET_SET_VALUE);

// These defines control initialization for USB
#define force_low_speed false
#define start_attached  true
#define start_enabled   true
  SI32_USB_A_initialize(
     SI32_USB_0,
     // Write to FADDR register.
     0x00000000,
     // Write to POWER register.
     //0x00000011,
     0x00000001 |
     (!start_enabled << SI32_USB_A_POWER_USBINH_SHIFT),
     // Write to IOINT register.
     0x001E001F,
     // Write to CMINT register.
     0x0000000F,
     // Write to IOINTE register.
     0x001E001F,
     // Write to CMINTEPE register.
     0x001F000F,
     // Write to CRCONTROL register.
     0x00000080 | (force_low_speed << SI32_USB_A_CRCONTROL_LSCRMD_SHIFT),
     // Write to TCONTROL register.
     0x00000040 |
     (!force_low_speed << SI32_USB_A_TCONTROL_SSEL_SHIFT) |
     (start_attached << SI32_USB_A_TCONTROL_PUEN_SHIFT),
     // Write to CLKSEL register.
#if (force_low_speed == false)
     0x00000000, // full-speed
#else
     0x00000030, // low-speed
#endif // (force_low_speed == false)
     // Write to OSCCONTROL register.
     0x00000080,
     // Write to DMACONTROL register.
     0x00000000,
     // Write to EP0CONTROL register.
     0x00000000);

  NVIC_EnableIRQ (USB0_IRQn);
}

/**************************************************************************/
/*!
    Returns a byte stored in the flash. Some MCUs can't access the flash
    memory from a standard pointer so they need a special function to handle it.
*/
/**************************************************************************/
U8 hw_flash_get_byte(U32 *addr)
{
    return ( U8 )*addr;
}

/**************************************************************************/
/*!
    Disable global interrupts
*/
/**************************************************************************/
void hw_intp_disable()
{
  __disable_irq();
}

/**************************************************************************/
/*!
    Enable global interrupts
*/
/**************************************************************************/
void hw_intp_enable()
{
  __enable_irq();
}
