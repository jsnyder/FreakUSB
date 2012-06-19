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

#define SI32_USB_A_CLKSEL_TEST_MASK 0x00000080

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
bool SI32_USB_A_verify_clock_is_running(SI32_USB_A_Type * usb)
{
  usb->CLKSEL.U32 |= SI32_USB_A_CLKSEL_TEST_MASK;
  int loop=100;

  while( (usb->CLKSEL.U32 & SI32_USB_A_CLKSEL_TEST_MASK) )
  {
    if( 0==loop--)
    {
      //printf("USB Clock Not Running! clksel = %02x\n", usb->CLKSEL.U32);
      return false;
    }
  }
  //printf("USB Clock is running.  clksel = %02x\n", usb->CLKSEL.U32);
  return true;
}

void mySystemInit()
{
   // oscillators
   // pmu
   // vreg

   // set Priority for Cortex-M0 System Interrupts.
   NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
   // TBD the rest of them

  // disable the watchdog.
  SI32_WDTIMER_A_stop_counter(SI32_WDTIMER_0);
}

void hw_init()
{
  usb_pcb_t *pcb = usb_pcb_get();
  
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
  SI32_USB_A_verify_clock_is_running(SI32_USB_0);
  SI32_USB_A_select_usb_clock_48mhz (SI32_USB_0);

  // Perform asynchronous reset of the USB module
  SI32_USB_A_reset_module(SI32_USB_0);
  // Wait for reset to complete
  while (SI32_USB_0->CLKSEL.RESET == SI32_USB_A_CLKSEL_RESET_SET_VALUE);

  SI32_USB_A_write_cmint (SI32_USB_0, 0x00000000);
  SI32_USB_A_write_ioint (SI32_USB_0, 0x00000000);
  SI32_USB_A_enable_ep0_interrupt (SI32_USB_0);

  // Enable Reset, Resume, Suspend interrupts
  SI32_USB_A_enable_suspend_interrupt (SI32_USB_0);
  SI32_USB_A_enable_resume_interrupt (SI32_USB_0);
  SI32_USB_A_enable_reset_interrupt (SI32_USB_0);
  //SI32_USB_A_enable_start_of_frame_interrupt (SI32_USB_0);

  // Enable Transceiver, fullspeed
  SI32_USB_A_write_tcontrol (SI32_USB_0, 0x00);
  SI32_USB_A_select_transceiver_full_speed (SI32_USB_0);
  SI32_USB_A_enable_transceiver (SI32_USB_0);
  // _SI32_USB_A_enable_internal_pull_up (SI32_USB_0);
  SI32_USB_A_enable_internal_pull_up( SI32_USB_0 );

  // Enable clock recovery, single-step mode disabled
  SI32_USB_A_enable_clock_recovery (SI32_USB_0);
  SI32_USB_A_select_clock_recovery_mode_full_speed (SI32_USB_0);
  SI32_USB_A_select_clock_recovery_normal_cal  (SI32_USB_0);

  SI32_USB_0->CMINTEPE.U32 |= (1<<16) ;
  SI32_USB_0->CMINTEPE.U32 |= 7;

  NVIC_EnableIRQ (USB0_IRQn);

  SI32_USB_A_enable_module( SI32_USB_0 );

  pcb->connected = true;
}

/**************************************************************************/
/*!
    Returns a byte stored in the flash. Some MCUs can't access the flash
    memory from a standard pointer so they need a special function to handle it.
*/
/**************************************************************************/
U8 hw_flash_get_byte(U8 *addr)
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
