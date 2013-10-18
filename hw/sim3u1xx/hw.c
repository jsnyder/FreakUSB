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
#include "platform_conf.h"

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

// void hard_fault_handler_c(unsigned int * hardfault_args)
// {
//   unsigned int stacked_r0;
//   unsigned int stacked_r1;
//   unsigned int stacked_r2;
//   unsigned int stacked_r3;
//   unsigned int stacked_r12;
//   unsigned int stacked_lr;
//   unsigned int stacked_pc;
//   unsigned int stacked_psr;

//   stacked_r0 = ((unsigned long) hardfault_args[0]);
//   stacked_r1 = ((unsigned long) hardfault_args[1]);
//   stacked_r2 = ((unsigned long) hardfault_args[2]);
//   stacked_r3 = ((unsigned long) hardfault_args[3]);

//   stacked_r12 = ((unsigned long) hardfault_args[4]);
//   stacked_lr = ((unsigned long) hardfault_args[5]);
//   stacked_pc = ((unsigned long) hardfault_args[6]);
//   stacked_psr = ((unsigned long) hardfault_args[7]);

//   // printf ("[Hard fault handler]\n");
//   // printf ("R0 = %x\n", stacked_r0);
//   // printf ("R1 = %x\n", stacked_r1);
//   // printf ("R2 = %x\n", stacked_r2);
//   // printf ("R3 = %x\n", stacked_r3);
//   // printf ("R12 = %x\n", stacked_r12);
//   // printf ("LR = %x\n", stacked_lr);
//   // printf ("PC = %x\n", stacked_pc);
//   // printf ("PSR = %x\n", stacked_psr);
//   // printf ("BFAR = %x\n", (*((volatile unsigned  *)(0xE000ED38))));
//   // printf ("CFSR = %x\n", (*((volatile unsigned  *)(0xE000ED28))));
//   // printf ("HFSR = %x\n", (*((volatile unsigned  *)(0xE000ED2C))));
//   // printf ("DFSR = %x\n", (*((volatile unsigned  *)(0xE000ED30))));
//   // printf ("AFSR = %x\n", (*((volatile unsigned  *)(0xE000ED3C))));

//   while (1) { ;; }
// }

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
// void HardFault_Handler(void)
// {
//   asm(
//   "tst lr, #4\n\t"
//   "ite eq\n\t"
//   "mrseq r0, msp\n\t"
//   "mrsne r0, psp\n\t"
//   "b hard_fault_handler_c"
//   );
// }

void mySystemInit()
{
  SI32_WDTIMER_A_stop_counter(SI32_WDTIMER_0);

  // enable APB clock to the Port Bank module
  SI32_CLKCTRL_A_enable_apb_to_modules_0 (SI32_CLKCTRL_0, SI32_CLKCTRL_A_APBCLKG0_PB0CEN_MASK);
  // make the SWO pin (PB1.3) push-pull to enable SWV printf
  //SI32_PBSTD_A_set_pins_push_pull_output (SI32_PBSTD_1, (1<<3));
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
                           

  SI32_WDTIMER_A_stop_counter(SI32_WDTIMER_0);

#ifdef PCB_V7
  int pcb_v7_is_defined; //Will show a compiler warning to note hardware version 
  // Setup PBHD4
  SI32_PBCFG_A_unlock_ports(SI32_PBCFG_0);
  SI32_PBHD_A_write_pblock(SI32_PBHD_4, 0x00);

  SI32_PBHD_A_select_pin0_safe_state(SI32_PBHD_4, 0x0);
  SI32_PBHD_A_select_pin1_safe_state(SI32_PBHD_4, 0x0);
  SI32_PBHD_A_select_pin2_safe_state(SI32_PBHD_4, 0x0);
  SI32_PBHD_A_select_pin3_safe_state(SI32_PBHD_4, 0x0);
  SI32_PBHD_A_select_pin4_safe_state(SI32_PBHD_4, 0x0);
  SI32_PBHD_A_select_pin5_safe_state(SI32_PBHD_4, 0x0);

  SI32_PBHD_A_enable_bias(SI32_PBHD_4);
  SI32_PBHD_A_select_normal_power_port_mode(SI32_PBHD_4);
  SI32_PBHD_A_enable_drivers(SI32_PBHD_4);

  //Setup PB4.3 LED0/1
  //Setup PB4.2 to LOW to turn on mosfets for bat charger!
  SI32_PBHD_A_set_pins_push_pull_output( SI32_PBHD_4, 0x000C );
  SI32_PBHD_A_disable_pullup_resistors( SI32_PBHD_4 );
  SI32_PBHD_A_write_pins_low( SI32_PBHD_4, 0x04 );
  SI32_PBHD_A_write_pins_high( SI32_PBHD_4, 0x08 );
#else
  int pcb_v5_is_defined; //Will show a compiler warning to note hardware version 
#endif




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
