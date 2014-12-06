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

//------------------------------------------------------------------------------
// Copyright (c) 2012 by Silicon Laboratories.
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the Silicon Laboratories End User
// License Agreement which accompanies this distribution, and is available at
// http://developer.silabs.com/legal/version/v10/License_Agreement_v10.htm
// Original content and implementation provided by Silicon Laboratories.
//------------------------------------------------------------------------------

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

void hw_wait_ms(U32 delay_amount)
{
   U32 elapsed_time, start_time, last_time;

   elapsed_time = 0;
   start_time = SI32_USB_0->FRAME.U32;

   while (elapsed_time < delay_amount)
   {
      // Find how much time has elapsed
      last_time = SI32_USB_0->FRAME.U32;

      if (last_time >= start_time)
      {
         elapsed_time = last_time - start_time;
      }
      else
      {
         elapsed_time = 2048 + last_time - start_time;
      }
   }
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

  SI32_PBCFG_A_unlock_ports(SI32_PBCFG_0);
  SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_3, ( uint32_t ) 1 << 11);
#if defined( PCB_V8 ) || defined( PCB_V10 )
  SI32_PBSTD_A_write_pins_low(SI32_PBSTD_3, ( uint32_t ) 1 << 11 ); //Set 5V regulator off
#else
  SI32_PBSTD_A_write_pins_high(SI32_PBSTD_3, ( uint32_t ) 1 << 11 ); //Set 5V regulator off
#endif
  SI32_PMU_A_clear_pmu_level_shifter_hold(SI32_PMU_0);
  SI32_PMU_A_clear_pin_level_shifter_hold(SI32_PMU_0);
  SI32_PBSTD_A_disable_pullup_resistors( SI32_PBSTD_0 );
  SI32_PBSTD_A_disable_pullup_resistors( SI32_PBSTD_1 );
  SI32_PBSTD_A_disable_pullup_resistors( SI32_PBSTD_2 );
  SI32_PBSTD_A_disable_pullup_resistors( SI32_PBSTD_3 );
  SI32_PBHD_A_write_pblock(SI32_PBHD_4, 0x00);
  SI32_PBHD_A_disable_pullup_resistors( SI32_PBHD_4 );
}

#if defined( USE_DFU_CLASS )

extern uint32_t SystemCoreClock;
U32 cmsis_get_cpu_frequency()
{
  return SystemCoreClock;
}

static void gTIMER1_enter_auto_reload_config(void)
{
  // ENABLE TIMER1 CLOCK
  SI32_CLKCTRL_A_enable_apb_to_modules_0(SI32_CLKCTRL_0,
    SI32_CLKCTRL_A_APBCLKG0_TIMER1CEN_ENABLED_U32);

  // INITIALIZE TIMER
  SI32_TIMER_A_initialize (SI32_TIMER_1, 0x00, 0x00, 0x00, 0x00);
  SI32_TIMER_A_select_single_timer_mode (SI32_TIMER_1);
  SI32_TIMER_A_select_high_clock_source_apb_clock (SI32_TIMER_1);
  SI32_TIMER_A_select_high_auto_reload_mode (SI32_TIMER_1);

  // Set overflow frequency to SYSTICKHZ
  SI32_TIMER_A_write_capture (SI32_TIMER_1, (unsigned) -(cmsis_get_cpu_frequency()/LEDTICKHZ));
  SI32_TIMER_A_write_count (SI32_TIMER_1, (unsigned) -(cmsis_get_cpu_frequency()/LEDTICKHZ));

  // Run Timer
  SI32_TIMER_A_start_high_timer(SI32_TIMER_1);

  // ENABLE INTERRUPTS
  NVIC_ClearPendingIRQ(TIMER1H_IRQn);
  NVIC_EnableIRQ(TIMER1H_IRQn);
  SI32_TIMER_A_enable_high_overflow_interrupt(SI32_TIMER_1);
}

#endif // USE_DFU_CLASS

#if defined( PCB_V7 )
  #warning "Building for PCB V7"
  //int pcb_v7_is_defined; //Will show a compiler warning to note hardware version

#elif defined( PCB_V8 )
  #warning "Building for PCB V8"
  //int pcb_v8_is_defined;
#elif defined (PCB_V10)
  #warning "Building for PCB V10"
#else
  #error "No PCB Revision Selected"
#endif

void hw_init()
{
  usb_pcb_t *pcb = usb_pcb_get();


  SI32_CLKCTRL_0->APBCLKG0_SET = SI32_CLKCTRL_A_APBCLKG0_PLL0CEN_ENABLED_U32 |
                                 SI32_CLKCTRL_A_APBCLKG0_PB0CEN_ENABLED_U32 |
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

#if defined( USE_DFU_CLASS )

SI32_PMU_A_clear_pmu_level_shifter_hold(SI32_PMU_0);
SI32_PMU_A_clear_pin_level_shifter_hold(SI32_PMU_0);

SI32_WDTIMER_A_stop_counter(SI32_WDTIMER_0);

#if defined( PCB_V7 ) || defined( PCB_V8 ) || defined( PCB_V10 )

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


  SI32_PBHD_A_set_pins_push_pull_output( SI32_PBHD_4, 0x000C );
  SI32_PBHD_A_disable_pullup_resistors( SI32_PBHD_4 );
#if defined( PCB_V8 ) || defined( PCB_V10 )
  //Setup PB4.2 to HIGH to turn on mosfets for bat charger!
  SI32_PBHD_A_write_pins_high( SI32_PBHD_4, 0x04 );
#else
  //Setup PB4.3 LED0/1
  //Setup PB4.2 to LOW to turn on mosfets for bat charger!
  SI32_PBHD_A_write_pins_low( SI32_PBHD_4, 0x04 );
#endif

  SI32_PBCFG_A_enable_crossbar_0(SI32_PBCFG_0);

  // Setup PB2
  SI32_PBSTD_A_disable_pullup_resistors( SI32_PBSTD_2 );
  // PB2.1 is wakeup
  SI32_PBSTD_A_set_pins_digital_input(SI32_PBSTD_2, 0x00000002);

  // Setup PB3
  SI32_PBSTD_A_disable_pullup_resistors( SI32_PBSTD_3 );
  //PB3.9 is usb voltage detection
  SI32_PBSTD_A_set_pins_digital_input(SI32_PBSTD_3, 0x00000200);

#else
  #warning "Building for PCB V5"
  //int pcb_v5_is_defined; //Will show a compiler warning to note hardware version
#endif

#if defined( PCB_V7 )
  SI32_PBSTD_A_write_pins_low(SI32_PBSTD_2, 0x03E0 ); //Set external LEDS 0-4 off
  SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_2, 0x03E0); //Set external LEDS 0-4 as outputs
#endif

#if defined( PCB_V8 )
  SI32_PBSTD_A_write_pins_low(SI32_PBSTD_0, 0x3F0 ); //Set external LEDS 0-4 off
  SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_0, 0x3F0); //Set external LEDS 0-4 as outputs
#endif

#if defined( PCB_V10 )
  SI32_PBSTD_A_write_pins_low(SI32_PBSTD_0, 0x3FF0);
  SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_0, 0x3F00); //Set external LEDS 0-4 as outputs
  SI32_PBSTD_A_set_pins_push_pull_output( SI32_PBSTD_1, 1 << 10);
  SI32_PBSTD_A_write_pins_low( SI32_PBSTD_1, 1 << 10 );
#endif

  gTIMER1_enter_auto_reload_config();

#endif // USE_DFU_CLASS

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

volatile U8 flash_key_mask  = 0x00;
volatile U8 armed_flash_key = 0x00;

U8 hw_flash_erase( U32 address, U8 verify)
{
    flash_key_mask = 0x01;

    // Write the address of the Flash page to WRADDR
    SI32_FLASHCTRL_A_write_wraddr( SI32_FLASHCTRL_0, address );
    // Enter Flash Erase Mode
    SI32_FLASHCTRL_A_enter_flash_erase_mode( SI32_FLASHCTRL_0 );

    // Disable interrupts
    hw_intp_disable();

    // Unlock the flash interface for a single access
    armed_flash_key = flash_key_mask ^ 0xA4;
    SI32_FLASHCTRL_A_write_flash_key(SI32_FLASHCTRL_0, armed_flash_key);
    armed_flash_key = flash_key_mask ^ 0xF0;
    SI32_FLASHCTRL_A_write_flash_key(SI32_FLASHCTRL_0, armed_flash_key);
    armed_flash_key = 0;

    // Write any value to initiate a page erase.
    SI32_FLASHCTRL_A_write_wrdata(SI32_FLASHCTRL_0, 0xA5);

    // Wait for flash operation to complete
    while (SI32_FLASHCTRL_A_is_flash_busy(SI32_FLASHCTRL_0));

    if( verify )
    {
        address &= ~(FLASH_PAGE_SIZE_U8 - 1); // Round down to nearest even page address
        U32* verify_address = (U32*)address;
        U32 wc;

        for( wc = FLASH_PAGE_SIZE_U32; wc != 0; wc-- )
        {
            if ( *verify_address != 0xFFFFFFFF )
                return 1;

            verify_address++;
        }
    }

    hw_intp_enable();

    return 0;
}


U8 hw_flash_write( U32 address, U32* data, U32 count, U8 verify )
{
    U32* tmpdata = data;
    U32 wc;
    flash_key_mask = 0x01;

    // Write the address of the Flash page to WRADDR
    SI32_FLASHCTRL_A_write_wraddr( SI32_FLASHCTRL_0, address );
    // Enter flash erase mode
    SI32_FLASHCTRL_A_exit_flash_erase_mode(SI32_FLASHCTRL_0);

    // disable interrupts
    hw_intp_disable();

    // Unlock flash interface for multiple accesses
    armed_flash_key = flash_key_mask ^ 0xA4;
    SI32_FLASHCTRL_A_write_flash_key(SI32_FLASHCTRL_0, armed_flash_key);
    armed_flash_key = flash_key_mask ^ 0xF3;
    SI32_FLASHCTRL_A_write_flash_key(SI32_FLASHCTRL_0, armed_flash_key);
    armed_flash_key = 0;

    // Write word-sized
    for( wc = count; wc != 0; wc-- )
    {
        SI32_FLASHCTRL_A_write_wrdata( SI32_FLASHCTRL_0, *data );
        SI32_FLASHCTRL_A_write_wrdata( SI32_FLASHCTRL_0, *data >> 16 );
        data++;
    }

    // Relock flash interface
    SI32_FLASHCTRL_A_write_flash_key( SI32_FLASHCTRL_0, 0x5A );

    // Wait for flash operation to complete
    while( SI32_FLASHCTRL_A_is_flash_busy(SI32_FLASHCTRL_0 ) );

    if( verify )
    {
        U32* verify_address = (U32*)address;

        for( wc = count; wc != 0; wc-- )
        {
            if (*verify_address != *tmpdata++)
                return 1;

            verify_address++;
        }
    }

    // re-enable interrupts
    hw_intp_enable();

    return 0;
}



void hw_enable_watchdog( void )
{
    SI32_CLKCTRL_A_enable_apb_to_modules_1(SI32_CLKCTRL_0,
                                           SI32_CLKCTRL_A_APBCLKG1_MISC1CEN_ENABLED_U32);

    SI32_WDTIMER_A_stop_counter(SI32_WDTIMER_0);
    SI32_WDTIMER_A_reset_counter (SI32_WDTIMER_0);
    while(SI32_WDTIMER_A_is_threshold_update_pending(SI32_WDTIMER_0));
    SI32_WDTIMER_A_set_early_warning_threshold (SI32_WDTIMER_0, EARLY_WARNING_THRESHOLD);
    while(SI32_WDTIMER_A_is_threshold_update_pending(SI32_WDTIMER_0));
    SI32_WDTIMER_A_set_reset_threshold (SI32_WDTIMER_0, RESET_THRESHOLD);

    // Enable Watchdog Timer
    SI32_WDTIMER_A_start_counter(SI32_WDTIMER_0);

    /// Enable Watchdog Interrupt
    NVIC_ClearPendingIRQ(WDTIMER0_IRQn);
    NVIC_EnableIRQ(WDTIMER0_IRQn);
    SI32_WDTIMER_A_enable_early_warning_interrupt(SI32_WDTIMER_0);

    SI32_RSTSRC_A_enable_watchdog_timer_reset_source(SI32_RSTSRC_0);
}


void hw_disable_watchdog( void )
{
    SI32_WDTIMER_A_disable_early_warning_interrupt(SI32_WDTIMER_0);
    NVIC_ClearPendingIRQ(WDTIMER0_IRQn);
    NVIC_DisableIRQ(WDTIMER0_IRQn);

    SI32_WDTIMER_A_stop_counter(SI32_WDTIMER_0);
    SI32_RSTSRC_A_disable_watchdog_timer_reset_source(SI32_RSTSRC_0);
}

// Originally based on comments by Marco Accame
// http://www.keil.com/forum/17025/
void hw_boot_image( int usb_started )
{
    volatile uint32_t * image_base = ( volatile uint32_t * )FLASH_TARGET;
    void ( *enter_image )( void );
    volatile U32 down_count;

    if( *image_base != 0xFFFFFFFF )
    {
        if( usb_started )
        {
          SI32_USB_A_disable_internal_pull_up( SI32_USB_0 );
          for( down_count = 0x147AE1; down_count != 0; down_count-- );
        }

        // Set up image entry point function pointer
        // entry point is 1 word after start of image
        enter_image = ( void ( * )( void ) )( *( image_base + 1 ) );

        // Disable USB Interrupts
        NVIC_DisableIRQ( USB0_IRQn );

        // Update interrupt vector table
        SCB->VTOR = FLASH_TARGET;

        // Configure image stack pointer
        __set_MSP( *image_base );

        enter_image();
    }
}

#if defined( USE_DFU_CLASS )

extern U8 led_mask;
extern U8 led_background[LED_COUNT];
extern U8 * led_pending_mode_ptr[LED_COUNT];
extern U8 const * led_cled_ptr[12];
extern U8 led_pending_repeats_ptr[LED_COUNT];
extern U8 * led_mode_ptr[LED_COUNT];

void hw_led_set_mask( U8 mask )
{
#if defined ( MEMBRANE_V1 )
  led_mask = mask;
#else
  // Module Board LED numbers
  led_mask = 0;
  led_mask |= ( ( mask & 1 ) << 3 ); // GPS
  led_mask |= ( ( mask & 1 << 1 ) << 3 ); // MSG
  led_mask |= ( ( mask & 1 << 2 ) >> 1 ); // PWR
  led_mask |= ( ( mask & 1 << 3 ) >> 3 ); // SAT
  led_mask |= ( ( mask & 1 << 4 ) >> 2 ); // ALRM
#endif
}

void hw_led_set_background(int led, U8 bkgnd)
{
  if(led > LED_COUNT)
    return;
  if(bkgnd > 15)
    bkgnd = 15;

  led_background[led]  = bkgnd;
}

void hw_led_set_mode(int led, int mode, int cycles)
{
  if(led > LED_COUNT)
    return;
  if(cycles > 255)
    cycles = 255;
  led_pending_mode_ptr[led] = (U8 *)led_cled_ptr[mode];
  led_pending_repeats_ptr[led] = cycles;
}

int hw_led_get_mode(int led)
{
  if(led > LED_COUNT)
    return -1;

  int i;
  if(led_pending_mode_ptr[led] != NULL)
  {
    for(i=0;i<( sizeof( led_cled_ptr ) / sizeof( led_cled_ptr[0] ) );i++)
    {
      if(led_pending_mode_ptr[led] == led_cled_ptr[i])
        return i;
    }
  } else {
    for(i=0;i<( sizeof( led_cled_ptr ) / sizeof( led_cled_ptr[0] ) );i++)
    {
      if(led_mode_ptr[led] == led_cled_ptr[i])
        return i;
    }
  }
  return -1;
}

static U8 toggle = 0;
void hw_state_indicator( U32 state )
{
  switch( state )
  {
    case HW_STATE_COUNTDOWN:
        hw_led_set_mask( 0xff );
        toggle ^= 1;
        if( toggle )
          hw_led_set_mode(LED_COLOR_PWR, LED_ON, LED_CONTINUOUS);
        else
          hw_led_set_mode(LED_COLOR_PWR, LED_OFF, LED_CONTINUOUS);
        break;
    case HW_STATE_CONNECTED:
        hw_led_set_mode(LED_COLOR_PWR, LED_ON, LED_CONTINUOUS);
        hw_led_set_mode(LED_COLOR_GPS, LED_SLOWFLASH, LED_CONTINUOUS);
        break;
    case HW_STATE_TRANSFER:
        hw_led_set_mode(LED_COLOR_PWR, LED_ON, LED_CONTINUOUS);
        hw_led_set_mode(LED_COLOR_GPS, LED_ON, LED_CONTINUOUS);
        hw_led_set_mode(LED_COLOR_SAT, LED_FASTFLASH, LED_CONTINUOUS);
        break;
    case HW_STATE_DONE:
        hw_led_set_mode(LED_COLOR_PWR, LED_ON, LED_CONTINUOUS);
        hw_led_set_mode(LED_COLOR_GPS, LED_ON, LED_CONTINUOUS);
        hw_led_set_mode(LED_COLOR_SAT, LED_ON, LED_CONTINUOUS);
        hw_led_set_mode(LED_COLOR_MSG, LED_ON, LED_CONTINUOUS);
        break;
    case HW_STATE_ERROR:
        hw_led_set_mode(LED_COLOR_ALRM, LED_ON, LED_CONTINUOUS);
        break;
    case HW_STATE_ERROR_CLR:
        hw_led_set_mode(LED_COLOR_ALRM, LED_OFF, LED_CONTINUOUS);
        break;
  }
}

#endif // USE_DFU_CLASS

int hw_check_skip_bootloader( void )
{
    U32 reset_status = SI32_RSTSRC_0->RESETFLAG.U32;
    U32 pmu_status = SI32_PMU_0->STATUS.U32;

    // If the watchdog, software, pmu or RTC rest us, boot the image
    if ((reset_status & SI32_RSTSRC_A_RESETFLAG_WDTRF_MASK) || // watchdog
        (reset_status & SI32_RSTSRC_A_RESETFLAG_WAKERF_MASK) || // pmu wakeup
        (reset_status & SI32_RSTSRC_A_RESETFLAG_RTC0RF_MASK) || // rtc0 reset
        (reset_status & SI32_RSTSRC_A_RESETFLAG_CMP0RF_MASK)) // comparator reset
    {

        if ((((reset_status & SI32_RSTSRC_A_RESETFLAG_PORRF_MASK)
            || (reset_status & SI32_RSTSRC_A_RESETFLAG_VMONRF_MASK ))) == 0 )
        {
            return 1;
        }
    }

    if( pmu_status & SI32_PMU_A_STATUS_PM9EF_MASK )
        return 1;

    if( ! SI32_VREG_A_is_vbus_valid( SI32_VREG_0 ) )
        return 1;

    return 0;
}

int hw_check_extend_bootloader( void )
{
    U32 reset_status = SI32_RSTSRC_0->RESETFLAG.U32;
    if( reset_status & SI32_RSTSRC_A_RESETFLAG_SWRF_MASK )
    {
        if ((((reset_status & SI32_RSTSRC_A_RESETFLAG_PORRF_MASK)
            || (reset_status & SI32_RSTSRC_A_RESETFLAG_VMONRF_MASK ))) == 0 )
        {
            return 1;
        }
    }
    return 0;
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
