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
    \file isr.c
    \ingroup hw_at90usb
*/
/*******************************************************************/

#include "freakusb.h"
#include "hw.h"
#include "sim3u1xx.h"
#include "sim3u1xx_Types.h"

static SI32_USBEP_A_Type* const usb_ep[] = { SI32_USB_0_EP1, SI32_USB_0_EP2, SI32_USB_0_EP3, SI32_USB_0_EP4 };

/**************************************************************************/
/*!
    Clear all USB related interrupts.
*/
/**************************************************************************/
void intp_clear_all()
{
    uint32_t usbCommonInterruptMask = SI32_USB_A_read_cmint(SI32_USB_0);
    uint32_t usbEpInterruptMask = SI32_USB_A_read_ioint(SI32_USB_0);
    
    if (usbCommonInterruptMask)
    {
    // SI32_USB_A_clear_common_interrupts( SI32_USB_0, bCommon );
        {
            volatile uint32_t * CMINT = /* _addr_of */ ((uint32_t *) 0x40018030) ;
            *CMINT = usbCommonInterruptMask ;
        }
    }

    if (usbEpInterruptMask)
    {
        // Note:  Clearing the IRQ simply clears the interrupt line.  It does not
        // clear the interrupt source bit, nor does it relay any info to the hardware
        // about whether or not a packet is available, or has been consumed from the
        // fifo.
        // SI32_USB_A_clear_endpoint_interrupts( SI32_USB_0, usbEpInterruptMask );
        {
          volatile uint32_t * IOINT = /* _addr_of */ ((uint32_t *) 0x40018020) ;
          *IOINT = usbEpInterruptMask;
        }
    }


}

/**************************************************************************/
/*!
    Suspend interrupt handler.
*/
/**************************************************************************/
void intp_suspend()
{
    SI32_USB_A_clear_suspend_interrupt( SI32_USB_0 );

    // freeze the clock and turn off the USB PLL
    SI32_USB_A_suspend_usb_oscillator( SI32_USB_0 );
}

/**************************************************************************/
/*!
    Resume interrupt handler.
*/
/**************************************************************************/
void intp_resume()
{
    SI32_USB_A_clear_resume_interrupt( SI32_USB_0 );
}

/**************************************************************************/
/*!
    Wakeup interrupt handler.
*/
/**************************************************************************/
void intp_wakeup()
{
    // unfreeze the clock
    SI32_USB_A_enable_usb_oscillator( SI32_USB_0 );

    //WAKEUP_INT_CLR();
    //WAKEUP_INT_DIS();

    // disable the wakeup until next time we go into suspend
    //WAKEUP_INT_DIS();
}

/**************************************************************************/
/*!
    End of Reset interrupt handler. Gets triggered at the end of a bus reset.
*/
/**************************************************************************/
void intp_eor()
{
    SI32_USB_A_clear_reset_interrupt( SI32_USB_0 );
    ep_init();
}

U8 get_usbep_num()
{
    if( SI32_USB_A_is_ep1_in_interrupt_pending( SI32_USB_0 ) ||
        SI32_USB_A_is_ep1_out_interrupt_pending( SI32_USB_0 ))
        return 1;

    if( SI32_USB_A_is_ep2_in_interrupt_pending( SI32_USB_0 ) ||
        SI32_USB_A_is_ep2_out_interrupt_pending( SI32_USB_0 ))
        return 2;

    if( SI32_USB_A_is_ep3_in_interrupt_pending( SI32_USB_0 ) ||
        SI32_USB_A_is_ep3_out_interrupt_pending( SI32_USB_0 ))
        return 3;

    if( SI32_USB_A_is_ep4_in_interrupt_pending( SI32_USB_0 ) ||
        SI32_USB_A_is_ep4_out_interrupt_pending( SI32_USB_0 ))
        return 4;

    return 0xFF;
}

void ep0_handler( void )
{
    uint32_t ControlReg = SI32_USB_A_read_ep0control(SI32_USB_0);

    if (ControlReg & SI32_USB_A_EP0CONTROL_STSTLI_MASK)
        ep_clear_stall( 0 );

    if (ControlReg & SI32_USB_A_EP0CONTROL_SUENDI_MASK)
        SI32_USB_A_clear_setup_end_early_ep0( SI32_USB_0 );

    if( SI32_USB_A_is_out_packet_ready_ep0( SI32_USB_0 ) )
    {
        ep_read( 0 );
        return;
    }

    if( SI32_USB_0->EP0CONTROL.IPRDYI == 0 )
    {
        ep_write( 0 );
        //return;
    }
}

void usbep_handler( U8 ep_intp_num )
{
    usb_pcb_t *pcb;

    if( get_usbep_num == 0xFF )
        return;

    // get the pcb for later use
    pcb = usb_pcb_get();

    if( usb_ep[ ep_intp_num - 1 ]->EPCONTROL.ISTSTLI )
        SI32_USBEP_A_clear_in_stall_sent( usb_ep[ ep_intp_num - 1 ] );

    if( usb_ep[ ep_intp_num - 1 ]->EPCONTROL.ISTSTLI )
        SI32_USBEP_A_clear_out_stall_sent( usb_ep[ ep_intp_num - 1 ] );

    if( SI32_USBEP_A_is_outpacket_ready( usb_ep[ ep_intp_num - 1 ] ))
    {
        //ep_read( ep_intp_num );
        //pcb->pending_data |= ( 1 << ep_intp_num );
        ep_read( ep_intp_num );
        return;
    }

    if( usb_ep[ ep_intp_num - 1 ]->EPCONTROL.IPRDYI == 0 )
    {
        ep_write( ep_intp_num );
        //return;
    }
}


/**************************************************************************/
/*!
    This is the ISR that handles communications for the AT90USB. These interrupts
    are endpoint specific and are mostly used for data transfers and communications.
*/
/**************************************************************************/
void USB0_IRQHandler( void )
{

    uint32_t usbCommonInterruptMask = SI32_USB_A_read_cmint(SI32_USB_0);
    uint32_t usbEpInterruptMask = SI32_USB_A_read_ioint(SI32_USB_0);
    U8 ep_intp_num = get_usbep_num();


    // Clear the interrupt sources, then process the interrupts by the mask
    // so that any subsequent interrupt will immediately reenter the IRQHandler
    if (usbCommonInterruptMask)
    {
    // SI32_USB_A_clear_common_interrupts( SI32_USB_0, bCommon );
        {
            volatile uint32_t * CMINT = /* _addr_of */ ((uint32_t *) 0x40018030) ;
            *CMINT = usbCommonInterruptMask ;
        }
    }

    if (usbEpInterruptMask)
    {
        // Note:  Clearing the IRQ simply clears the interrupt line.  It does not
        // clear the interrupt source bit, nor does it relay any info to the hardware
        // about whether or not a packet is available, or has been consumed from the
        // fifo.
        // SI32_USB_A_clear_endpoint_interrupts( SI32_USB_0, usbEpInterruptMask );
        {
          volatile uint32_t * IOINT = /* _addr_of */ ((uint32_t *) 0x40018020) ;
          *IOINT = usbEpInterruptMask;
        }
    }

    if( usbEpInterruptMask & ( SI32_USB_A_IOINT_IN1I_MASK | SI32_USB_A_IOINT_OUT1I_MASK |
                               SI32_USB_A_IOINT_IN2I_MASK | SI32_USB_A_IOINT_OUT2I_MASK |
                               SI32_USB_A_IOINT_IN3I_MASK | SI32_USB_A_IOINT_OUT3I_MASK |
                               SI32_USB_A_IOINT_IN4I_MASK | SI32_USB_A_IOINT_OUT4I_MASK ) )
    {
        usbep_handler( ep_intp_num );
    }
    
    if( usbEpInterruptMask & SI32_USB_A_IOINT_EP0I_MASK )
    {
        ep0_handler();
    }

    if( SI32_USB_A_is_suspend_interrupt_pending( SI32_USB_0 ) )
        intp_suspend();

    if( SI32_USB_A_is_resume_interrupt_pending( SI32_USB_0 ) )
        intp_resume();

    if( SI32_USB_A_is_reset_interrupt_pending( SI32_USB_0 ) )
        intp_eor();
}

/**************************************************************************/
/*!
    This ISR handles general USB functions on the AT90USB. The interrupts
    are sepearated into a general and communications interrupt. The general
    interrupt handles global USB features such as VBUS detection, suspend, resume
    and bus reset.
*/
/**************************************************************************/
/*ISR(USB_GEN_vect)
{
    U8 ep_num;

    cli();
        
        // save off the endpoint number we were just on. we'll restore it later
    ep_num = UENUM;

    // suspend detected
    if (SUSP_INT && SUSP_INT_ENABLED)
    {
        intp_suspend();
    }

    if (RESM_INT && RESM_INT_ENABLED)
    {
        intp_resume();
    }

    // wakeup detected
    if (WAKEUP_INT && WAKEUP_INT_ENABLED)
    {
        intp_wakeup();
    }

    // end of bus reset
    if (EOR_INT && EOR_INT_ENABLED)
    {
        intp_eor();
    }

    // restore the endpoint number
    ep_select(ep_num);

    sei();
}*/

/**************************************************************************/
/*!
    This ISR is only for the AT90USB16 since we need to use an IO as the VBUS sense
*/
/**************************************************************************/
/* ISR(PCINT0_vect)
{
    usb_pcb_t *pcb = usb_pcb_get();

    cli();

    if (is_vbus_on())
    {
        pcb->connected = true;

        // enable the 3.3V regulator for the USB pads
        REGCR &= ~_BV(REGDIS);

        // freeze the clock
        USBCON |= _BV(FRZCLK);
        
        // enable the 48 MHz PLL
        PLLCSR &= ~(_BV(PLLP2) | _BV(PLLP1) | _BV(PLLP0)); 
        PLLCSR |= _BV(1<<PLLE);
    
        // busy wait until the PLL is locked
        while (!(PLLCSR & _BV(PLOCK)));
    
        // unfreeze clock
        USBCON &= ~_BV(FRZCLK);
    
        // attach USB
        UDCON &= ~_BV(DETACH);
    
        // reset CPU
        UDCON |= _BV(RSTCPU);
    }
    else
    {
        // if we're connected, but VBUS is gone, then detach

        // detach from the bus
        UDCON  |= _BV(DETACH);

        // freeze the clock and turn off the USB PLL
        USBCON |= _BV(FRZCLK);
        PLLCSR &= ~_BV(PLLE);

        // disable the USB voltage regulator
        REGCR |= _BV(REGDIS);

        pcb->connected = false;
        pcb->flags = 0;
    }
    sei();
} */
