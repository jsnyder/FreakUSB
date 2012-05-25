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

static SI32_PBSTD_A_Type* const usb_ep[] = { SI32_USB_0_EP1, SI32_USB_0_EP2, SI32_USB_0_EP3, SI32_USB_0_EP4 };

/**************************************************************************/
/*!
    Clear all USB related interrupts.
*/
/**************************************************************************/
void intp_clear_all()
{
    U8 i;

    UDINT  = 0;

    for (i=0; i<MAX_EPS; i++)
    {
        ep_select(i);
        UEINTX = 0;
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


    WAKEUP_INT_CLR();
    WAKEUP_INT_DIS();

    // disable the wakeup until next time we go into suspend
    WAKEUP_INT_DIS();
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

/**************************************************************************/
/*!
    This is the ISR that handles communications for the AT90USB. These interrupts
    are endpoint specific and are mostly used for data transfers and communications.
*/
/**************************************************************************/
void USB0_IRQHandler( void )
{
    U8 ep_intp_num, intp_src, ep_num;
    usb_pcb_t *pcb;


    // get the pcb for later use
    pcb = usb_pcb_get();

    // save off the ep number we just had
    ep_num = UENUM;

    if ((ep_intp_num= ep_intp_get_num()) == 0xFF)
    {
        // no intp number was found. restore the ep number and enable the interrupts
        ep_select(ep_num);
        return;
    }

    // select the endpoint number and get the intp src
    ep_select(ep_intp_num);
    if ((intp_src = ep_intp_get_src()) == 0xFF)
    {   
                // no intp source was found. restore the ep number and enable the interrupts
        ep_select(ep_num);
        return;
    }

    switch (intp_src)
    {
    case OPRDYI: // Out packet ready to be read
        // mark a flag as having data in the EP bank that needs to be transferred to the fifo. 
        // we need to get out of ISR to transfer data because we need to check that the fifo 
        // has space to put the data. if it's a ctrl endpoint, then we'll just read the data 
        // directly.
        if (ep_intp_num != EP_CTRL)
        {
            pcb->pending_data |= (1 << ep_intp_num);

            // clear the intps
            SI32_USBEP_A_clear_outpacket_ready( usb_ep[ ep_num - 1 ] );
        }
        else
        {
            ep_read(ep_intp_num);

            // clear the intps
            SI32_USB_A_clear_out_packet_ready_ep0( SI32_USB_0 );
        }
        break;
    case IPRDYI: // FIFO has space or transmission completed
        ep_write(ep_intp_num);

        // clear the intps -- not sure if I need to do this?
        break;
    case SUSI:
        intp_suspend();
        break;
    case RESI:
        intp_resume();
        break;
    case RSTI:
        intp_eor();
        break;
    case SOFI:
    default:
        break;
    }

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
