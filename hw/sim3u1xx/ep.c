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

#include "freakusb.h"
#include "hw.h"
#include "sim3u1xx.h"
#include "sim3u1xx_Types.h"


static SI32_PBSTD_A_Type* const usb_ep[] = { SI32_USB_0_EP1, SI32_USB_0_EP2, SI32_USB_0_EP3, SI32_USB_0_EP4 };

/**************************************************************************/
/*!
  Select the endpoint number so that we can see the endpoint's associated
  registers.
*/
/**************************************************************************/
void ep_select(U8 ep_num)
{
    // Don't need this
}

/**************************************************************************/
/*!
  Get the max packet size of the endpoint.
*/
/**************************************************************************/
U8 ep_size_get(U8 ep_num)
{
    if( ep_dir_get( ep_num ) == DIR_IN )
        return SI32_USBEP_A_get_in_max_packet_size( usb_ep[ ep_num - 1 ] );
    else
        return SI32_USBEP_A_get_out_max_packet_size( usb_ep[ ep_num - 1 ] );
}

/**************************************************************************/
/*!
  Get the direction of the endpoint.
*/
/**************************************************************************/
U8 ep_dir_get(U8 ep_num)
{
    return (UECFG0X & 0x1);
}

/**************************************************************************/
/*!
  Get the endpoint type: BULK, CONTROL, INTERRUPT, ISOCHRONOUS
*/
/**************************************************************************/
U8 ep_type_get(U8 ep_num)
{
    if( ep_num == 0 )
        return CONTROL;
    
    return BULK;
    // FIXME
}

/**************************************************************************/
/*!
  Clear the endpoint configuration registers
*/
/**************************************************************************/
void ep_cfg_clear()
{
    //FIXME
}

/**************************************************************************/
/*!
  Clear the specified endpoint's enable bit.
*/
/**************************************************************************/
void ep_disable(U8 ep_num)
{
    switch ( ep_num )
    {
    case 1:
        SI32_USB_A_disable_ep1(SI32_USB_0);
        break;
    case 2:
        SI32_USB_A_disable_ep2(SI32_USB_0);
        break;
    case 3:
        SI32_USB_A_disable_ep3(SI32_USB_0);
        break;
    case 4:
        SI32_USB_A_disable_ep4(SI32_USB_0);
        break;
    }
}

/**************************************************************************/
/*!
  Configure the endpoint with the specified parameters.
*/
/**************************************************************************/


void ep_config(U8 ep_num, U8 type, U8 dir, U8 size)
{
    // init the direction the fifo
    usb_buf_init(ep_num,  dir);

    // select the endpoint and reset it
    // FIXME

    if( ep_num > 0 )
    {
        if( dir == DIR_OUT )
        {        
            switch( type )
            {
            case ISOCHRONOUS:
                SI32_USBEP_A_enable_out_isochronous_mode( usb_ep[ ep_num - 1 ] );
                break;
            case BULK:
            case INTP:
                SI32_USBEP_A_select_out_bulk_interrupt_mode( usb_ep[ ep_num - 1 ] );
                break;
            case CONTROL:
                // only for EP0
            }
            SI32_USBEP_A_set_endpoint_direction_out( usb_ep[ ep_num - 1 ] );
            SI32_USBEP_A_clear_out_data_underrun( usb_ep[ ep_num - 1 ] );
            SI32_USBEP_A_stop_out_stall( usb_ep[ ep_num - 1 ] );
            SI32_USBEP_A_reset_out_data_toggle( usb_ep[ ep_num - 1 ] );
            SI32_USBEP_A_set_out_max_packet_size(usb_ep[ ep_num - 1 ], (1 << size)*8 );
        }
        else
        {
            SI32_USBEP_A_set_endpoint_direction_in( usb_ep[ ep_num - 1 ] );
            SI32_USBEP_A_clear_in_data_underrun( usb_ep[ ep_num - 1 ] );
            switch( type )
            {
            case ISOCHRONOUS:
                SI32_USBEP_A_enable_in_isochronous_mode( usb_ep[ ep_num - 1 ] );
                break;
            case BULK:
            case INTP:
                SI32_USBEP_A_select_in_bulk_interrupt_mode( usb_ep[ ep_num - 1 ] );
                SI32_USBEP_A_stop_in_stall( usb_ep[ ep_num - 1 ] );
                SI32_USBEP_A_reset_in_data_toggle( usb_ep[ ep_num - 1 ] );
                break;
            case CONTROL:
                // only for EP0        
            }
            SI32_USBEP_A_set_in_max_packet_size(usb_ep[ ep_num - 1 ], (1 << size)*8 );
        }
    }

    // Enable endpoints
    switch ( ep_num )
    {
    case 1:
        SI32_USB_A_enable_ep1(SI32_USB_0);
        break;
    case 2:
        SI32_USB_A_enable_ep2(SI32_USB_0);
        break;
    case 3:
        SI32_USB_A_enable_ep3(SI32_USB_0);
        break;
    case 4:
        SI32_USB_A_enable_ep4(SI32_USB_0);
    }


    NVIC_EnableIRQ(USB0_IRQn);

    if (ep_num == EP_CTRL)
    {
        RX_SETUP_INT_ENB();
        RX_OUT_INT_ENB();
    }
    else if (dir == DIR_OUT)
    {
        RX_OUT_INT_ENB();
    }
}

/**************************************************************************/
/*!
  Write into the endpoint's FIFOs. These will be used to transfer data out
  of that particular endpoint to the host.
*/
/**************************************************************************/
void ep_write(U8 ep_num)
{
    U8 i, ep_size, len;
    usb_pcb_t *pcb = usb_pcb_get();

    ep_size = ep_size_get(ep_num);
    len = pcb->fifo[ep_num].len;

    // make sure that the tx fifo is ready to receive the out data

    if( ep_num > 0 )
    {
        for (i=0; i<len; i++)
        {
            // check if we've reached the max packet size for the endpoint
            if (i == ep_size)
            {
                // we've filled the max packet size so break and send the data
                break;
            }

            SI32_USBEP_A_write_fifo_u8( usb_ep[ ep_num - 1 ], usb_buf_read( ep_num ) );
        }

        // clearing these two will send the data out
        SI32_USBEP_A_clear_in_data_underrun( usb_ep[ ep_num - 1 ] );
        SI32_USBEP_A_set_in_packet_ready( usb_ep[ ep_num - 1 ] );
    }
}

/**************************************************************************/
/*!
  Read data from the endpoint's FIFO. This is where data coming into the
  device from the host is stored.
*/
/**************************************************************************/
void ep_read(U8 ep_num)
{
    U8 i, len;
    usb_pcb_t *pcb = usb_pcb_get();

    if( ep_num > 0 )
    {
        len = SI32_USBEP_A_read_data_count( usb_ep[ ep_num - 1 ] );

        for (i=0; i<len; i++)
        {
            usb_buf_write(ep_num,  SI32_USBEP_A_read_fifo_u8( usb_ep[ ep_num - 1 ] ));
        }

        if ( 0==SI32_USBEP_A_read_data_count(ep))
        {
            // Clear overrun out overrun if it has occured
            if ( SI32_USBEP_A_has_out_data_overrun_occurred( usb_ep[ ep_num - 1 ] ) )
                SI32_USBEP_A_clear_out_data_overrun( usb_ep[ ep_num - 1 ] );

            SI32_USBEP_A_clear_outpacket_ready( usb_ep[ ep_num - 1 ] );
        }

        if (len > 0)
        {
            pcb->flags |= (ep_num == 0) ? (1<<SETUP_DATA_AVAIL) : (1<<RX_DATA_AVAIL);
        }
    }
}

/**************************************************************************/
/*!
  Send a zero length packet on the specified endpoint. This is usually used
  to terminate a transfer.
*/
/**************************************************************************/
void ep_send_zlp(U8 ep_num)
{
    if( ep_num == 0 )
    {
        SI32_USB_A_set_data_end_ep0( SI32_USB_0 );
        // Service Setup Packet
        SI32_USB_A_clear_out_packet_ready_ep0( SI32_USB_0 );
    }
}

/**************************************************************************/
/*!
  Stall the endpoint due to some problem. This function will also set the
  stall flag in the protocol control block.
*/
/**************************************************************************/
void ep_set_stall(U8 ep_num)
{
    usb_pcb_t *pcb = usb_pcb_get();

    pcb->ep_stall |= (1 << ep_num);
    if( ep_num == 0 )
        SI32_USB_A_send_stall_ep0( SI32_USB_0 );
    else
    {
        if( ep_dir_get() == DIR_IN )
        {
            SI32_USBEP_A_clear_in_stall_sent( usb_ep[ ep_num - 1 ] );
            SI32_USBEP_A_send_in_stall( usb_ep[ ep_num - 1 ] );
        }
        else
        {
            SI32_USBEP_A_clear_out_stall_sent( usb_ep[ ep_num - 1 ] );
            SI32_USBEP_A_send_out_stall( usb_ep[ ep_num - 1 ] );
        }
    }
}

/**************************************************************************/
/*!
  This function will clear the stall on an endpoint.
*/
/**************************************************************************/
void ep_clear_stall(U8 ep_num)
{
    usb_pcb_t *pcb = usb_pcb_get();

    pcb->ep_stall &= ~(1 << ep_num);
    if( ep_num == 0 )
        SI32_USB_A_clear_stall_sent_ep0( SI32_USB_0 );
    else
    {
        if( ep_dir_get() == DIR_IN )
        {

            SI32_USBEP_A_reset_in_data_toggle( usb_ep[ ep_num - 1 ] );
            SI32_USBEP_A_stop_in_stall( usb_ep[ ep_num - 1 ] );
        }
        else
        {
            SI32_USBEP_A_reset_out_data_toggle( usb_ep[ ep_num - 1 ] );
            SI32_USBEP_A_stop_out_stall( usb_ep[ ep_num - 1 ] );
        }
    }
}

/**************************************************************************/
/*!
  Reset the data toggle on the specified endpoint.
*/
/**************************************************************************/
void ep_reset_toggle(U8 ep_num)
{
    if( ep_num > 0 )
        SI32_USBEP_A_reset_out_data_toggle( usb_ep[ ep_num - 1 ] );
}

/**************************************************************************/
/*!
  Clear all endpoints and initialize ep0 for control transfers.
*/
/**************************************************************************/
void ep_init()
{
    U8 i;

    // disable and clear all endpoints
    for (i=0; i<MAX_EPS; i++)
    {
        ep_disable(i);
        ep_cfg_clear(i);
    }

    // reset all the endpoints
    UERST = 0x7F;
    UERST = 0;

    // configure the control endpoint first since that one is needed for enumeration
    ep_config(EP_CTRL, CONTROL, DIR_OUT, MAX_PACKET_SZ);

    // set the rx setup interrupt to received the enumeration interrupts
    //ep_select(EP_CTRL);
}

/**************************************************************************/
/*!
  Set the address for the device. This will be called when a SET_ADDR request
  is sent by the host. We can only set the address after we send a ZLP to the host
  informing it that we successfully received the request. Otherwise, we will
  be on a different address when the host ACKs us back on the original address (0).
*/
/**************************************************************************/
void ep_set_addr(U8 addr)
{
    // send out a zlp to ack the set address request
    ep_send_zlp(EP_CTRL);

    // only write the top 7 bits of the address. the 8th bit is for enable
    SI32_USB_A_write_faddr( SI32_USB_0, addr & 0x7F);

    // Wait for address to be updated
    while( SI32_USB_A_is_function_address_updating( SI32_USB_0 ) );
}


/**************************************************************************/
/*!
  Return the ep where an intp occurred. If no intp occurred, then return 0xff.
*/
/**************************************************************************/
U8 ep_intp_get_num()
{
 
    if( SI32_USB_A_is_ep0_interrupt_pending( SI32_USB_0 ) )
        return 0;

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

/**************************************************************************/
/*!
  Get the endpoint number that the interrupt occurred on. If no interrupt
  is found, return 0xFF.
*/
/**************************************************************************/
// It seems like what this should actually be doing is returning the type of event that has occurred.
//  case RXSTPI: Received SETUP
//  case RXOUTI: Received OUT data
//  case TXINI eady to accept IN data
//  case STALLEDI: CRC error on OUT in isochronous mode or Stalled packet
//  case NAKOUTI: NAK OUT sent(
//  case RWAL: fifo not full or empty
//  case NAKINI: Set by hardware when a NAK handshake has been sent in response of a IN request from the host.
//  case FIFOCON: for IN: Set by hardware when the current bank is free, OUT: Set by hardware when a new OUT message is stored in the current bank,
U8 ep_intp_get_src()
{
    //U8 i;
    if( SI32_USB_A_is_out_packet_ready_ep0( SI32_USB_0 ) )
        return OPRDYI;

    if( SI32_USB_A_is_out_packet_ready_ep0( SI32_USB_0 ) == 0 )
        return IPRDYI;

    if( USB_A_is_suspend_interrupt_pending( SI32_USB_0 ) )
        return SUSI;

    if( SI32_USB_A_is_resume_interrupt_pending( SI32_USB_0 ) )
        return RESI;

    if( SI32_USB_A_is_reset_interrupt_pending( SI32_USB_0 ) )
        return RSTI;

    if( SI32_USB_A_is_start_of_frame_interrupt_pending( SI32_USB_0 ) )
        return SOFI;

    return 0xFF;
}

/**************************************************************************/
/*!
  Drain the fifo of any received data and clear the FIFOCON intp to allow
  it to start receiving more data
*/
/**************************************************************************/
void ep_drain_fifo(U8 ep)
{
    U8 byte_cnt;
    usb_pcb_t *pcb = usb_pcb_get();
    
    ep_select(ep);
    byte_cnt = UEBCLX;
    if (byte_cnt <= (MAX_BUF_SZ - pcb->fifo[ep].len))
    {
        ep_read(ep);
        pcb->pending_data &= ~(1<<ep);
        FIFOCON_INT_CLR();
    }   
}
