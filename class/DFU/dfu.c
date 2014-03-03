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

/* *****************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 LeafLabs LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ****************************************************************************/

/*!
    \defgroup dfu_class USB DFU Class
    \file dfu.c
    \ingroup dfu_class
*/
/*******************************************************************/
#include "freakusb.h"
#include "dfu.h"
#include "sim3u1xx.h"
#include "sim3u1xx_Types.h"

volatile U16 dfu_reset_counter = 10;         // 10 warnings are allowed

DFUStatus dfu_status;

// this is the rx handler callback function. it gets registered by the application program
// and will handle any incoming data.
static void (*rx_handler)();

uint32_t flash_buffer[BLOCK_SIZE_U32];
uint32_t* flash_buffer_ptr = flash_buffer;
uint32_t flash_target = FLASH_TARGET;

volatile U8 need_to_write = 0;
volatile U8 dfu_communication_started = 0;

/**************************************************************************/
/*!
    This is the class specific request handler for the USB Comm-unications Device
    Class (DFU). Currently, this class driver only support the Virtual COM Port
    feature of the DFU.
*/
/**************************************************************************/

//                             // bmRequestType, wValue,    wIndex,    wLength, Data
// #define  DFU_DETACH    0x00 // 0x21,          wTimeout,  Interface, Zero,    None
// #define  DFU_DNLOAD    0x01 // 0x21,          wBlockNum, Interface, Length,  Firmware
// #define  DFU_UPLOAD    0x02 // 0xA1,          Zero,      Interface, Length,  Firmware
// #define  DFU_GETSTATUS 0x03 // 0xA1,          Zero,      Interface, 6,       Status
// #define  DFU_CLRSTATUS 0x04 // 0x21,          Zero,      Interface, Zero,    None
// #define  DFU_GETSTATE  0x05 // 0xA1,          Zero,      Interface, 1,       State
// #define  DFU_ABORT     0x06 // 0x21,          Zero,      Interface, Zero,    None

void dfu_req_handler(req_t *req)
{
    U8 i;
    usb_pcb_t *pcb = usb_pcb_get();

    switch (req->req)
    {
    case DFU_DETACH:
        if (req->type & (HOST_TO_DEVICE | TYPE_CLASS | RECIPIENT_INTF))
        {
            // wvalue is wTimeout
            // wLength is zero
            // data is none
            dfu_status.bState = appDETACH;
            dfu_status.bStatus = OK;

        }
        break;

    case DFU_DNLOAD:
        if (req->type & (HOST_TO_DEVICE | TYPE_CLASS | RECIPIENT_INTF))
        {
            // wvalue is wBlockNum
            // wlength is Length
            // data is firmware
            if( dfu_status.bState == dfuIDLE )
            {
                if( req->len > 0 )
                {
                    dfu_status.bState = dfuDNLOAD_SYNC;
                }
                else
                {
                    dfu_status.bState  = dfuERROR;
                    dfu_status.bStatus = errNOTDONE;
                    ep_send_zlp(EP_CTRL);
                    return;
                }
            }
        	i = req->val;
            if( dfu_status.bState == dfuDNLOAD_IDLE )
            {
                if( req->len > 0 )
                {
                    dfu_status.bState = dfuDNLOAD_SYNC;
                }
                else
                {
                    if( flash_buffer_ptr > flash_buffer )
                    {
                        need_to_write = 1;
                        hw_activity_indicator( HW_STATE_TRANSFER );
                        //flash_buffer_ptr = flash_buffer;
                    }
                    dfu_status.bState  = dfuMANIFEST_SYNC;
                    ep_send_zlp(EP_CTRL);
                    return;
                }
            }

            SI32_USB_A_clear_out_packet_ready_ep0(SI32_USB_0);

            while(pcb->fifo[EP_CTRL].len < req->len)
            {
                //ep_read(EP_CTRL);
            	i = pcb->fifo[EP_CTRL].len;
            }

            // clear the setup flag if needed
            pcb->flags &= ~(1<<SETUP_DATA_AVAIL);

            // send out a zero-length packet to ack to the host that we received
            // the new line coding
            U8* byte_buf_ptr = ( U8* )flash_buffer_ptr;
            U8 tmp_len = pcb->fifo[EP_CTRL].len;
            for(i = 0; i < tmp_len; i++)
            {
                *byte_buf_ptr = usb_buf_read(EP_CTRL);
                byte_buf_ptr++;
            }
            flash_buffer_ptr += i/4;

            if( flash_buffer_ptr == flash_buffer + BLOCK_SIZE_U32 )
            {
                // Reset buffer pointer
                //flash_buffer_ptr = flash_buffer;
                need_to_write = 1;
            }

            if( flash_buffer_ptr > flash_buffer + BLOCK_SIZE_U32)
            {
                dfu_status.bState  = dfuERROR;
            }

            ep_send_zlp(EP_CTRL);
        }
        break;

    case DFU_UPLOAD:
        if (req->type & (DEVICE_TO_HOST | TYPE_CLASS | RECIPIENT_INTF))
        {
            // wvalue is zero
            // wlength is length
            // data is firmware
            // NOT SUPPORTED
            ep_set_stall(EP_CTRL);
        }
        break;

    case DFU_GETSTATUS:
        if (req->type & (DEVICE_TO_HOST | TYPE_CLASS | RECIPIENT_INTF))
        {
            if( dfu_communication_started == 0 )
                hw_activity_indicator( HW_STATE_CONNECTED );

            dfu_communication_started = 1;
            // If we're still transmitting blocks
            if( dfu_status.bState == dfuDNLOAD_SYNC )
            {
                if( need_to_write == 0 )
                {
                    dfu_status.bState=dfuDNLOAD_IDLE;
                    dfu_status.bwPollTimeout0 = 0x00;
                }
                else
                {
                	dfu_status.bState=dfuDNBUSY;
                    dfu_status.bwPollTimeout0 = 0x3F;
                }
            }
            else if( dfu_status.bState == dfuDNBUSY )
            {
                if( need_to_write == 0)
                    dfu_status.bState=dfuDNLOAD_SYNC;
            }
            else if( dfu_status.bState == dfuMANIFEST_SYNC)
            {
            	dfu_status.bState=dfuMANIFEST;
                dfu_status.bwPollTimeout0 = 0xFF;
            }
            else if( dfu_status.bState == dfuMANIFEST &&
                     need_to_write == 0)
            {
                // Finish erasing flash
                while( flash_target < SI32_MCU_FLASH_SIZE)
                {
                    if( 0 != hw_flash_erase( flash_target, 1 ) )
                    {
                        dfu_status.bState  = dfuERROR;
                        dfu_status.bStatus = errERASE;
                    }
                    flash_target += BLOCK_SIZE_U8;
                }
                dfu_status.bState=dfuMANIFEST_WAIT_RESET;
            }

            for (i=0; i<STATUS_SZ; i++)
            {
                usb_buf_write(EP_CTRL, *((U8 *)&dfu_status + i));
            }
            ep_write(EP_CTRL);

            if( dfu_status.bState == dfuMANIFEST_WAIT_RESET )
            {
                hw_wait_ms(200);
                SI32_USB_A_disable_internal_pull_up( SI32_USB_0 );
                for (U32 down_count = 0x1FFFFFF; down_count != 0; down_count--);
                hw_boot_image();
            }

            if( need_to_write )
            {
                if( 0 != hw_flash_erase( flash_target, 1 ) )
                {
                    dfu_status.bState  = dfuERROR;
                    dfu_status.bStatus = errERASE;
                }
                if( 0 != hw_flash_write( flash_target, ( U32* )flash_buffer, flash_buffer_ptr - flash_buffer, 1 ) )
                {
                    dfu_status.bState  = dfuERROR;
                    dfu_status.bStatus = errVERIFY;
                }
                flash_buffer_ptr = flash_buffer;
                flash_target += BLOCK_SIZE_U8;
                need_to_write = 0;
                if( dfu_status.bState != dfuMANIFEST )
                    dfu_status.bState=dfuDNLOAD_SYNC;
            }
        }
        break;

    case DFU_CLRSTATUS:
        if (req->type & (HOST_TO_DEVICE | TYPE_CLASS | RECIPIENT_INTF))
        {
            // wvalue is zero
            // wlength is 0
            // data is  none
            if( dfu_status.bState == dfuERROR )
            {
                dfu_status.bStatus = OK;
                dfu_status.bState = dfuIDLE;
            }
        }
        break;

    case DFU_GETSTATE:
        if (req->type & (DEVICE_TO_HOST | TYPE_CLASS | RECIPIENT_INTF))
        {
            // wvalue is zero
            // wlength is 1
            // data is  state
            // Transition?: No State Transition
            usb_buf_write( EP_CTRL, dfu_status.bState );
            ep_write(EP_CTRL);
        }
        break;

    case DFU_ABORT:
        if (req->type & (HOST_TO_DEVICE | TYPE_CLASS | RECIPIENT_INTF))
        {
            // wvalue is zero
            // wlength is 0
            // data is none
            if( dfu_status.bState == dfuIDLE )
            {
                dfu_status.bStatus = OK;
                dfu_status.bState = dfuIDLE;
            }
            else if ( dfu_status.bState == dfuDNLOAD_IDLE )
            {
                flash_target = FLASH_TARGET;
                if( 0 != hw_flash_erase( flash_target, 1 ) )
                {
                    dfu_status.bState  = dfuERROR;
                    dfu_status.bStatus = errERASE;
                }
                dfu_status.bStatus = OK;
                dfu_status.bState = dfuIDLE;
                ep_send_zlp(EP_CTRL);
            }
        }
        break;

    default:
        ep_set_stall(EP_CTRL);
        break;
    }
}

/**************************************************************************/
/*!
    This is the rx data handler for the DFU class driver. This should be
    implemented by the user depending on what they want to do with the incoming
    virtual COM data.
*/
/**************************************************************************/
void dfu_rx_handler()
{
    if (rx_handler)
    {
        rx_handler();
    }
}

/**************************************************************************/
/*!
    Initialize the endpoints according to the DFU class driver. The class
    driver specifies a BULK IN, BULK OUT, and INTERRUPT IN endpoint. We will
    usually set this after the host issues the set_configuration request.
*/
/**************************************************************************/
void dfu_ep_init()
{
    // setup the endpoints

}

/**************************************************************************/
/*!
    This is the DFU's rx handler. You need to register your application's rx
    function here since the DFU doesn't know what to do with received data.
*/
/**************************************************************************/
void dfu_reg_rx_handler(void (*rx)())
{
    if (rx)
    {
        rx_handler = rx;
    }
}



/**************************************************************************/
/*!
    Initialize the DFU class driver. We basically register our init, request handler,
    and rx data handler with the USB core.
*/
/**************************************************************************/
void dfu_init()
{
    U32 *target_boot_address = (U32*)flash_target;

    if( hw_check_skip_bootloader() )
        hw_boot_image();

    // For software resets, extend the DFU countdown
    if( hw_check_extend_bootloader() )
        dfu_reset_counter = 30;

    // Otherwise prep for loading
    dfu_status.bStatus = OK;
    dfu_status.bwPollTimeout0 = 0x00;  
    dfu_status.bwPollTimeout1 = 0x00;  
    dfu_status.bwPollTimeout2 = 0x00;  
    dfu_status.bState = dfuIDLE;
    dfu_status.iString = 0x00;          /* all strings must be 0x00 until we make them! */

    if( ( *( volatile uint32_t* ) FLASH_TARGET ) != 0xFFFFFFFF )
    {
        hw_enable_watchdog();
    }

    usb_reg_class_drvr(dfu_ep_init, dfu_req_handler, dfu_rx_handler);
}

