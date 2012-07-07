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
    \defgroup dfu_class USB DFU Class
    \file dfu.c
    \ingroup dfu_class
*/
/*******************************************************************/
#include "freakusb.h"
#include "dfu.h"

DFUStatus dfu_status;

// this is the rx handler callback function. it gets registered by the application program
// and will handle any incoming data.
static void (*rx_handler)();

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
            if( dfu_status.bState == appIDLE )
            {
                dfu_status.bState = appDETACH;
                dfu_status.bStatus = OK;
            }
        }
        break;

    case DFU_DNLOAD:
        if (req->type & (HOST_TO_DEVICE | TYPE_CLASS | RECIPIENT_INTF))
        {
            // wvalue is wBlockNum
            // wlength is Length
            // data is firmware

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
            for (i=0; i<STATUS_SZ; i++)
            {
                usb_buf_write(EP_CTRL, *((U8 *)&dfu_status + i));
            }
            ep_write(EP_CTRL);
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
        }
        break;

    case DFU_ABORT:
        if (req->type & (HOST_TO_DEVICE | TYPE_CLASS | RECIPIENT_INTF))
        {
            // wvalue is zero
            // wlength is 0
            // data is none
            dfu_status.bStatus = OK;
            dfu_status.bState = dfuIDLE;
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
    ep_config(EP_1, XFER_BULK, DIR_IN, MAX_PACKET_SZ);
    ep_config(EP_2, XFER_INTP, DIR_IN, MAX_PACKET_SZ);
    ep_config(EP_3, XFER_BULK, DIR_OUT, MAX_PACKET_SZ);
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
    This is the putchar function that is used by avr-libc's printf. We need
    to hook this function into the stdout file stream using the FDEV_SETUP_STREAM
    macro in avr-libc. Once the stream is set up, we hook the stream to stdout
    and we can do printfs via USB.
*/
/**************************************************************************/
int dfu_demo_putchar(char c, FILE *unused)
{
    usb_pcb_t *pcb = usb_pcb_get();
    
    if (!(pcb->flags & (1<<ENUMERATED)))
    {
        return 0;
    }

    if (c == '\n')
    {
        usb_buf_write(EP_1, '\n');
        usb_buf_write(EP_1, '\r');
    }
    else
    {
      usb_buf_write(EP_1, (U8)c);
    }
    ep_write(EP_1);
    return 0;
}

/**************************************************************************/
/*!
    Initialize the DFU class driver. We basically register our init, request handler,
    and rx data handler with the USB core.
*/
/**************************************************************************/
void dfu_init()
{
    // hook the putchar function into the printf stdout filestream. This is needed
    // for printf to work.
    //stdout = &file_str;

    usb_reg_class_drvr(dfu_ep_init, dfu_req_handler, dfu_rx_handler);
}

