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
#include "sim3u1xx.h"
#include "sim3u1xx_Types.h"


// Watchdog timer
#define EARLY_WARNING_DELAY_MS        1000   // Will result in approx a 1 s
                                             // periodic early warning interrupt

volatile U16 dfu_reset_counter = 10;         // 10 warnings are allowed

#define RESET_DELAY_MS                2000  // Will result in approx a 2 s
                                            // reset delay (if early warning isn't captured)

#define EARLY_WARNING_THRESHOLD       (uint32_t)((16400*EARLY_WARNING_DELAY_MS)/1000)
                                      
#define RESET_THRESHOLD               (uint32_t)((16400*RESET_DELAY_MS)/1000)



DFUStatus dfu_status;

// this is the rx handler callback function. it gets registered by the application program
// and will handle any incoming data.
static void (*rx_handler)();

uint32_t flash_buffer[BLOCK_SIZE_U32];
uint32_t* flash_buffer_ptr = flash_buffer;
uint32_t flash_target = 0x3000;


volatile U8 flash_key_mask  = 0x00;
volatile U8 armed_flash_key = 0x00;
volatile U8 need_to_write = 0;
volatile U8 dfu_communication_started = 0;

void boot_image( void )
{
    volatile uint32_t jumpaddr;
    void (*app_fn)(void) = NULL;

    // prepare jump address
    jumpaddr = *(volatile uint32_t*) (0x3000 + 4);
    // prepare jumping function
    app_fn = (void (*)(void)) jumpaddr;

    NVIC_DisableIRQ( USB0_IRQn );

    SCB->VTOR = 0x3000;

    // initialize user application's stack pointer
    __set_MSP(*(volatile uint32_t*) 0x3000);

    // jump.
    app_fn();
}


U8 flash_erase( U32 address, U8 verify)
{
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

        for( U32 wc = FLASH_PAGE_SIZE_U32; wc != 0; wc-- )
        {
            if ( *verify_address != 0xFFFFFFFF )
                return 1;

            verify_address++;
        }
    }

    hw_intp_enable();

    return 0;
}


U8 flash_write( U32 address, U32* data, U32 count, U8 verify )
{
    U32* tmpdata = data;

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
    for( U32 wc = count; wc != 0; wc-- )
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

        for( U32 wc = count; wc != 0; wc-- )
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

void wait_ms_using_usb(U32 delay_amount)
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
            flash_key_mask = 0x01;



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
                    SI32_USB_A_clear_out_packet_ready_ep0(SI32_USB_0);
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
                        //flash_buffer_ptr = flash_buffer;
                    }
                    dfu_status.bState  = dfuMANIFEST_SYNC;
                    SI32_USB_A_clear_out_packet_ready_ep0(SI32_USB_0);
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

            dfu_communication_started = 1;
            // If we're still transmitting blocks
            if( dfu_status.bState == dfuDNLOAD_SYNC )
            {
                if( need_to_write == 0 )
                    dfu_status.bState=dfuDNLOAD_IDLE;
                else
                {
                	dfu_status.bState=dfuDNBUSY;
                }
            }
            else if( dfu_status.bState == dfuDNBUSY )
            {
                if( need_to_write == 0)
                    dfu_status.bState=dfuDNLOAD_SYNC;
            }
            else if( dfu_status.bState == dfuMANIFEST_SYNC)
            	dfu_status.bState=dfuMANIFEST;
            else if( dfu_status.bState == dfuMANIFEST &&
                     need_to_write == 0)
                dfu_status.bState=dfuMANIFEST_WAIT_RESET;

            for (i=0; i<STATUS_SZ; i++)
            {
                usb_buf_write(EP_CTRL, *((U8 *)&dfu_status + i));
            }
            ep_write(EP_CTRL);

            if( dfu_status.bState == dfuMANIFEST_WAIT_RESET )
            {
                wait_ms_using_usb(200);
                SI32_USB_A_disable_internal_pull_up( SI32_USB_0 );
                for (U32 down_count = 0x1FFFFFF; down_count != 0; down_count--);
                boot_image();
            }

            if( need_to_write )
            {
                flash_key_mask = 0x01;
                if( 0 != flash_erase( flash_target, 1 ) )
                {
                    dfu_status.bState  = dfuERROR;
                    dfu_status.bStatus = errERASE;
                }
                flash_key_mask = 0x01;
                if( 0 != flash_write( flash_target, ( U32* )flash_buffer, flash_buffer_ptr - flash_buffer, 1 ) )
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
    // If the watchdog, software, pmu or RTC rest us, boot the image
    if ((SI32_RSTSRC_A_get_last_reset_source(SI32_RSTSRC_0) == SI32_WDT_RESET) ||
        (SI32_RSTSRC_A_get_last_reset_source(SI32_RSTSRC_0) == SI32_PMU_WAKEUP_RESET) ||
        (SI32_RSTSRC_A_get_last_reset_source(SI32_RSTSRC_0) == SI32_RTC0_RESET) ||
        (SI32_RSTSRC_A_get_last_reset_source(SI32_RSTSRC_0) == SI32_CMP0_RESET))
    {
        if ((SI32_RSTSRC_A_get_last_reset_source(SI32_RSTSRC_0) != SI32_POWER_ON_RESET)
            || (SI32_RSTSRC_A_get_last_reset_source(SI32_RSTSRC_0) != SI32_VDD_MON_RESET))
        {
            // SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_2, 0x000000400);
            // SI32_PBCFG_A_enable_crossbar_1(SI32_PBCFG_0);
            // SI32_PBSTD_A_write_pins_low (SI32_PBSTD_2, 0x000000400);

            boot_image();
        }
    }

    // Boot if 3.8 is low
    if( ( SI32_PBSTD_A_read_pins( SI32_PBSTD_3 ) & ( 1 << 8 ) ) == 0 )
        boot_image();

    // For software resets, extend the DFU countdown
    if( SI32_RSTSRC_A_get_last_reset_source(SI32_RSTSRC_0) == SI32_SW_RESET )
    {
        if ((SI32_RSTSRC_A_get_last_reset_source(SI32_RSTSRC_0) != SI32_POWER_ON_RESET)
            || (SI32_RSTSRC_A_get_last_reset_source(SI32_RSTSRC_0) != SI32_VDD_MON_RESET))
        {
            dfu_reset_counter = 300;
        }
    }

    // Otherwise prep for loading
    dfu_status.bStatus = OK;
    dfu_status.bwPollTimeout0 = 0xFF;  
    dfu_status.bwPollTimeout1 = 0x00;  
    dfu_status.bwPollTimeout2 = 0x00;  
    dfu_status.bState = dfuIDLE;
    dfu_status.iString = 0x00;          /* all strings must be 0x00 until we make them! */

    // Enable Watchdog Timer
    // ENABLE CLOCK
    SI32_CLKCTRL_A_enable_apb_to_modules_1(SI32_CLKCTRL_0,
                                           SI32_CLKCTRL_A_APBCLKG1_MISC1CEN_ENABLED_U32);


    if( *target_boot_address != 0xFFFFFFFF )
    {
        // Setup Watchdog Timer
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

    usb_reg_class_drvr(dfu_ep_init, dfu_req_handler, dfu_rx_handler);


}

