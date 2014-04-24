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
    \defgroup dfu_demo CDC Demo
    \file main.c
    \ingroup dfu_demo
*/
/*******************************************************************/
#include <stdio.h>
#include "sim3u1xx.h"
#include "sim3u1xx_Types.h"
#include "main.h"
#include "freakusb.h"
#include "hw.h"
#include "dfu.h"

//static U8 msg[MAX_MSG_SIZE];
//static U8 *msg_ptr = msg;





/**************************************************************************/
/*!

*/
/**************************************************************************/


/**************************************************************************/
/*!
    This is the main function. Duh.
*/
/**************************************************************************/
int main()
{
    usb_init();
    hw_init();

    // init the class driver here
    dfu_init();

    // register the rx handler function with the dfu
    //dfu_reg_rx_handler(rx);

    // and off we go...
    while (1)
    {
        usb_poll();
        if( dfu_is_boot_pending() )
            hw_boot_image();
    }
}
