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
    \defgroup cdc_demo CDC Demo
    \file main.c
    \ingroup cdc_demo
*/
/*******************************************************************/
#include <stdio.h>
#include "sim3u1xx.h"
#include "sim3u1xx_Types.h"
#include "main.h"
#include "freakusb.h"
#include "hw.h"

static U8 msg[MAX_MSG_SIZE];
static U8 *msg_ptr = msg;

static cmd_t cmd_tbl[] = 
{
    {NULL,      NULL}
};

/**************************************************************************/
/*!
    This is the rx handling function. It will handle any incoming data from
    the USB. This function needs to be registered with the CDC class since
    the CDC doesn't know what to do with received data.
*/
/**************************************************************************/
void rx()
{
    U8 i, c, ep_num, len;
    usb_pcb_t *pcb = usb_pcb_get();

    // get the ep number of any endpoint with pending rx data
    if ((ep_num = usb_buf_data_pending(DIR_OUT)) != 0xFF)
    {
        // get the length of data in the OUT buffer
        len = pcb->fifo[ep_num].len;

        // read out the data in the buffer and echo it back to the host. 
        for (i=0; i<len; i++)
        {
            c = usb_buf_read(ep_num);

            switch (c)
            {
            case '\r':
                // terminate the msg and reset the msg ptr. then send
                // it to the handler for processing.
                *msg_ptr = '\0';
                printf_P(PSTR("\n\r"));
                cmd_parse((char *)msg);
                msg_ptr = msg;
                break;
            
            case '\b':
                usb_buf_write(EP_1, c);
                if (msg_ptr > msg)
                {
                    msg_ptr--;
                }
                break;
            
            default:
                usb_buf_write(EP_1, c);
                *msg_ptr++ = c;
                break;
            }
        }
        pcb->flags |= (1 << TX_DATA_AVAIL);
    }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
static void cmd_menu()
{
    fputs( "\n", stdout );
    fputs( "********** FreakUSB **************\n", stdout );
    fputs( "FREAKUSB >> ", stdout );
    fflush( stdout );
}


/**************************************************************************/
/*!

*/
/**************************************************************************/
void cmd_parse(char *cmd)
{
    U8 argc, i = 0;
    char *argv[30];

    fflush(stdout);

    argv[i] = strtok(cmd, " ");
    do
    {
        argv[++i] = strtok(NULL, " ");
    } while ((i < 30) && (argv[i] != NULL));
    
    argc = i;
    for (i=0; cmd_tbl[i].cmd != NULL; i++)
    {
        if (!strcmp(argv[0], cmd_tbl[i].cmd))
        {
            cmd_tbl[i].func(argc, argv);
            cmd_menu();
            return;
        }
    }
    printf_P("CMD: Command not recognized: ");
    printf_P(cmd);
    printf_P("\n");

    cmd_menu();
}

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
    cdc_init();

    // register the rx handler function with the cdc
    cdc_reg_rx_handler(rx);

    // and off we go...
    while (1)
    {
        usb_poll();
    }
}
