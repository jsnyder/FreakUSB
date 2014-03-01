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
    \file hw.h
    \ingroup hw_sim3u1xx
*/
/*******************************************************************/
#ifndef HW_H
#define HW_H

enum
{
    OPRDYI = 0,
    IPRDYI,
    SUSI,
    RESI,
    RSTI,
    SOFI
};

// Flash writing target
#define FLASH_TARGET 0x3000

// Watchdog timer
#define EARLY_WARNING_DELAY_MS        1000   // Will result in approx a 1 s
                                             // periodic early warning interrupt
#define RESET_DELAY_MS                2000  // Will result in approx a 2 s
                                            // reset delay (if early warning isn't captured)
#define EARLY_WARNING_THRESHOLD       (uint32_t)((16400*EARLY_WARNING_DELAY_MS)/1000)
#define RESET_THRESHOLD               (uint32_t)((16400*RESET_DELAY_MS)/1000)


#define PROGMEM 

#define PSTR(a) (a)
#define printf_P(a) fputs(a, stdout)

void hw_init();
void hw_intp_disable();
void hw_intp_enable();
U8 hw_flash_get_byte(U8 *addr);
U8 hw_flash_erase( U32 address, U8 verify);
U8 hw_flash_write( U32 address, U32* data, U32 count, U8 verify );
void hw_enable_watchdog( void );
void hw_boot_image( void );
void hw_activity_indicator( void );
void hw_wait_ms(U32 delay_amount);

#endif
