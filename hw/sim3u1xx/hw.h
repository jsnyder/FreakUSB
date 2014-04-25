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

#if defined( USE_DFU_CLASS )

enum
{
    HW_STATE_COUNTDOWN = 0,
    HW_STATE_CONNECTED,
    HW_STATE_TRANSFER,
    HW_STATE_DONE,
    HW_STATE_ERROR,
    HW_STATE_ERROR_CLR
};

enum {
  LED_FADEUP,
  LED_FADEDOWN,
  LED_OFF,
  LED_ON,
  LED_FASTFLASH,
  LED_MEDIUMFLASH,
  LED_SLOWFLASH,
  LED_FLASH1,
  LED_FLASH2,
  LED_FLASH3,
  LED_FLASH4,
  LED_FLASH5
} enum_led_state;

#if defined ( MEMBRANE_V1 )
enum {
  LED_COLOR_GPS = 0, // was sat
  LED_COLOR_MSG = 1, // was pwr
  LED_COLOR_PWR = 2, // was alrm
  LED_COLOR_SAT = 3, // was gps
  LED_COLOR_ALRM = 4 // was msg
};
#else
enum {
  LED_COLOR_SAT = 0,
  LED_COLOR_PWR = 1,
  LED_COLOR_ALRM = 2,
  LED_COLOR_GPS = 3,
  LED_COLOR_MSG = 4
};
#endif

#define LED_CONTINUOUS 255
#define LED_REPEATS_FOREVER 255
#define LEDTICKHZ 1250
//NOTE! These must be sized by a factor of 2 to calculate properly
//First byte is size of the array
#define LED_COUNT 5
#define LED_MAX_ARRAY 32

#endif // USE_DFU_CLASS

// Device DFU/Flash Constants
#define FLASH_TARGET 0x3000
#define FLASH_PAGE_SIZE_U8    1024
#define FLASH_PAGE_SIZE_U32   (FLASH_PAGE_SIZE_U8/4)
#define TOTAL_FLASH_BLOCKS    (SI32_MCU_FLASH_SIZE / FLASH_PAGE_SIZE_U8)
#define BLOCK_CAPACITY        (TOTAL_FLASH_BLOCKS - DFU_SIZE)
#define BLOCK_SIZE_U8         FLASH_PAGE_SIZE_U8
#define BLOCK_SIZE_U32        FLASH_PAGE_SIZE_U32
#define DFU_START             (BLOCK_CAPACITY *  BLOCK_SIZE_U8)
#define VECTOR_TABLE_ADDRESS  (SI32_MCU_FLASH_SIZE - BLOCK_SIZE_U8)

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
void hw_disable_watchdog( void );
void hw_boot_image( int usb_started );
void hw_state_indicator( U32 state );
void hw_wait_ms(U32 delay_amount);
int hw_check_skip_bootloader( void );
int hw_check_extend_bootloader( void );
void hw_led_set_mode(int led, int mode, int cycles);
#endif
