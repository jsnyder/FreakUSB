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
    \file dfu.h
    \ingroup dfu_class
*/
/*******************************************************************/
#ifndef DFU_H
#define DFU_H
#include "types.h"

typedef struct _DFUStatus {
  U8 bStatus;
  U8 bwPollTimeout0;
  U8 bwPollTimeout1;
  U8 bwPollTimeout2;
  U8 bState;  /* state of device at the time the host receives the message! */
  U8 iString;
} DFUStatus;

#define NUM_EPS             1
#define STATUS_SZ           6

// Device DFU/Flash Constants
#define DFU_SIZE              16 // In 1k increments for flash pages, also called dfu blocks
#define FLASH_PAGE_SIZE_U8    1024
#define FLASH_PAGE_SIZE_U32   (FLASH_PAGE_SIZE_U8/4)
#define TOTAL_FLASH_BLOCKS    (SI32_MCU_FLASH_SIZE / FLASH_PAGE_SIZE_U8)
#define BLOCK_CAPACITY        (TOTAL_FLASH_BLOCKS - DFU_SIZE)
#define BLOCK_SIZE_U8         FLASH_PAGE_SIZE_U8
#define BLOCK_SIZE_U32        FLASH_PAGE_SIZE_U32
#define DFU_START             (BLOCK_CAPACITY *  BLOCK_SIZE_U8)
#define VECTOR_TABLE_ADDRESS  (SI32_MCU_FLASH_SIZE - BLOCK_SIZE_U8)
#define BOOT_MANAGER_ADDRESS  (DFU_START + 1) // Need to add 1 to address to indicate this is thumb code
#define DFU_SIGNATURE_ADDRESS (DFU_START - 4) // Always 4 bytes below start of dfu
#define VALID_DFU_SIGNATURE   0x44465531      // Signature word is 'D', 'F', 'U', '1' after succesful download

// DFU Request Definitions
                            // bmRequestType, wValue,    wIndex,    wLength, Data
#define  DFU_DETACH    0x00 // 0x21,          wTimeout,  Interface, Zero,    None
#define  DFU_DNLOAD    0x01 // 0x21,          wBlockNum, Interface, Length,  Firmware
#define  DFU_UPLOAD    0x02 // 0xA1,          Zero,      Interface, Length,  Firmware
#define  DFU_GETSTATUS 0x03 // 0xA1,          Zero,      Interface, 6,       Status
#define  DFU_CLRSTATUS 0x04 // 0x21,          Zero,      Interface, Zero,    None
#define  DFU_GETSTATE  0x05 // 0xA1,          Zero,      Interface, 1,       State
#define  DFU_ABORT     0x06 // 0x21,          Zero,      Interface, Zero,    None

// DFU Status Values
#define  OK              0x00 // No error
#define  errTARGET       0x01 // File is not appropriate for this device
#define  errFILE         0x02 // File fails some vendor tests
#define  errWRITE        0x03 // Device is unable to write memory
#define  errERASE        0x04 // Memory erase failed
#define  errCHECK_ERASED 0x05 // Memory erase check failed
#define  errPROG         0x06 // Program memory function failed
#define  errVERIFY       0x07 // Written program failed verification
#define  errADDRESS      0x08 // address out of range
#define  errNOTDONE      0x09 // received DNLOAD with wLength=0, but firmware seems incomplete
#define  errFIRMWARE     0x0A // Runtime firmware corrupt, cannot return to non-dfu operations!
#define  errVENDOR       0x0B // vendor specific error
#define  errUSBR         0x0C // Unexpected usb reset!
#define  errPOR          0x0D // Unexpected power on reset
#define  errUNKNOWN      0x0E // Unknown error
#define  errSTALLEDPKT   0x0F // device stalled unexpected request


// DFU State Values
#define  appIDLE                0x00
#define  appDETACH              0x01
#define  dfuIDLE                0x02
#define  dfuDNLOAD_SYNC         0x03
#define  dfuDNBUSY              0x04
#define  dfuDNLOAD_IDLE         0x05
#define  dfuMANIFEST_SYNC       0x06
#define  dfuMANIFEST            0x07
#define  dfuMANIFEST_WAIT_RESET 0x08
#define  dfuUPLOAD_IDLE         0x09
#define  dfuERROR               0x0A



void dfu_init();
void dfu_req_handler();
void dfu_rx_handler();
void dfu_reg_rx_handler(void (*rx)());
void boot_image( void );

#endif // DFU_H

