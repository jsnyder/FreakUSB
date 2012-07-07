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
    \file desc.c
    \ingroup dfu_class
*/
/*******************************************************************/
#include "freakusb.h"
#include "hw.h"

#define wTransferSize 1024

U8 dev_desc[] PROGMEM =
{
    0x12,       // bLength
    DEV_DESCR,  // bDescriptorType
    0x00,
    0x02,       // bcdUSB: 0200 (2.0)
    0x00,       // bDeviceClass: Get from cfg descr
    0x00,       // bDeviceSubClass: Get from cfg descr
    0x00,       // bDeviceProtocol: Get from cfg descr
    MAX_BUF_SZ, // bMaxPacketSize: MAX_BUF_SZ
    0xEB,
    0x03,       // idVendor: 0x03EB (use Atmel's VID for now)
    0x18,
    0x20,       // idProduct: 0x2018 (use Atmel's PID for now)
    0x00,
    0x01,       // bcdDevice: 0100 (v1.00)
    0x01,          // Index of string descriptor describing manufacturer
    0x02,          // Index of string descriptor describing product
    0x03,          // Index of string descriptor describing the device's serial number
    0x01        // bNumConfigurations
};

// cfg descriptor (updated for DFU)
U8 cfg_desc[27] PROGMEM =
{
    // cfg descr
    0x09,           // bLength: cfg desc len
    CFG_DESCR,      // bDescriptorType: Configuration
    0x1B,           // wTotalLength: 0x24 (36 bytes = total size of cfg)
    0x00,
    0x01,           // bNumInterfaces: 1 interface
    0x01,           // bConfigurationValue: Configuration value
    0x00,           // iConfiguration: Index of string descriptor describing the configuration
    0x80,           // bmAttributes: ??
    0x32,           // MaxPower 100 mA

    // intf descr alternate 0
    0x09,           // bLength: Interface Descriptor size
    INTF_DESCR,     // bDescriptorType: Interface
    0x00,           // bInterfaceNumber: Number of Interface
    0x00,           // bAlternateSetting: Alternate setting
    0x00,           // bNumEndpoints: zero endpoints
    0xFE,           // bInterfaceClass: Device Firmware Upgrade
    0x01,           // bInterfaceSubClass: ???
    0x02,           // bInterfaceProtocol:  switched to 0x02 while in dfu_mode
    0x04,           // iInterface:

    0x09,           // blength = 7 Bytes
    0x21,           // DFU Functional Descriptor
    0x01,           // bmAttribute, can only download for now 
    0xFF,           // DetachTimeOut= 255 ms
    0xFF,
    (wTransferSize & 0x00FF),
    (wTransferSize & 0xFF00) >> 8,  // TransferSize = 1024 Byte
    0x01,                           // bcdDFUVersion
    0x00
};

U8 dfu_functional_desc[] PROGMEM =
{
    0x09,           // bLength
    0x21,           // bDescriptorType: DFU Functional Descriptor
    0x01,
    0xFF,           // DetachTimeOut: 255 ms
    0xFF,           // DetachTimeOut: 255 ms
    (wTransferSize & 0x00FF),
    (wTransferSize & 0xFF00) >> 8, /* TransferSize = 1024 Byte*/
    0x1A,                          /* bcdDFUVersion*/
    0x01
};

U8 lang_str_desc[] PROGMEM =
{
    0x4,        // bLength
    STR_DESCR,  // bDescriptorType: String
                // Language: English
    0x09, 0x04
};

U8 vendor_str_desc[] PROGMEM =
{
    0x14,       // bLength
    STR_DESCR,  // bDescriptorType: String
                // Manufacturer: "FreakLabs"
    'F',0, 'r',0, 'e',0, 'a',0, 'k',0, 'L',0, 'a',0, 'b',0,
    's',0
};

U8 prod_str_desc[] PROGMEM =
{
    0x32,      // bLength: 0x32 (50 bytes = sizeof str fields + string)
    STR_DESCR, // bDescriptorType: String
               // Product name: "FreakLabs RP AVRUSB FLASH"

    'F',0, 'r',0, 'e',0, 'a',0, 'k',0, 'L',0, 'a',0, 'b',0,
    's',0, ' ',0, 'R',0, 'P',0, ' ',0, 'A',0, 'V',0, 'R',0,
    'U',0, 'S',0, 'B',0, ' ',0, 'F',0, 'L',0, 'A',0, 'S',0,
    'H',0,
};

U8 serial_str_desc[] PROGMEM =
{
    0x14,       // bLength: 0x14 (20 bytes = sizeof str fields + string)
    STR_DESCR,  // bDescriptorType: String
                // Serial: Beta 0.50
    'B',0, 'e',0, 't',0, 'a',0, ' ',0, '0',0, '.',0, '5',0,
    '0',0,
};

/**************************************************************************/
/*!
    Return a pointer to the device descriptor.
*/
/**************************************************************************/
U8 *desc_dev_get()
{
    return dev_desc;
}

/**************************************************************************/
/*!
    Return the length of the device descriptor. The length is stored in the
    first byte of the device descriptor.
*/
/**************************************************************************/
U8 desc_dev_get_len()
{
    return hw_flash_get_byte(dev_desc);
}

/**************************************************************************/
/*!
    Return a pointer to the configuration descriptor.
*/
/**************************************************************************/
U8 *desc_cfg_get()
{
    return cfg_desc;
}

/**************************************************************************/
/*!
    Return the length of the configuration descriptor. The length of the complete
    configuration descriptor is stored in the third byte of the config
    descriptor.
*/
/**************************************************************************/
U8 desc_cfg_get_len()
{
    return hw_flash_get_byte(cfg_desc + 2);
}

/**************************************************************************/
/*!
    Return a pointer to the device qualifier.
*/
/**************************************************************************/
U8 *desc_dfu_func_get()
{
    return dfu_functional_desc;
}

/**************************************************************************/
/*!
    Return the length of the device qualifier. The length is stored in the
    first byte of the qualifier.
*/
/**************************************************************************/
U8 desc_dfu_func_get_len()
{
    return hw_flash_get_byte(dfu_functional_desc);
}

/**************************************************************************/
/*!
    Return a pointer to the device qualifier.
*/
/**************************************************************************/
U8 *desc_dev_qual_get()
{
    return NULL;
}

/**************************************************************************/
/*!
    Return the length of the device qualifier. The length is stored in the
    first byte of the qualifier.
*/
/**************************************************************************/
U8 desc_dev_qual_get_len()
{
    return 0;
}

/**************************************************************************/
/*!
    Return a pointer to the specified string descriptor.
*/
/**************************************************************************/
U8 *desc_str_get(U8 index)
{
    switch (index)
    {
    case LANG_DESC_IDX:
        return (U8 *)lang_str_desc;
    case MANUF_DESC_IDX:
        return (U8 *)vendor_str_desc;
    case PROD_DESC_IDX:
        return (U8 *)prod_str_desc;
    case SERIAL_DESC_IDX:
        return (U8 *)serial_str_desc;
    default:
        return NULL;
    }
}

/**************************************************************************/
/*!
    Return the length of the specified string descriptor.
*/
/**************************************************************************/
U8 desc_str_get_len(U8 index)
{
    switch (index)
    {
    case LANG_DESC_IDX:
        return hw_flash_get_byte(lang_str_desc);
    case MANUF_DESC_IDX:
        return hw_flash_get_byte(vendor_str_desc);
    case PROD_DESC_IDX:
        return hw_flash_get_byte(prod_str_desc);
    case SERIAL_DESC_IDX:
        return hw_flash_get_byte(serial_str_desc);
    default:
        return 0;
    }
}
