//------------------------------------------------------------------------------
// Copyright (c) 2012 by Silicon Laboratories. 
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the Silicon Laboratories End User 
// License Agreement which accompanies this distribution, and is available at
// http://developer.silabs.com/legal/version/v10/License_Agreement_v10.htm
// Original content and implementation provided by Silicon Laboratories.
//------------------------------------------------------------------------------

// Version: 1

#ifndef __SI32_CAPSENSE_A_SUPPORT_H__
#define __SI32_CAPSENSE_A_SUPPORT_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//-----------------------------------------------------------------------------
// Define Conversion Rate Enum type

typedef enum SI32_CAPSENSE_A_CONVERSION_RATE_Enum
{
   SI32_CAPSENSE_A_CONVERSION_RATE_12_CLOCKS_12_BITS = 0,
   SI32_CAPSENSE_A_CONVERSION_RATE_13_CLOCKS_13_BITS = 1,
   SI32_CAPSENSE_A_CONVERSION_RATE_14_CLOCKS_14_BITS = 2,
   SI32_CAPSENSE_A_CONVERSION_RATE_16_CLOCKS_16_BITS = 3
} SI32_CAPSENSE_A_CONVERSION_RATE_Enum_Type;

//-----------------------------------------------------------------------------
// Define Accumulator Mode Enum type

typedef enum SI32_CAPSENSE_A_ACCUMULATOR_MODE_Enum
{
   SI32_CAPSENSE_A_ACCUMULATOR_MODE_1_SAMPLE   = 0,
   SI32_CAPSENSE_A_ACCUMULATOR_MODE_4_SAMPLES  = 1,
   SI32_CAPSENSE_A_ACCUMULATOR_MODE_8_SAMPLES  = 2,
   SI32_CAPSENSE_A_ACCUMULATOR_MODE_16_SAMPLES = 3,
   SI32_CAPSENSE_A_ACCUMULATOR_MODE_32_SAMPLES = 4,
   SI32_CAPSENSE_A_ACCUMULATOR_MODE_64_SAMPLES = 5
} SI32_CAPSENSE_A_ACCUMULATOR_MODE_Enum_Type;

//-----------------------------------------------------------------------------
// Define Pin Monitor Mode Enum type

typedef enum SI32_CAPSENSE_A_PIN_MONITOR_MODE_Enum
{
   SI32_CAPSENSE_A_PIN_MONITOR_MODE_RETRY_ON_PIN_STATE_CHANGE = 0,
   SI32_CAPSENSE_A_PIN_MONITOR_MODE_RETRY_UP_TO_TWICE         = 1,
   SI32_CAPSENSE_A_PIN_MONITOR_MODE_RETRY_UP_TO_4_TIMES       = 2,
   SI32_CAPSENSE_A_PIN_MONITOR_MODE_IGNORE_STATE_CHANGE       = 3
} SI32_CAPSENSE_A_PIN_MONITOR_MODE_Enum_Type;

#ifdef __cplusplus
}
#endif

#endif // __SI32_CAPSENSE_A_SUPPORT_H__

//-eof--------------------------------------------------------------------------
