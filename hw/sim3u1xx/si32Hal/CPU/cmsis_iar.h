/**************************************************************************//**
 * @file     cmsis_iar.h
 * @brief    CMSIS Cortex-M Core Instruction Access Header File for IAR
 * @version  V2.01
 * @date     06. December 2010
 *
 * @note
 * Copyright (C) 2009-2010 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M 
 * processor based microcontrollers.  This file can be freely distributed 
 * within development tools that are supporting such ARM based processors. 
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/

#ifndef __CMSIS_IAR_H__
#define __CMSIS_IAR_H__

/* ##########################  Core Instruction Access  ######################### */
/** \defgroup CMSIS_Core_InstructionInterface CMSIS Core Instruction Interface
  Access to dedicated instructions
  @{
*/

#include <intrinsics.h>                     /* IAR Intrinsics   */

#pragma diag_suppress=Pe940

/** \brief  No Operation

    No Operation does nothing. This instruction can be used for code alignment purposes.
 */
#define __NOP                           __no_operation


/** \brief  Wait For Interrupt

    Wait For Interrupt is a hint instruction that suspends execution
    until one of a number of events occurs.
 */
static __INLINE  void __WFI(void)
{
  __ASM ("wfi");
}


/** \brief  Wait For Event

    Wait For Event is a hint instruction that permits the processor to enter
    a low-power state until one of a number of events occurs.
 */
static __INLINE  void __WFE(void)
{
  __ASM ("wfe");
}


/** \brief  Send Event

    Send Event is a hint instruction. It causes an event to be signaled to the CPU.
 */
static __INLINE  void __SEV(void)
{
  __ASM ("sev");
}


/* intrinsic     void __ISB(void)            (see intrinsics.h) */
/* intrinsic     void __DSB(void)            (see intrinsics.h) */
/* intrinsic     void __DMB(void)            (see intrinsics.h) */
/* intrinsic uint32_t __REV(uint32_t value)  (see intrinsics.h) */
/* intrinsic          __SSAT                 (see intrinsics.h) */
/* intrinsic          __USAT                 (see intrinsics.h) */


/** \brief  Reverse byte order (16 bit)

    This function reverses the byte order in two unsigned short values.

    \param [in]    value  Value to reverse
    \return               Reversed value
 */
static uint32_t __REV16(uint32_t value)
{
  __ASM("rev16 r0, r0");
}


/* intrinsic uint32_t __REVSH(uint32_t value)  (see intrinsics.h */


#if       (__CORTEX_M >= 0x03)

/** \brief  Reverse bit order of value

    This function reverses the bit order of the given value.

    \param [in]    value  Value to reverse
    \return               Reversed value
 */
static uint32_t __RBIT(uint32_t value)
{
  __ASM("rbit r0, r0");
}


/** \brief  LDR Exclusive (8 bit)

    This function performs a exclusive LDR command for 8 bit value.

    \param [in]    ptr  Pointer to data
    \return             value of type uint8_t at (*ptr)
 */
static uint8_t __LDREXB(volatile uint8_t *addr)
{
  __ASM("ldrexb r0, [r0]");
}


/** \brief  LDR Exclusive (16 bit)

    This function performs a exclusive LDR command for 16 bit values.

    \param [in]    ptr  Pointer to data
    \return        value of type uint16_t at (*ptr)
 */
static uint16_t __LDREXH(volatile uint16_t *addr)
{
  __ASM("ldrexh r0, [r0]");
}


/** \brief  LDR Exclusive (32 bit)

    This function performs a exclusive LDR command for 32 bit values.

    \param [in]    ptr  Pointer to data
    \return        value of type uint32_t at (*ptr)
 */
/* intrinsic unsigned long __LDREX(unsigned long *)  (see intrinsics.h) */
static uint32_t __LDREXW(volatile uint32_t *addr)
{
  __ASM("ldrex r0, [r0]");
}


/** \brief  STR Exclusive (8 bit)

    This function performs a exclusive STR command for 8 bit values.

    \param [in]  value  Value to store
    \param [in]    ptr  Pointer to location
    \return          0  Function succeeded
    \return          1  Function failed
 */
static uint32_t __STREXB(uint8_t value, volatile uint8_t *addr)
{
  __ASM("strexb r0, r0, [r1]");
}


/** \brief  STR Exclusive (16 bit)

    This function performs a exclusive STR command for 16 bit values.

    \param [in]  value  Value to store
    \param [in]    ptr  Pointer to location
    \return          0  Function succeeded
    \return          1  Function failed
 */
static uint32_t __STREXH(uint16_t value, volatile uint16_t *addr)
{
  __ASM("strexh r0, r0, [r1]");
}


/** \brief  STR Exclusive (32 bit)

    This function performs a exclusive STR command for 32 bit values.

    \param [in]  value  Value to store
    \param [in]    ptr  Pointer to location
    \return          0  Function succeeded
    \return          1  Function failed
 */
/* intrinsic unsigned long __STREX(unsigned long, unsigned long)  (see intrinsics.h )*/
static uint32_t __STREXW(uint32_t value, volatile uint32_t *addr)
{
  __ASM("strex r0, r0, [r1]");
}


/** \brief  Remove the exclusive lock

    This function removes the exclusive lock which is created by LDREX.

 */
static __INLINE void __CLREX(void)
{
  __ASM ("clrex");
}

/* intrinsic   unsigned char __CLZ( unsigned long )      (see intrinsics.h) */

#endif /* (__CORTEX_M >= 0x03) */

#pragma diag_default=Pe940

/*@}*/ /* end of group CMSIS_Core_InstructionInterface */

#endif /* __CMSIS_IAR_H__ */
