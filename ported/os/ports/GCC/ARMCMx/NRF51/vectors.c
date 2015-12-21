/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/

/**
 * @file    GCC/ARMCMx/NRF51/vectors.c
 * @brief   Interrupt vectors for the NRF51 family.
 *
 * @defgroup ARMCMx_NRF51_VECTORS NRF51 Interrupt Vectors
 * @ingroup ARMCMx_SPECIFIC
 * @details Interrupt vectors for the NRF51 family.
 * @{
 */

#include "ch.h"
#include "hal.h"

/**
 * @brief   Type of an IRQ vector.
 */
typedef void  (*irq_vector_t)(void);

/**
 * @brief   Type of a structure representing the whole vectors table.
 */
typedef struct {
  uint32_t      *init_stack;
  irq_vector_t  reset_vector;
  irq_vector_t  nmi_vector;
  irq_vector_t  hardfault_vector;
  irq_vector_t  memmanage_vector;
  irq_vector_t  busfault_vector;
  irq_vector_t  usagefault_vector;
  irq_vector_t  vector1c;
  irq_vector_t  vector20;
  irq_vector_t  vector24;
  irq_vector_t  vector28;
  irq_vector_t  svcall_vector;
  irq_vector_t  debugmonitor_vector;
  irq_vector_t  vector34;
  irq_vector_t  pendsv_vector;
  irq_vector_t  vector3c;
  irq_vector_t  vectors[32];
} vectors_t;

#if !defined(__DOXYGEN__)
extern uint32_t __main_stack_end__;       //   __cs3_stack
extern void ResetHandler(void);           //   __cs3_reset
extern void NMIVector(void);              //   NMI_Handler
extern void HardFaultVector(void);        //   HardFault_Handler
extern void MemManageVector(void);        //   0
extern void BusFaultVector(void);         //   0
extern void UsageFaultVector(void);       //   0
extern void Vector1C(void);               //   0
extern void Vector20(void);               //   0
extern void Vector24(void);               //   0
extern void Vector28(void);               //   0
extern void SVCallVector(void);           //   SVC_Handler
extern void DebugMonitorVector(void);     //   0
extern void Vector34(void);               //   0
extern void PendSVVector(void);           //   PendSV_Handler
extern void Vector3C(void);               //   SysTick_Handler
extern void Vector40(void);               //   POWER_CLOCK_IRQHandler
extern void Vector44(void);               //   RADIO_IRQHandler
extern void Vector48(void);               //   UART0_IRQHandler
extern void Vector4C(void);               //   SPI0_TWI0_IRQHandler
extern void Vector50(void);               //   SPI1_TWI1_IRQHandler
extern void Vector54(void);               //   0
extern void Vector58(void);               //   GPIOTE_IRQHandler
extern void Vector5C(void);               //   ADC_IRQHandler
extern void Vector60(void);               //   TIMER0_IRQHandler
extern void Vector64(void);               //   TIMER1_IRQHandler
extern void Vector68(void);               //   TIMER2_IRQHandler
extern void Vector6C(void);               //   RTC0_IRQHandler
extern void Vector70(void);               //   TEMP_IRQHandler
extern void Vector74(void);               //   RNG_IRQHandler
extern void Vector78(void);               //   ECB_IRQHandler
extern void Vector7C(void);               //   CCM_AAR_IRQHandler
extern void Vector80(void);               //   WDT_IRQHandler
extern void SysTickVector(void);          //   RTC1_IRQHandler
extern void Vector88(void);               //   QDEC_IRQHandler
extern void Vector8C(void);               //   LPCOMP_IRQHandler
extern void Vector90(void);               //   SWI0_IRQHandler
extern void Vector94(void);               //   SWI1_IRQHandler
extern void Vector98(void);               //   SWI2_IRQHandler
extern void Vector9C(void);               //   SWI3_IRQHandler
extern void VectorA0(void);               //   SWI4_IRQHandler
extern void VectorA4(void);               //   SWI5_IRQHandler
extern void VectorA8(void);               //   0
extern void VectorAC(void);               //   0
extern void VectorB0(void);               //   0
extern void VectorB4(void);               //   0
extern void VectorB8(void);               //   0
extern void VectorBC(void);               //   0


#endif

/**
 * @brief   NRF51 vectors table.
 */
#if !defined(__DOXYGEN__)
__attribute__ ((section("vectors")))
#endif
vectors_t _vectors = {
  &__main_stack_end__,ResetHandler,       NMIVector,          HardFaultVector,
  MemManageVector,    BusFaultVector,     UsageFaultVector,   Vector1C,
  Vector20,           Vector24,           Vector28,           SVCallVector,
  DebugMonitorVector, Vector34,           PendSVVector,       Vector3C,
  {
    Vector40,           Vector44,           Vector48,           Vector4C,
    Vector50,           Vector54,           Vector58,           Vector5C,
    Vector60,           Vector64,           Vector68,           Vector6C,
    Vector70,           Vector74,           Vector78,           Vector7C,
    Vector80,           SysTickVector,      Vector88,           Vector8C,
    Vector90,           Vector94,           Vector98,           Vector9C,
    VectorA0,           VectorA4,           VectorA8,           VectorAC,
    VectorB0,           VectorB4,           VectorB8,           VectorBC
  }
};

/**
 * @brief   Unhandled exceptions handler.
 * @details Any undefined exception vector points to this function by default.
 *          This function simply stops the system into an infinite loop.
 *
 * @notapi
 */
#if !defined(__DOXYGEN__)
__attribute__ ((naked))
#endif
void _unhandled_exception(void) {

//  palSetPad(GPIO, PIN_LED);
  while (TRUE);
}


/*!
 * \note The following declaration is mandatory to avoid compiler errors. \n
 * In the declaration below, we are assigning the assembler label __label_hardfaultGetContext__
 * to the entry point of __hardfaultGetContext__ function
 */
void hardfaultGetContext(unsigned long* stackedContextPtr) asm("label_hardfaultGetContext");


/*!
 *  \fn void hardfaultGetContext(unsigned long* stackedContextPtr)
 *  \brief Copies system stacked context into function local variables. \n
 *  This function is called from asm-coded Interrupt Service Routine associated to HARD_FAULT exception
 *  \param stackedContextPtr : Address of stack containing stacked processor context.
 */
void hardfaultGetContext(unsigned long* stackedContextPtr)
{
    volatile unsigned long stacked_r0;
    volatile unsigned long stacked_r1;
    volatile unsigned long stacked_r2;
    volatile unsigned long stacked_r3;
    volatile unsigned long stacked_r12;
    volatile unsigned long stacked_lr;
    volatile unsigned long stacked_pc;
    volatile unsigned long stacked_psr;
    volatile unsigned long _CFSR;
    volatile unsigned long _HFSR;
    volatile unsigned long _DFSR;
    volatile unsigned long _AFSR;
    volatile unsigned long _BFAR;
    volatile unsigned long _MMAR;

    stacked_r0  = stackedContextPtr[0];
    stacked_r1  = stackedContextPtr[1];
    stacked_r2  = stackedContextPtr[2];
    stacked_r3  = stackedContextPtr[3];
    stacked_r12 = stackedContextPtr[4];
    stacked_lr  = stackedContextPtr[5];
    stacked_pc  = stackedContextPtr[6];
    stacked_psr = stackedContextPtr[7];

    // Configurable Fault Status Register
    // Consists of MMSR, BFSR and UFSR
    _CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;

    // Hard Fault Status Register
    _HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;

    // Debug Fault Status Register
    _DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;

    // Auxiliary Fault Status Register
    _AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;

    // Read the Fault Address Registers. These may not contain valid values.
    // Check BFARVALID/MMARVALID to see if they are valid values
    // MemManage Fault Address Register
    _MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
    // Bus Fault Address Register
    _BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;

    __asm("BKPT #0\n") ; // Break into the debugger

    // The following code avoids compiler warning [-Wunused-but-set-variable]
    stackedContextPtr[0] = stacked_r0;
    stackedContextPtr[1] = stacked_r1;
    stackedContextPtr[2] = stacked_r2;
    stackedContextPtr[3] = stacked_r3;
    stackedContextPtr[4] = stacked_r12;
    stackedContextPtr[5] = stacked_lr;
    stackedContextPtr[6] = stacked_pc;
    stackedContextPtr[7] = stacked_psr;
    (*((volatile unsigned long *)(0xE000ED28))) = _CFSR;
    (*((volatile unsigned long *)(0xE000ED2C))) = _HFSR;
    (*((volatile unsigned long *)(0xE000ED30))) = _DFSR;
    (*((volatile unsigned long *)(0xE000ED3C))) = _AFSR;
    (*((volatile unsigned long *)(0xE000ED34))) = _MMAR;
    (*((volatile unsigned long *)(0xE000ED38))) = _BFAR;
}


/*!
 *  \fn void hardfaultHandler(void)
 *  \brief HARD_FAULT interrupt service routine. Selects among PSP or MSP stacks and \n
 *  calls \ref hardfaultGetContext passing the selected stack pointer address as parameter.
 *  \note __naked__ attribute avoids generating prologue and epilogue code sequences generated \n
 *  for C-functions, and only pure asm instructions should be included into the function body.
 */
#if !defined(__DOXYGEN__)
__attribute__ ((naked))
#endif
void hardfaultHandler(void)
{
  __asm__ volatile (
    "      movs   r0, #4                      \n" /* determine if processor uses psp or msp by checking bit.4 at lr register.     */
    "      mov    r1, lr                      \n"
    "      tst    r0, r1                      \n"
    "      beq    _is_msp                     \n" /* jump to '_msp' if processor uses msp stack.                                  */
    "      mrs    r0, psp                     \n" /* prepare psp content as parameter to the calling function below.              */
    "      bl     label_hardfaultGetContext   \n" /* call 'hardfaultgetcontext' passing psp content as stackedcontextptr value.   */
    "_is_msp:                                 \n"
    "      mrs    r0, msp                     \n" /* prepare msp content as parameter to the calling function below.              */
    "      bl     label_hardfaultGetContext   \n" /* call 'hardfaultgetcontext' passing msp content as stackedcontextptr value.   */
    ::  );
}

void NMIVector(void) __attribute__((weak, alias("_unhandled_exception")));
void HardFaultVector(void) __attribute__((weak, alias("hardfaultHandler")));
void MemManageVector(void) __attribute__((weak, alias("_unhandled_exception")));
void BusFaultVector(void) __attribute__((weak, alias("_unhandled_exception")));
void UsageFaultVector(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector1C(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector20(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector24(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector28(void) __attribute__((weak, alias("_unhandled_exception")));
void SVCallVector(void) __attribute__((weak, alias("_unhandled_exception")));
void DebugMonitorVector(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector34(void) __attribute__((weak, alias("_unhandled_exception")));
void PendSVVector(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector3C(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector40(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector44(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector48(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector4C(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector50(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector54(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector58(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector5C(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector60(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector64(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector68(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector6C(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector70(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector74(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector78(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector7C(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector80(void) __attribute__((weak, alias("_unhandled_exception")));
void SysTickVector(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector88(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector8C(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector90(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector94(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector98(void) __attribute__((weak, alias("_unhandled_exception")));
void Vector9C(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorA0(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorA4(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorA8(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorAC(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorB0(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorB4(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorB8(void) __attribute__((weak, alias("_unhandled_exception")));
void VectorBC(void) __attribute__((weak, alias("_unhandled_exception")));

/** @} */
