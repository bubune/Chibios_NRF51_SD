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
 * @file    NRF51/hal_lld.h
 * @brief   HAL subsystem low level driver header template.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef _HAL_LLD_H_
#define _HAL_LLD_H_

#include "nrf51.h"
#include "nvic.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Defines the support for realtime counters in the HAL.
 */
#define HAL_IMPLEMENTS_COUNTERS FALSE

/**
 * @brief   Platform name.
 */
#define PLATFORM_NAME           "NRF51"

#define NRF51_SYSCLK            16000000    /**< HFCLK clock frequency       */
#define NRF51_LFCLK             32768       /**< LFCLK clock frequency       */

#define _16MHZ_XTOSC            0           /**< Crystal osc is 16MHz        */
#define _32MHZ_XTOSC            1           /**< Crystal osc is 32MHz        */

//=> dans board.h
#define ISHFXTALSUPPORTED       TRUE        /**< Is HF Crystal present       */
#define ISLFXTALSUPPORTED       FALSE       /**< Is LF Crystal present       */

#define HFCLKSEL_RCOSC          0           /**< Clock source is RC.         */
#define HFCLKSEL_XTOSC          1           /**< Clock source is XTAL.       */

#define LFCLKSEL_RCOSC          0           /**< Clock source is RC.         */
#define LFCLKSEL_XTOSC          1           /**< Clock source is XTAL.       */
#define LFCLKSEL_SYNTHOSC       2           /**< Clock source is SYNTH.      */

//=> dans board.h
#define LFCLK_SOURCE            LFCLKSEL_RCOSC


/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

#define NRF51_HAS_I2C1          TRUE
#define NRF51_HAS_I2C2          TRUE

/*===========================================================================*/
/* Platform specific friendly IRQ names.                                     */
/*===========================================================================*/

/**
 * @name  IRQ VECTOR names
 * @{
 */

#define UART0_IRQHandler        Vector48    /**< UART0                       */
#define SPI0_TWI0_IRQHandler    Vector4C    /**< SPI0_TWI0_IRQHandler        */
#define SPI1_TWI1_IRQHandler    Vector50    /**< SPI1_TWI1_IRQHandler        */
#define GPIOTE_IRQHandler       Vector58    /**< GPIOTE_IRQHandler           */
#define SWI2_IRQHandler         Vector98    /**< SOFTDEVICE_IRQHandler       */

/** @} */




/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Crystal oscillator frequency.
 */
#if !defined(NRF51_XT_VAL) || defined(__DOXYGEN__)
#define NRF51_XT_VAL                 _16MHZ_XTOSC
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
  void nrf51_clock_init(void);
#ifdef __cplusplus
}
#endif

#endif /* _HAL_LLD_H_ */

/** @} */
