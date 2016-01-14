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
 * @file    NRF51/hal_lld.c
 * @brief   NRF51 HAL subsystem low level driver source.
 *
 * @addtogroup HAL
 * @{
 */

#include "ch.h"
#include "hal.h"

/**
 * @brief   Register missing in NXP header file.
 */


/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {

  /* SysTick initialization using the RTC1 clock because NRF51 has no systic timer*/
  NRF_RTC1->PRESCALER = NRF51_LFCLK / CH_FREQUENCY - 1;
  NRF_RTC1->EVTENSET = 1;
  NRF_RTC1->INTENSET = 1;
  nvicEnableVector(RTC1_IRQn, CORTEX_PRIORITY_MASK(3));
}


/**
 * @brief   NRF51 clocks initialization.
 * @note    All the involved constants come from the file @p board.h.
 * @note    This function must be invoked only after the system reset.
 *
 * @special
 */
void nrf51_clock_init(void) {
  // HF clock initialisation
#if ISHFXTALSUPPORTED == TRUE
#if NRF51_XT_VAL ==_16MHZ_XTOSC
  NRF_CLOCK->XTALFREQ = 0xFF;
#else
  NRF_CLOCK->XTALFREQ = 0;
#endif

  // nrf51_SystemClockStart();
  NRF_CLOCK->TASKS_HFCLKSTART = 1;

  // LF clock initialisation
  NRF_CLOCK->LFCLKSRC = LFCLK_SOURCE;
  NRF_CLOCK->TASKS_LFCLKSTART = 1;
#endif
}

/** @} */
