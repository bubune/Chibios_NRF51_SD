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
 * @file    NRF51/serial_lld.c
 * @brief   NRF51 low level serial driver code.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#if NRF51_SERIAL_USE_UART0 || defined(__DOXYGEN__)
/** @brief UART0 serial driver identifier.*/
SerialDriver SD1;
#endif

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/** @brief Driver default configuration.*/
static const SerialConfig default_config = {
  SERIAL_DEFAULT_BITRATE,
  UART_CONFIG_PARITY_ENABLE | UART_CONFIG_HWFC_DISABLE,
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   UART initialization.
 *
 * @param[in] sdp       communication channel associated to the UART
 * @param[in] config    the architecture-dependent serial driver configuration
 */
static void uart_init(SerialDriver *sdp, const SerialConfig *config) {
  NRF_UART_Type *u = sdp->uart;

  // baurate formula:
  // The formula is: Baudrate = desired baudrate * 2^32 / 16000000
  // Note that you will have to round the number afterwards:
  // rounded_value = (value + 0x800) & 0xFFFFF000

  uint64_t br = (config->sc_speed * ((uint64_t)1 << 32)) / 16000000;

  u->CONFIG           = 0;
  u->BAUDRATE         = UART_BAUDRATE_115200;
//  u->BAUDRATE         = ((uint32_t)br + 0x800) & 0xFFFFF000;
//  u->CONFIG           = config->sc_config;
  u->EVENTS_TXDRDY    = 0;
  u->EVENTS_RXDRDY    = 0;
  u->ERRORSRC         = 0x0F;
  u->ENABLE           = UART_ENABLE;
  u->INTENCLR         = 0xFFFFFFFF;
  u->INTENSET         = UART_INTEN_RXDRDY | UART_INTEN_ERROR;
  u->TASKS_STARTRX    = 1;
}

/**
 * @brief   UART de-initialization.
 *
 * @param[in] u         pointer to an UART I/O block
 */
static void uart_deinit( NRF_UART_Type *u) {

  u->INTENCLR         = 0xFFFFFFFF;
  u->ENABLE           = UART_DISABLE;
  u->TASKS_STOPTX     = 1;
  u->TASKS_STOPRX     = 1;
}

/**
 * @brief   Error handling routine.
 *
 * @param[in] sdp       communication channel associated to the UART
 * @param[in] err       UART ERRORSRC register value
 */
static void set_error(SerialDriver *sdp, IOREG32 err) {
  flagsmask_t sts = 0;

  if (err & UART_ERRORSRC_OVERRUN)
    sts |= SD_OVERRUN_ERROR;
  if (err & UART_ERRORSRC_PARITY)
    sts |= SD_PARITY_ERROR;
  if (err & UART_ERRORSRC_FRAMING)
    sts |= SD_FRAMING_ERROR;
  if (err & UART_ERRORSRC_BREAK)
    sts |= SD_BREAK_DETECTED;
  chSysLockFromIsr();
  chnAddFlagsI(sdp, sts);
  chSysUnlockFromIsr();
}

/**
 * @brief   Common IRQ handler.
 * @note    Tries hard to clear all the pending interrupt sources, we don't
 *          want to go through the whole ISR and have another interrupt soon
 *          after.
 *
 * @param[in] u         pointer to an UART I/O block
 * @param[in] sdp       communication channel associated to the UART
 */
static void serve_interrupt(SerialDriver *sdp) {
  NRF_UART_Type *u = sdp->uart;
  if (((u->INTENSET & UART_INTEN_RXDRDY) && u->EVENTS_RXDRDY) != 0)
  {
    // Clear UART RX event flag
    u->EVENTS_RXDRDY = 0;
    chSysLockFromIsr();
    if (chIQIsEmptyI(&sdp->iqueue))
      chnAddFlagsI(sdp, CHN_INPUT_AVAILABLE);
    if (chIQPutI(&sdp->iqueue, u->RXD) < Q_OK)
      chnAddFlagsI(sdp, SD_OVERRUN_ERROR);
    chSysUnlockFromIsr();
  }

  // Handle transmission.
  if (((u->INTENSET & UART_INTEN_TXDRDY) && u->EVENTS_TXDRDY) != 0)
  {
    msg_t b;

    // Clear UART TX event flag.
    u->EVENTS_TXDRDY = 0;
    chSysLockFromIsr();
    b = chOQGetI(&sdp->oqueue);
    chSysUnlockFromIsr();
    if (b < Q_OK) {
      chSysLockFromIsr();
      chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
      chSysUnlockFromIsr();
      u->TASKS_STOPTX = 1;
      u->INTENCLR = UART_INTEN_TXDRDY;
    }
    else
      u->TXD = b;
  }

  // Handle errors.
  if (((u->INTENSET & UART_INTEN_ERROR) && u->EVENTS_ERROR) != 0)
  {
    // Clear UART ERROR event flag.
    u->EVENTS_ERROR = 0;
    set_error(sdp, u->ERRORSRC);
    // Clear error source.
    u->ERRORSRC = 0;
  }
}


/**
 * @brief   Driver SD1 output notification.
 */
#if NRF51_SERIAL_USE_UART0 || defined(__DOXYGEN__)
static void notify1(GenericQueue *qp) {
  NRF_UART_Type *u = SD1.uart;

  (void)qp;
  if (!(u->INTENSET & UART_INTEN_TXDRDY)) {
    msg_t b = chOQGetI(&SD1.oqueue);
    if (b < Q_OK) {
      chnAddFlagsI(&SD1, CHN_OUTPUT_EMPTY);
      return;
    }
    u->INTENSET = UART_INTEN_TXDRDY;
    u->TASKS_STARTTX = 1;
    u->TXD = b;
  }
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   UART0 IRQ handler.
 *
 * @isr
 */
#if NRF51_SERIAL_USE_UART0 || defined(__DOXYGEN__)
CH_IRQ_HANDLER(UART0_IRQHandler) {

  CH_IRQ_PROLOGUE();

  serve_interrupt(&SD1);

  CH_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level serial driver initialization.
 *
 * @notapi
 */
void sd_lld_init(void) {

#if NRF51_SERIAL_USE_UART0
  sdObjectInit(&SD1, NULL, notify1);
  SD1.uart = NRF_UART0;
  NRF_UART0->PSELRTS = UART_PIN_DISCONNECTED;      /* No hardware flow control */
  NRF_UART0->PSELCTS = UART_PIN_DISCONNECTED;      /* No hardware flow control */

#if 0
  // Configure RX and TX pins.
  NRF_GPIO->PIN_CNF[PIN_UART_TXD] = GPIO_PIN_CNF_SENSE_DISABLE |
                                    GPIO_PIN_CNF_DRIVE_S0S1     |
                                    GPIO_PIN_CNF_PULL_DISABLE  |
                                    GPIO_PIN_CNF_INPUT_CONNECT  |
                                    GPIO_PIN_CNF_DIR_OUTPUT;

  NRF_GPIO->PIN_CNF[PIN_UART_RXD] = GPIO_PIN_CNF_SENSE_DISABLE |
                                    GPIO_PIN_CNF_DRIVE_S0S1     |
                                    GPIO_PIN_CNF_PULL_DISABLE  |
                                    GPIO_PIN_CNF_INPUT_CONNECT  |
                                    GPIO_PIN_CNF_DIR_INPUT;
#endif

  NRF_UART0->PSELTXD = PIN_UART_TXD;
  NRF_UART0->PSELRXD = PIN_UART_RXD;
#endif
}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 *
 * @notapi
 */
void sd_lld_start(SerialDriver *sdp, const SerialConfig *config) {

  if (config == NULL)
    config = &default_config;

  if (sdp->state == SD_STOP) {
#if NRF51_SERIAL_USE_UART0
    if (&SD1 == sdp) {
//      sd_nvic_SetPriority(UART0_IRQn,3);  =>  It seems it doesn't work!
//      sd_nvic_EnableIRQ(UART0_IRQn);
      nvicEnableVector(UART0_IRQn,
                       CORTEX_PRIORITY_MASK(NRF51_SERIAL_UART0_IRQ_PRIORITY));
    }
#endif
  }
  uart_init(sdp, config);
}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the UART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
void sd_lld_stop(SerialDriver *sdp) {

  if (sdp->state == SD_READY) {
    uart_deinit(sdp->uart);
#if NRF51_SERIAL_USE_UART0
    if (&SD1 == sdp) {
      nvicDisableVector(UART0_IRQn);
      return;
    }
#endif
  }
}

#endif /* HAL_USE_SERIAL */

/** @} */
