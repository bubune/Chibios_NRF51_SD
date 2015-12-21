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
 * @file    NRF51/serial_lld.h
 * @brief   NRF51 low level serial driver header.
 *
 * @addtogroup SERIAL
 * @{
 */

#ifndef _SERIAL_LLD_H_
#define _SERIAL_LLD_H_

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/* Register: UART_INTEN */
/* Description: Interrupt enable clear register. */
/* Bit 17 : interrupt on RXTO event. */
#define UART_INTEN_RXTO (0x20000UL) /*!< Disable interrupt on write. */
/* Bit 9 : interrupt on ERROR event. */
#define UART_INTEN_ERROR (0x200UL) /*!< Interrupt enabled. */
/* Bit 7 : interrupt on TXRDY event. */
#define UART_INTEN_TXDRDY (0x80UL) /*!< Disable interrupt on write. */
/* Bit 2 : interrupt on RXRDY event. */
#define UART_INTEN_RXDRDY (0x4UL) /*!< Disable interrupt on write. */

/* Register: UART_ERRORSRC */
/* Description: Error source. Write error field to 1 to clear error. */
/* Bit 3 : The serial data input is '0' for longer than the length of a data frame. */
#define UART_ERRORSRC_BREAK (0x8UL)
/* Bit 2 : A valid stop bit is not detected on the serial data input after all bits in a character have been received. */
#define UART_ERRORSRC_FRAMING (0x4UL)
/* Bit 1 : A character with bad parity is received. Only checked if HW parity control is enabled. */
#define UART_ERRORSRC_PARITY (0x2UL)
/* Bit 0 : A start bit is received while the previous data still lies in RXD. (Data loss). */
#define UART_ERRORSRC_OVERRUN (0x1UL)

/* Register: UART_ENABLE */
/* Description: Enable UART and acquire IOs. */
/* Bits 2..0 : Enable or disable UART and acquire IOs. */
#define UART_DISABLE (0x0UL) /*!< UART disabled. */
#define UART_ENABLE (0x4UL) /*!< UART enabled. */

/* Register: UART_BAUDRATE */
/* Description: UART Baudrate. */
/* Bits 31..0 : UART baudrate. */
#define UART_BAUDRATE_1200 (0x0004F000UL) /*!< 1200 baud. */
#define UART_BAUDRATE_4800 (0x0013B000UL) /*!< 4800 baud. */
#define UART_BAUDRATE_9600 (0x00275000UL) /*!< 9600 baud. */
#define UART_BAUDRATE_14400 (0x003B0000UL) /*!< 14400 baud. */
#define UART_BAUDRATE_19200 (0x004EA000UL) /*!< 19200 baud. */
#define UART_BAUDRATE_28800 (0x0075F000UL) /*!< 28800 baud. */
#define UART_BAUDRATE_38400 (0x009D5000UL) /*!< 38400 baud. */
#define UART_BAUDRATE_57600 (0x00EBF000UL) /*!< 57600 baud. */
#define UART_BAUDRATE_76800 (0x013A9000UL) /*!< 76800 baud. */
#define UART_BAUDRATE_115200 (0x01D7E000UL) /*!< 115200 baud. */
#define UART_BAUDRATE_230400 (0x03AFB000UL) /*!< 230400 baud. */
#define UART_BAUDRATE_250000 (0x04000000UL) /*!< 250000 baud. */
#define UART_BAUDRATE_460800 (0x075F7000UL) /*!< 460800 baud. */
#define UART_BAUDRATE_921600 (0x0EBEDFA4UL) /*!< 921600 baud. */
#define UART_BAUDRATE_1M (0x10000000UL) /*!< 1M baud. */

/* Register: UART_CONFIG */
/* Description: Configuration of parity and hardware flow control register. */
/* Bits 3..1 : Include parity bit. */
#define UART_CONFIG_PARITY_ENABLE (0xEUL) /*!< Parity bit enable. */
#define UART_CONFIG_PARITY_DISABLE (0x0UL) /*!< Parity bit disable. */
/* Bit 0 : Hardware flow control. */
#define UART_CONFIG_HWFC_ENABLE (1UL) /*!< Hardware flow control enabled. */
#define UART_CONFIG_HWFC_DISABLE (1UL) /*!< Hardware flow control disable. */

#define  UART_PIN_DISCONNECTED  0xFFFFFFFF  /**< Value indicating that no pin is connected to this UART register. */


/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   UART0 driver enable switch.
 * @details If set to @p TRUE the support for UART0 is included.
 * @note    The default is @p TRUE .
 */
#if !defined(NRF51_SERIAL_USE_UART0) || defined(__DOXYGEN__)
#define NRF51_SERIAL_USE_UART0            TRUE
#endif

/**
 * @brief   UART0 interrupt priority level setting.
 */
#if !defined(NRF51_SERIAL_UART0_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define NRF51_SERIAL_UART0_IRQ_PRIORITY   3
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/


/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   NRF51 Serial Driver configuration structure.
 * @details An instance of this structure must be passed to @p sdStart()
 *          in order to configure and start a serial driver operations.
 */
typedef struct {
  /**
   * @brief Bit rate.
   */
  uint32_t                  sc_speed;
  /**
   * @brief Initialization value for the CONFIG register.
   */
  uint32_t                  sc_config;

} SerialConfig;

/**
 * @brief   @p SerialDriver specific data.
 */
#define _serial_driver_data                                                 \
  _base_asynchronous_channel_data                                           \
  /* Driver state.*/                                                        \
  sdstate_t                 state;                                          \
  /* Input queue.*/                                                         \
  InputQueue                iqueue;                                         \
  /* Output queue.*/                                                        \
  OutputQueue               oqueue;                                         \
  /* Input circular buffer.*/                                               \
  uint8_t                   ib[SERIAL_BUFFERS_SIZE];                        \
  /* Output circular buffer.*/                                              \
  uint8_t                   ob[SERIAL_BUFFERS_SIZE];                        \
  /* End of the mandatory fields.*/                                         \
  /* Pointer to the USART registers block.*/                                \
  NRF_UART_Type           *uart;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if NRF51_SERIAL_USE_UART0 && !defined(__DOXYGEN__)
extern SerialDriver SD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void sd_lld_init(void);
  void sd_lld_start(SerialDriver *sdp, const SerialConfig *config);
  void sd_lld_stop(SerialDriver *sdp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SERIAL */

#endif /* _SERIAL_LLD_H_ */

/** @} */
