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
 * @file    NRF51/pal_lld.h
 * @brief   NRF51 GPIO low level driver header.
 *
 * @addtogroup PAL
 * @{
 */

#ifndef _PAL_LLD_H_
#define _PAL_LLD_H_

#if HAL_USE_PAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Unsupported modes and specific modes                                      */
/*===========================================================================*/

#undef PAL_MODE_INPUT_PULLUP
#undef PAL_MODE_INPUT_PULLDOWN
#undef PAL_MODE_INPUT_ANALOG
#undef PAL_MODE_OUTPUT_OPENDRAIN

/*===========================================================================*/
/* I/O Ports Types and constants.                                            */
/*===========================================================================*/

/**
 * @brief   Width, in bits, of an I/O port.
 */
#define PAL_IOPORTS_WIDTH 32

/**
 * @brief   Whole port mask.
 * @brief   This macro specifies all the valid bits into a port.
 */
#define PAL_WHOLE_PORT ((ioportmask_t)0xFFFFFFFF)

/**
 * @brief   GPIO port setup info.
 */
typedef struct {
  /** Initial value for GPIO_PIN register.*/
  uint32_t      data;
  /** Initial value for PIN_CNF[n] register.*/
  uint32_t      cnf[PAL_IOPORTS_WIDTH];
} nrf51_gpio_setup_t;

/**
 * @brief   GPIO static initializer.
 * @details An instance of this structure must be passed to @p palInit() at
 *          system startup time in order to initialized the digital I/O
 *          subsystem. This represents only the initial setup, specific pads
 *          or whole ports can be reprogrammed at later time.
 * @note    The @p IOCON block is not configured, initially all pins have
 *          enabled pullups and are programmed as GPIO. It is responsibility
 *          of the various drivers to reprogram the pins in the proper mode.
 *          Pins that are not handled by any driver may be programmed in
 *          @p board.c.
 */
typedef struct {
  /** @brief GPIO 0 setup data.*/
  nrf51_gpio_setup_t   P0;
} PALConfig;

/**
 * @brief   Digital I/O port sized unsigned type.
 */
typedef uint32_t ioportmask_t;

/**
 * @brief   Digital I/O modes.
 */
typedef uint32_t iomode_t;

/**
 * @brief   Port Identifier.
 */
typedef NRF_GPIO_Type *ioportid_t;

/* Peripheral: GPIO */
/* Description: General purpose input and output. */

/* Register: GPIO_PIN_CNF */
/* Description: Configuration of GPIO pins. */
/* Bits 17..16 : Pin sensing mechanism. */
#define GPIO_PIN_CNF_SENSE_DISABLE     (0x00000UL) /*!< Disabled. */
#define GPIO_PIN_CNF_SENSE_HIGH        (0x20000UL) /*!< Wakeup on high level. */
#define GPIO_PIN_CNF_SENSE_LOW         (0x30000UL) /*!< Wakeup on low level. */
#define GPIO_PIN_CNF_SENSE_MSK         (0x30000UL) /*!< Mask. */
/* Bits 10..8 : Drive configuration. */
#define GPIO_PIN_CNF_DRIVE_S0S1        (0x000UL) /*!< Standard '0', Standard '1'. */
#define GPIO_PIN_CNF_DRIVE_H0S1        (0x100UL) /*!< High '0', Standard '1'. */
#define GPIO_PIN_CNF_DRIVE_S0H1        (0x200UL) /*!< Standard '0', High '1'. */
#define GPIO_PIN_CNF_DRIVE_H0H1        (0x300UL) /*!< High '0', High '1'. */
#define GPIO_PIN_CNF_DRIVE_D0S1        (0x400UL) /*!< Disconnected '0', Standard '1'. */
#define GPIO_PIN_CNF_DRIVE_D0H1        (0x500UL) /*!< Disconnected '0', High '1'. */
#define GPIO_PIN_CNF_DRIVE_S0D1        (0x600UL) /*!< Standard '0', Disconnected '1'. */
#define GPIO_PIN_CNF_DRIVE_H0D1        (0x700UL) /*!< High '0', Disconnected '1'. */
/* Bits 3..2 : Pull-up or -down configuration. */
#define GPIO_PIN_CNF_PULL_DISABLE      (0x00UL) /*!< No pull. */
#define GPIO_PIN_CNF_PULL_PULLDOWN     (0x04UL) /*!< Pulldown on pin. */
#define GPIO_PIN_CNF_PULL_PULLUP       (0x0CUL) /*!< Pullup on pin. */
/* Bit 1 : Connect or disconnect input path. */
#define GPIO_PIN_CNF_INPUT_CONNECT     (0UL) /*!< Connect input pin. */
#define GPIO_PIN_CNF_INPUT_DISCONNECT  (2UL) /*!< Disconnect input pin. */
/* Bit 0 : Pin direction. */
#define GPIO_PIN_CNF_DIR_INPUT         (0UL) /*!< Configure pin as an input pin. */
#define GPIO_PIN_CNF_DIR_OUTPUT        (1UL) /*!< Configure pin as an output pin. */


/* Peripheral: GPIOTE */
/* Description: GPIO tasks and events. */
/* Bit 31 : Enable interrupt on PORT event. */
#define GPIOTE_PORT_ENABLE             (0x80000000) /*!< Interrupt define. */
/* Bit 3 : Enable interrupt on IN[3] event. */
#define GPIOTE_IN3_ENABLE              (0x8UL) /*!< Interrupt define. */
/* Bit 2 : Enable interrupt on IN[2] event. */
#define GPIOTE_IN2_ENABLE              (0x4UL) /*!< Interrupt define. */
/* Bit 1 : Enable interrupt on IN[2] event. */
#define GPIOTE_IN1_ENABLE              (0x2UL) /*!< Interrupt define. */
/* Bit 0 : Enable interrupt on IN[2] event. */
#define GPIOTE_IN0_ENABLE              (0x1UL) /*!< Interrupt define. */


/* Register: GPIOTE_CONFIG */
/* Description: Channel configuration registers. */
/* Bit 20 : Initial value of the output when the GPIOTE channel is configured as a Task. */
#define GPIOTE_CONFIG_OUTINIT_LOW      (0UL) /*!< Initial low output when in task mode. */
#define GPIOTE_CONFIG_OUTINIT_HIGH     (0x100000UL) /*!< Initial high output when in task mode. */
/* Bits 17..16 : Effects on output when in Task mode, or events on input that generates an event. */
#define GPIOTE_CONFIG_POLARITY_LOTOHI  (0x10000UL) /*!< Low to high. */
#define GPIOTE_CONFIG_POLARITY_HITOLO  (0x20000UL) /*!< High to low. */
#define GPIOTE_CONFIG_POLARITY_TOGGLE  (0x30000UL) /*!< Toggle. */
/* Bits 12..8 : Pin select. */
#define GPIOTE_CONFIG_PSEL             (0x1F00UL) /*!< PSEL field. */
#define GPIOTE_CONFIG_SHIFT            (7)        /*!< PSEL shift field. */
/* Bits 1..0 : Mode */
#define GPIOTE_CONFIG_MODE_DISABLE     (0x01UL) /*!< Channel disabled. */
#define GPIOTE_CONFIG_MODE_EVENT       (0x01UL) /*!< Channel configure in event mode. */
#define GPIOTE_CONFIG_MODE_TASK        (0x03UL) /*!< Channel configure in task mode. */


/*===========================================================================*/
/* I/O Ports Identifiers.                                                    */
/*===========================================================================*/

/**
 * @brief   GPIO port identifier.
 */
#define IOPORT         NRF_GPIO
#define GPIO           NRF_GPIO

/*===========================================================================*/
/* Implementation, some of the following macros could be implemented as      */
/* functions, if so please put them in pal_lld.c.                            */
/*===========================================================================*/

/**
 * @brief   Low level PAL subsystem initialization.
 *
 * @param[in] config    architecture-dependent ports configuration
 *
 * @notapi
 */
#define pal_lld_init(config) _pal_lld_init(config)

/**
 * @brief   Reads the physical I/O port states.
 *
 * @param[in] port      port identifier
 * @return              The port bits.
 *
 * @notapi
 */
#define pal_lld_readport(port) ((port)->IN)

/**
 * @brief   Reads the output latch.
 * @details The purpose of this function is to read back the latched output
 *          value.
 *
 * @param[in] port      port identifier
 * @return              The latched logical states.
 *
 * @notapi
 */
#define pal_lld_readlatch(port) ((port)->IN)

/**
 * @brief   Writes a bits mask on a I/O port.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be written on the specified port
 *
 * @notapi
 */
#define pal_lld_writeport(port, bits) ((port)->OUT = (bits))

/**
 * @brief   Sets a bits mask on a I/O port.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be ORed on the specified port
 *
 * @notapi
 */
#define pal_lld_setport(port, bits) ((port)->OUTSET = bits)

/**
 * @brief   Clears a bits mask on a I/O port.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be cleared on the specified port
 *
 * @notapi
 */
#define pal_lld_clearport(port, bits) ((port)->OUTCLR = bits)

/**
 * @brief   Pads group mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
 * @note    Programming an unknown or unsupported mode is silently ignored.
 *
 * @param[in] port      port identifier
 * @param[in] mask      group mask
 * @param[in] offset    group bit offset within the port
 * @param[in] mode      group mode
 *
 * @notapi
 */
#define pal_lld_setgroupmode(port, mask, offset, mode)                      \
  _pal_lld_setgroupmode(port, mask << offset, mode)


#if !defined(__DOXYGEN__)
extern const PALConfig pal_default_config;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void _pal_lld_init(const PALConfig *config);
  void _pal_lld_setgroupmode(ioportid_t port,
                             ioportmask_t mask,
                             iomode_t mode);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PAL */

#endif /* _PAL_LLD_H_ */

/** @} */
