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

#include "ch.h"
#include "hal.h"

/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
#if HAL_USE_PAL || defined(__DOXYGEN__)

/*  I2C Pin Conf */
#define PIN_CNF_SCL          GPIO_PIN_CNF_SENSE_DISABLE  | \
                             GPIO_PIN_CNF_DRIVE_S0D1     | \
                             GPIO_PIN_CNF_PULL_PULLUP    | \
                             GPIO_PIN_CNF_INPUT_CONNECT  | \
                             GPIO_PIN_CNF_DIR_INPUT

#define PIN_CNF_SDA          GPIO_PIN_CNF_SENSE_DISABLE  | \
                             GPIO_PIN_CNF_DRIVE_S0D1     | \
                             GPIO_PIN_CNF_PULL_PULLUP    | \
                             GPIO_PIN_CNF_INPUT_CONNECT  | \
                             GPIO_PIN_CNF_DIR_INPUT


/* SPI Pin Conf */
#define PIN_CNF_SCK          GPIO_PIN_CNF_SENSE_DISABLE | \
                             GPIO_PIN_CNF_DRIVE_S0S1    | \
                             GPIO_PIN_CNF_PULL_DISABLE  | \
                             GPIO_PIN_CNF_INPUT_CONNECT | \
                             GPIO_PIN_CNF_DIR_OUTPUT

#define PIN_CNF_MOSI         GPIO_PIN_CNF_SENSE_DISABLE | \
                             GPIO_PIN_CNF_DRIVE_S0S1    | \
                             GPIO_PIN_CNF_PULL_DISABLE  | \
                             GPIO_PIN_CNF_INPUT_CONNECT | \
                             GPIO_PIN_CNF_DIR_OUTPUT

#define PIN_CNF_MISO         GPIO_PIN_CNF_SENSE_DISABLE | \
                             GPIO_PIN_CNF_DRIVE_S0D1    | \
                             GPIO_PIN_CNF_PULL_DISABLE  | \
                             GPIO_PIN_CNF_INPUT_CONNECT | \
                             GPIO_PIN_CNF_DIR_INPUT


/* Serial Pin Conf */
#define PIN_CNF_TXD          GPIO_PIN_CNF_SENSE_DISABLE | \
                             GPIO_PIN_CNF_DRIVE_S0S1    | \
                             GPIO_PIN_CNF_PULL_DISABLE  | \
                             GPIO_PIN_CNF_INPUT_CONNECT | \
                             GPIO_PIN_CNF_DIR_OUTPUT

#define PIN_CNF_RXD          GPIO_PIN_CNF_SENSE_DISABLE | \
                             GPIO_PIN_CNF_DRIVE_S0S1    | \
                             GPIO_PIN_CNF_PULL_DISABLE  | \
                             GPIO_PIN_CNF_INPUT_CONNECT | \
                             GPIO_PIN_CNF_DIR_INPUT

/* General pin Conf */
#define PIN_CNF_OUTPUT       GPIO_PIN_CNF_SENSE_DISABLE | \
                             GPIO_PIN_CNF_DRIVE_S0S1    | \
                             GPIO_PIN_CNF_PULL_DISABLE  | \
                             GPIO_PIN_CNF_INPUT_CONNECT | \
                             GPIO_PIN_CNF_DIR_OUTPUT

#define PIN_CNF_INPUT        GPIO_PIN_CNF_SENSE_DISABLE | \
                             GPIO_PIN_CNF_DRIVE_S0S1    | \
                             GPIO_PIN_CNF_PULL_DISABLE  | \
                             GPIO_PIN_CNF_INPUT_CONNECT | \
                             GPIO_PIN_CNF_DIR_INPUT

#define PIN_CNF_INPUT_SENSE_LOW                           \
                             GPIO_PIN_CNF_SENSE_DISABLE | \
                             GPIO_PIN_CNF_DRIVE_S0S1    | \
                             GPIO_PIN_CNF_PULL_DISABLE  | \
                             GPIO_PIN_CNF_INPUT_CONNECT | \
                             GPIO_PIN_CNF_DIR_INPUT




#define PIN_CNF_RESET_VAL    0x02

// Pin mapping
const PALConfig pal_default_config =
{
  {
    VAL_GPIODATA,
    {
      PIN_CNF_RESET_VAL,                       // 0:
      PIN_CNF_RESET_VAL,                       // 1:
      PIN_CNF_MOSI,                            // 2:  PIN_SPI_MOSI0
      PIN_CNF_INPUT_SENSE_LOW,                 // 3:  PIN_PB_TL
      PIN_CNF_SCK,                             // 4:  PIN_SPI_SCK0
      PIN_CNF_RESET_VAL,                       // 5:  PIN_SPI_SEL_MEM
      PIN_CNF_MISO,                            // 6:  PIN_SPI_MISO0
      PIN_CNF_RESET_VAL,                       // 7:
      PIN_CNF_OUTPUT,                          // 8:  PIN_LCD_EXT_COM
      PIN_CNF_RESET_VAL,                       // 9:
      PIN_CNF_OUTPUT,                          // 10: PIN_SPI_SEL_LCD
      PIN_CNF_INPUT_SENSE_LOW,                 // 11: PIN_PB_C
      PIN_CNF_OUTPUT,                          // 12: PIN_LCD_DISP
      PIN_CNF_INPUT_SENSE_LOW,                 // 13: PIN_PB_BL
      PIN_CNF_OUTPUT,                          // 14: PIN_LCD_EXT_MODE
      PIN_CNF_RESET_VAL,                       // 15:
      PIN_CNF_RESET_VAL,                       // 16:
      PIN_CNF_SCL,                             // 17: PIN_TWI0_SCL
      PIN_CNF_RESET_VAL,                       // 18:
      PIN_CNF_SDA,                             // 19: PIN_TWI0_SDA
      PIN_CNF_OUTPUT,                          // 20: PIN_LED
      PIN_CNF_TXD,                             // 21: PIN_UART_TXD
      PIN_CNF_INPUT_SENSE_LOW,                 // 22: PIN_PB_BR
      PIN_CNF_RESET_VAL,                       // 23:
      PIN_CNF_INPUT_SENSE_LOW,                 // 24: PIN_PB_TR
      PIN_CNF_RESET_VAL,                       // 25:
      PIN_CNF_RESET_VAL,                       // 26:
      PIN_CNF_RESET_VAL,                       // 27:
      PIN_CNF_RESET_VAL,                       // 28:
      PIN_CNF_RXD,                             // 29: PIN_UART_RXD
      PIN_CNF_RESET_VAL,                       // 30:
      PIN_CNF_RESET_VAL                        // 31:
    }
  }
};

#endif

static bool is_manual_peripheral_setup_needed(void);
static bool is_disabled_in_debug_needed(void);

/* Register: MPU_DISABLEINDEBUG */
/* Description: Disable protection mechanism in debug mode. */

/* Bit 0 : Disable protection mechanism in debug mode. */
#define MPU_DISABLEINDEBUG_DISABLEINDEBUG_Pos (0UL) /*!< Position of DISABLEINDEBUG field. */
#define MPU_DISABLEINDEBUG_DISABLEINDEBUG_Msk (0x1UL << MPU_DISABLEINDEBUG_DISABLEINDEBUG_Pos) /*!< Bit mask of DISABLEINDEBUG field. */
#define MPU_DISABLEINDEBUG_DISABLEINDEBUG_Enabled (0UL) /*!< Protection enabled. */
#define MPU_DISABLEINDEBUG_DISABLEINDEBUG_Disabled (1UL) /*!< Protection disabled. */

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
void __early_init(void) {

  /* If desired, switch off the unused RAM to lower consumption by the use of RAMON register.
     It can also be done in the application main() function. */

  /* Prepare the peripherals for use as indicated by the PAN 26 "System: Manual setup is required
     to enable the use of peripherals" found at Product Anomaly document for your device found at
     https://www.nordicsemi.com/. The side effect of executing these instructions in the devices
     that do not need it is that the new peripherals in the second generation devices (LPCOMP for
     example) will not be available. */
  if (is_manual_peripheral_setup_needed())
  {
    *(uint32_t *)0x40000504 = 0xC007FFDF;
    *(uint32_t *)0x40006C18 = 0x00008000;
  }

  /* Disable PROTENSET registers under debug, as indicated by PAN 59 "MPU: Reset value of DISABLEINDEBUG
     register is incorrect" found at Product Anomaly document four your device found at
     https://www.nordicsemi.com/. There is no side effect of using these instruction if not needed. */
  if (is_disabled_in_debug_needed())
  {
      NRF_MPU->DISABLEINDEBUG = MPU_DISABLEINDEBUG_DISABLEINDEBUG_Disabled << MPU_DISABLEINDEBUG_DISABLEINDEBUG_Pos;
  }

  nrf51_clock_init();
}


static bool is_manual_peripheral_setup_needed(void)
{
  if ((((*(uint32_t *)0xF0000FE0) & 0x000000FF) == 0x1) && (((*(uint32_t *)0xF0000FE4) & 0x0000000F) == 0x0))
  {
    if ((((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x00) && (((*(uint32_t *)0xF0000FEC) & 0x000000F0) == 0x0))
    {
      return true;
    }
    if ((((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x10) && (((*(uint32_t *)0xF0000FEC) & 0x000000F0) == 0x0))
    {
      return true;
    }
    if ((((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x30) && (((*(uint32_t *)0xF0000FEC) & 0x000000F0) == 0x0))
    {
      return true;
    }
  }
  return false;
}

static bool is_disabled_in_debug_needed(void)
{
  if ((((*(uint32_t *)0xF0000FE0) & 0x000000FF) == 0x1) && (((*(uint32_t *)0xF0000FE4) & 0x0000000F) == 0x0))
  {
    if ((((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x40) && (((*(uint32_t *)0xF0000FEC) & 0x000000F0) == 0x0))
    {
      return true;
    }
  }
  return false;
}

/*
 * Board-specific initialization code.
 */
void boardInit(void) {
}
