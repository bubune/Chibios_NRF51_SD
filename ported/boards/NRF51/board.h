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

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Board identifiers.
 */
#define BOARD_DK_NRF51422
#define BOARD_NAME "pca10003"

/*
 * Board frequencies.
 */
#define NRF51_XT_VAL         _16MHZ_XTOSC

/*
 * Pin definitions.
 */

#define PIN_UART_TXD            21
#define PIN_UART_RXD            29

#define PIN_LED                 20

#define PIN_TWI0_SDA            19
#define PIN_TWI0_SCL            17

#define PIN_SPI_SCK0            4
#define PIN_SPI_MOSI0           2
#define PIN_SPI_MISO0           6

#define PIN_SPI_SEL_MEM         5

#define PIN_SPI_SEL_LCD         10
#define PIN_LCD_EXT_MODE        14   // must be 0 if command from software
#define PIN_LCD_EXT_COM         8    // must be 0 if command from software
#define PIN_LCD_DISP            12   // enables/disables the display

#define PIN_PB_TL               3
#define PIN_PB_BL               13
#define PIN_PB_C                11
#define PIN_PB_TR               24
#define PIN_PB_BR               22



/*
 * GPIO 0 initial setup.
#define VAL_GPIODIR             PAL_PORT_BIT(PIN_LED) | PAL_PORT_BIT(PIN_UART_TXD) | PAL_PORT_BIT(PIN_SPI_SCK0) |\
                                PAL_PORT_BIT(PIN_SPI_MOSI0)
 */

#define VAL_GPIODATA            0

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
