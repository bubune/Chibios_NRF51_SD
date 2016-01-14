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

static void print(char *p)
{
  while (*p)
    chSequentialStreamPut(&SD1, *p++);
}

static void printn(uint32_t n)
{
  char buf[16], *p;

  if (!n)
      chSequentialStreamPut(&SD1, '0');
  else
  {
    p = buf;
    while (n)
      *p++ = (n % 10) + '0', n /= 10;
    while (p > buf)
      chSequentialStreamPut(&SD1, *--p);
  }
}

/*
 * RGB LED blinker thread, times are in milliseconds.
 */
static WORKING_AREA(waThreadLed, 256);
static msg_t ThreadLed(void *arg)
{
  (void)arg;
  chRegSetThreadName("blinker1");
  print("\r\nHello World!");
  while (TRUE)
  {
    palClearPad(GPIO, PIN_LED);
//    print("\r\nLed ON!");
    chThdSleepMilliseconds(250);
    palSetPad(GPIO, PIN_LED);
//    print("\r\nLed OFF!");
    chThdSleepMilliseconds(250);
  }
  return 0;
}


#define HRMRX_ANT_CHANNEL 0
/*
 * ANT+ HRM thread, times are in milliseconds.
 */
static WORKING_AREA(waThreadHrm, 256);
static msg_t ThreadHrm(void *arg)
{
  (void)arg;
  uint32_t err=0;
  msg_t pData;
  antEventMessage *antEvt;

  antDeviceProfile pHrm =
  {
    .ant_channel = HRMRX_ANT_CHANNEL,
    .ant_channel_type = 0x40,
    .ant_network_number = 0,
    .ant_network_key = {0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45},
    .ant_device_number = 0,
    .ant_rf_channel = 0x39,
    .ant_trans_type = 0,
    .ant_device_type = 0x78,
    .ant_device_id = 0,
    .ant_msg_period = 0x1F86,
    .ant_assign = 0,
  };

  chRegSetThreadName("ant+_Hrm");
  print("\r\nANT+ HRM starting");

  err = sdd_open_channel(&SDD1, &pHrm);
  if(!err)
    print("\r\nHRM channel opened successfully");
  else
  {
    print("\r\nHRM channel ERR:");
    printn(err);
  }
  while (TRUE)
  {
    chMBFetch(&SDD1.imb, &pData, TIME_INFINITE);
    antEvt=(antEventMessage*)pData;

    if(antEvt->ant_event == EVENT_RX)
    {
      print("\r\n");
      print(" HRM: ");
      printn(antEvt->ant_event_message_buffer[10]);
      print("bmp");
    }
  }
  return 0;
}

// softdevice is flased by using command (in a shell) "make flash-softdevice" (see Makefile for details)
/*
 * Application entry point.
 */

int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  uint32_t err=1;

  halInit();
  chSysInit();

  /*
   * Activates the softdevice using the driver default configuration.
   */

  // serial data are buffurised and sent only after "sdStart(&SD1, NULL);"
  err = sddStart(&SDD1, NULL);

  if(!err)
    print("\r\nANT enable OK");
  else
  {
    print("\r\nFail to enable ANT, err:");
    printn(err);
  }

  /*
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);

  /*
   * Creates the blinker threads.
   */
  chThdCreateStatic(waThreadLed, sizeof(waThreadLed), NORMALPRIO+1, ThreadLed, NULL);

  /*
   * Creates the HRM threads.
   */
  chThdCreateStatic(waThreadHrm, sizeof(waThreadHrm), NORMALPRIO+1, ThreadHrm, NULL);

  // make attention to start systic timer only now  due ti softdevice!
  NRF_RTC1->TASKS_START = 1;

  // for debuging
  palSetPad(GPIO, PIN_LED);

  // wait the end of world!
  while(TRUE);

}

