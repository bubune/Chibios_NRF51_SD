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


/*
 * ANT+ HRM thread, times are in milliseconds.
 */
static WORKING_AREA(waThreadHrm, 128);
static msg_t ThreadHrm(void *arg)
{
  (void)arg;
  uint32_t err=0;
  msg_t pData;
  antEventMessage *antEvt;
//  uint8_t i;
//  uint32_t cnt=0;

  antDeviceProfile pHrm =
  {
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

//    printn(cnt++);

#if 0
    print(" Ch: ");
    printn(antEvt->ant_channel);
    print(" Evt: ");
    printn(antEvt->ant_event);
    print(" Msg: ");
    for(i=0; i<13; i++)
    {
      print(" ");
      printn(antEvt->ant_event_message_buffer[i]);
    }
#endif

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
  uint32_t err;

// 0xE000E100 is the irq enable register address.
// I use gdb to do the debug (make debug).
// "make debug" in the shell will open 2 new shell.
// - first shell to connect to the segger probe, the one provided in the nordic NRF51 DK
// - second shell will display the gdb shell
// "x /1xw addr" will display the value @addr in hexadecimal
// "p var" will display the content of var

// 1.
// (gdb) x /1xw 0xE000E100 => 0xe000e100:     0x00000000

  halInit();
  chSysInit();
// 2.
// (gdb) x /1xw 0xE000E100 => 0xe000e100:     0x00020000
// IRQ enabled => RTC1_IRQn
// OK, It's enabled in halInit()/hal_lld.c()

  /*
   * Activates the softdevice using the driver default configuration.
   */
  // serial data are buffurised and sent only after "sdStart(&SD1, NULL);"
  err=1;
  err=sddStart(&SDD1, NULL);
//3.
//(gdb) p err => 0 (NRF_SUCESS)
//(gdb) x /1xw 0xE000E100 => 0xe000e100:     0x03402803
// IRQ enabled => POWER_CLOCK_IRQn | RADIO_IRQn | RTC0_IRQn | RNG_IRQn | SWI2_IRQn | SWI4_IRQn | SWI5_IRQn
// why RTC1_IRQn irq enable bit is back to "0". RTC1 is in "open" access when SD210 is enabled
// what's the behavior of the softdevice regarding interrupt enabled before the softdevice? It appears softdevice disable all interrupt and then enable is own interrupt. Why not, but it's not documented.

  if(!err)
    print("\r\nANT enable OK");
  else
  {
    print("\r\nFail to enable ANT, err:");
    printn(err);
  }

   err = sd_nvic_SetPriority(RTC1_IRQn,3);
//4.
//(gdb) p err => 0 (NRF_SUCESS)
// (gdb) x /1xw 0xE000E100 => 0xe000e100:     0x03402803
// regarding irq, no change

   err = sd_nvic_EnableIRQ(RTC1_IRQn); //=> It seems it doesn't work
//5.
//(gdb) p err => 0 (NRF_SUCESS)
// (gdb) x /1xw 0xE000E100 => 0xe000e100:     0x03402803
// regarding irq, no change
// strange here, RTC1_IRQn is not enabled! It should :-(

   nvicEnableVector(RTC1_IRQn, CORTEX_PRIORITY_MASK(3));
//6.
//(gdb) x /1xw 0xE000E100 => 0xe000e100:     0x03422803
// Now RTC1_IRQn is enable!
//(gdb) p err => 0

  /*
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);
// if I use softdevice call, interrupt are noy enabled but if I use normal way, interrupt are enabled like RTC1_IRQn. Here serial port is not working with sd_calls


  /*
   * Creates the blinker threads.
   2*/
  chThdCreateStatic(waThreadLed, sizeof(waThreadLed), NORMALPRIO+1, ThreadLed, NULL);
  chThdCreateStatic(waThreadHrm, sizeof(waThreadHrm), NORMALPRIO+1, ThreadHrm, NULL);

//  palSetPad(GPIO, PIN_LED);

  while(TRUE);

// If I use my ANT+ heart belt, the program run a few dozen of seconds
// (it's not fix, can be more or can be less) and then crash.
// In the Hardfault handler, I get 0xDEADBEEF in r2 register.
// It seems it's coming from the softdevice as I don't have this value in my code.
//
// Well I suppose, there is somewhere a bad initialisation that make sd_calls not working.
// Results, when code is running, there is probaly interrupt level priority issue between
// application and softdevice that make softdevice crashing...
//
// it's hard to debug as I don't have the softdevice code source...
//
// softdevice is flased by using command (in a shell) "make flash-softdevice" (see Makefile for details)


}

