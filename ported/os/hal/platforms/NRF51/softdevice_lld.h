/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    NRF51/softdevice_lld.h
 * @brief   Softdevice Driver subsystem low level driver header template.
 *
 * @addtogroup Softdevice
 * @{
 */

#include "nrf_sdm.h"
#include "ant_interface.h"
#include "ant_parameters.h"

#ifndef _SOFTDEVICE_LLD_H_
#define _SOFTDEVICE_LLD_H_

#if HAL_USE_SOFTDEVICE || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */

/**
 * @brief   SOFTDEVICE driver enable switch.
 * @details If set to @p TRUE the support for UART0 is included.
 * @note    The default is @p TRUE .
 */
#if !defined(PLATFORM_SOFTDEVICE_USE_SOFTDEVICE1) || defined(__DOXYGEN__)
#define PLATFORM_SOFTDEVICE_USE_SOFTDEVICE1            TRUE
#endif

/**
 * @brief   SOFTDEVICE interrupt priority level setting.
 */
#if !defined(NRF51_SWI2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define NRF51_SWI2_IRQ_PRIORITY   3
#endif



/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a structure representing an Softdevice driver.
 */
typedef struct SoftdeviceDriver SoftdeviceDriver;

/**
 * @brief   NRF51 Softdevice Driver configuration structure.
 * @details An instance of this structure must be passed to @p sddStart()
 *          in order to configure and start a softdevice driver operations.
 */
typedef struct {
  /**
   * @brief Clock input for softdevice.
   */
  uint8_t                  sd_clocksource;

} SoftdeviceConfig;

/**
 * @brief   ANT event message structure
 * @details each ant event message is stored in this structure.
 *          Then, this structure is pull in the iqueue
 */

typedef struct {
  /**
   * @brief Ant Channel.
   */
  uint8_t                  ant_channel;
  /**
   * @brief Ant Event.
   */
  uint8_t                  ant_event;
  /**
   * @brief Ant Event.
   */
  uint8_t                  ant_event_message_buffer[MESG_BUFFER_SIZE];
  /**
   * @brief status variable.
   */
  uint8_t                  isFree;

} antEventMessage;

/**
 * @brief   ANT Device info structure
 * @details each ant+ profile need to have these values filled.
 */

typedef struct {
  /**
   * @brief Ant Channel Type.
   */
  uint8_t                  ant_channel_type;
  /**
   * @brief Ant Network Number.
   */
  uint8_t                  ant_network_number;
  /**
   * @brief Ant Network Key.
   */
  uint8_t                  ant_network_key[8];
  /**
   * @brief Ant device number.
   */
  uint8_t                  ant_device_number;
  /**
   * @brief Ant RF Channel
   */
  uint8_t                  ant_rf_channel;
  /**
   * @brief Ant RF Channel
   */
  uint8_t                  ant_trans_type;
  /**
   * @brief Ant RF Channel
   */
  uint8_t                  ant_device_type;
  /**
   * @brief Ant RF Channel
   */
  uint8_t                  ant_device_id;
  /**
   * @brief Ant RF Channel
   */
  uint16_t                  ant_msg_period;
  /**
   * @brief Ant assign
   */
  uint8_t                  ant_assign;;
} antDeviceProfile;


#define ANT_EVENT_SIZE     8
#define FREE               1
#define NOTFREE            0

/**
 * @brief   Structure representing an Softdevice driver.
 */
struct SoftdeviceDriver {
  /**
   * @brief Driver state.
   */
  sddstate_t                state;
  /**
   * @brief Softdevice return error code
   */
  uint32_t                  sdderrcode;
  /**
   * @brief Input Mailbox.
   */
   Mailbox                  imb;
  /**
   * @brief Input Mailbox buffer.
   */
   msg_t                    iMbBuf[ANT_EVENT_SIZE];
  /**
   * @brief Output Mailbox.
   */
   Mailbox                  omb;
  /**
   * @brief Output Mailbox buffer.
   */
   msg_t                   oMbBuf[ANT_EVENT_SIZE];
  /**
   * @brief Input circular buffer.
   */
  antEventMessage           ib[ANT_EVENT_SIZE];
  /**
   * @brief Index ib buffer.
   */
  uint32_t                  indexIb;
  /**
   * @brief Input circular buffer.
   */
  antEventMessage           ob[ANT_EVENT_SIZE];
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if PLATFORM_SOFTDEVICE_USE_SOFTDEVICE1 && !defined(__DOXYGEN__)
extern SoftdeviceDriver SDD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void sdd_lld_init(void);
  uint32_t sdd_lld_start(SoftdeviceDriver *sddp, const SoftdeviceConfig *config);
  uint32_t sdd_lld_stop(SoftdeviceDriver *sddp);
  uint32_t sdd_lld_open_channel(SoftdeviceDriver *sddp, antDeviceProfile *p);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SOFTDEVICE */

#endif /* _SOFTDEVICE_LLD_H_ */

/** @} */
