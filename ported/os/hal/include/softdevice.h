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
 * @file    softdevice.h
 * @brief   Softdevice Driver macros and structures.
 *
 * @addtogroup SOFTDEVICE
 * @{
 */

#ifndef _SOFTDEVICE_H_
#define _SOFTDEVICE_H_


#if HAL_USE_SOFTDEVICE || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  SDD_UNINIT = 0,                   /**< Not initialized.                   */
  SDD_STOP = 1,                     /**< Stopped.                           */
  SDD_READY = 2,                    /**< Ready.                             */
} sddstate_t;

/**
 * @brief   Type of a structure representing a Softdevice driver.
 */
typedef struct SoftdeviceDriver SoftdeviceDriver;

#include "softdevice_lld.h"


/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void sddInit(void);
  void sddObjectInit(SoftdeviceDriver *sddp);
  uint32_t sddStart(SoftdeviceDriver *sddp, const SoftdeviceConfig *config);
  uint32_t sddStop(SoftdeviceDriver *sddp);
  uint32_t sdd_open_channel(SoftdeviceDriver *sddp, antDeviceProfile *p);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SOFTDEVICE */

#endif /* _SOFTDEVICE_H_ */

/** @} */
