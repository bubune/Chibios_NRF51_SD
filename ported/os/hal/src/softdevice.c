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
 * @file    softdevice.c
 * @brief   Softdevice Driver code.
 *
 * @addtogroup SOFTDEVICE
 * @{
 */

#include "ch.h"
#include "hal.h"
#include "softdevice.h"

#if HAL_USE_SOFTDEVICE || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Softdevice Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void sddInit(void) {

  sdd_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p SoftdeviceDriver structure.
 *
 * @param[out] sddp     pointer to the @p SoftdeviceDriver object
 *
 * @init
 */
void sddObjectInit(SoftdeviceDriver *sddp) {

  sddp->state  = SDD_STOP;
  /* MailBoxes Init */
  chMBInit(&sddp->imb, sddp->iMbBuf, ANT_EVENT_SIZE);
  chMBInit(&sddp->omb, sddp->oMbBuf, ANT_EVENT_SIZE);
}

/**
 * @brief   Configures and activates the Softdevice peripheral.
 *
 * @param[in] sddp      pointer to the @p SoftdeviceDriver object
 * @param[in] config    pointer to the @p SoftdeviceConfig object
 *
 * @api
 */
uint32_t sddStart(SoftdeviceDriver *sddp, const SoftdeviceConfig *config) {

  uint32_t err=0;
  chDbgCheck((sddp != NULL) && (config != NULL), "sddStart");

  chSysLock();
  chDbgAssert((sddp->state == SDD_STOP) || (sddp->state == SDD_READY),
              "sddStart(), #1", "invalid state");
  err = sdd_lld_start(sddp, config);
  if(!err)
    sddp->state = SDD_READY;
  chSysUnlock();
  return err;
}

/**
 * @brief   Deactivates the Softdevice peripheral.
 *
 * @param[in] sddp      pointer to the @p SoftdeviceDriver object
 *
 * @api
 */
uint32_t sddStop(SoftdeviceDriver *sddp) {

  uint32_t err=0;
  chDbgCheck(sddp != NULL, "sddStop");

  chSysLock();
  chDbgAssert((sddp->state == SDD_STOP) || (sddp->state == SDD_READY),
              "sddStop(), #1", "invalid state");
  err=sdd_lld_stop(sddp);
  sddp->state = SDD_STOP;
  chSchRescheduleS();
  chSysUnlock();
  return err;
}


uint32_t sdd_open_channel(SoftdeviceDriver *sddp, antDeviceProfile *p)
{
  return sdd_lld_open_channel(sddp, p);
}


#endif /* HAL_USE_Softdevice */

/** @} */
