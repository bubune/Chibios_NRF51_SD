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
 * @file    NRF51/softdevice_lld.c
 * @brief   Softdevice Driver subsystem low level driver source template.
 *
 * @addtogroup SOFTDEVICE
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_SOFTDEVICE || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   SDD1 driver identifier.
 */
SoftdeviceDriver SDD1;

/*===========================================================================*/
/* Driver local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/** @brief Driver default configuration.*/
static const SoftdeviceConfig default_config = {
  NRF_CLOCK_LFCLKSRC_XTAL_50_PPM,
};


/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**@brief       Callback function for asserts in the SoftDevice.
 *
 * @details     A pointer to this function will be passed to the SoftDevice. This function will be
 *              called if an ASSERT statement in the SoftDevice fails.
 *
 * @param[in]   pc         The value of the program counter when the ASSERT call failed.
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void softdevice_assertion_handler(uint32_t pc, uint16_t line_num, const uint8_t * file_name)
{
//    UNUSED_PARAMETER(pc);
//    assert_nrf_callback(line_num, file_name);
}

/**
 * @brief   Common IRQ handler.
 * @note    Tries hard to clear all the pending interrupt sources, we don't
 *          want to go through the whole ISR and have another interrupt soon
 *
 *
 * @param[in] sddp       communication channel associated to the SOFTDEVICE
 */
static void serve_interruptSD(SoftdeviceDriver *sddp) {

  chSysLockFromIsr();

//  if( sddp->ib[sddp->indexIb].isFree == FREE)
//  {
    antEventMessage *ib = &sddp->ib[sddp->indexIb];
    sddp->sdderrcode = sd_ant_event_get(&ib->ant_channel, &ib->ant_event, ib->ant_event_message_buffer);
    chMBPostI(&sddp->imb, (uint32_t)ib);
    sddp->indexIb++;
//  }

  /* Circulat buf */
  if(sddp->indexIb++ >= ANT_EVENT_SIZE)
    sddp->indexIb = 0;

    chSysUnlockFromIsr();
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   SOFTDEVICE IRQ handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(SWI2_IRQHandler) {

  CH_IRQ_PROLOGUE();

  serve_interruptSD(&SDD1);

  CH_IRQ_EPILOGUE();
}



/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level Softdevice driver initialization.
 *
 * @notapi
 */
void sdd_lld_init(void) {

  uint8_t i;
  SoftdeviceDriver *sddp = &SDD1;

  /* Driver initialization.*/
  sddObjectInit(sddp);

  for(i=0; i<ANT_EVENT_SIZE; ++i)
  {
    sddp->ib[i].isFree = FREE;
    sddp->ob[i].isFree = FREE;
  }

  sddp->indexIb = 0;
}

/**
 * @brief   Configures and activates the Softdevice peripheral.
 *
 * @param[in] sddp      pointer to the @p SoftdeviceDriver object
 *
 * @notapi
 */
uint32_t sdd_lld_start(SoftdeviceDriver *sddp, const SoftdeviceConfig *config) {

  sddp->sdderrcode = 0;
  if (config == NULL)
    config = &default_config;

  if (sddp->state == SDD_STOP) {
    /* Enables the peripheral.*/
    if (&SDD1 == sddp)
    {
      // Initialize SoftDevice. Get returned error code. */
      port_enable();
      sddp->sdderrcode = sd_softdevice_enable(config->sd_clocksource, softdevice_assertion_handler);
      if(sddp->sdderrcode) return sddp->sdderrcode;

      // Set application IRQ to lowest priority.
      sddp->sdderrcode = sd_nvic_SetPriority(SWI2_IRQn, NRF51_SWI2_IRQ_PRIORITY);
      if(sddp->sdderrcode) return sddp->sdderrcode;

      // Enable application IRQ (triggered from protocol).
      sddp->sdderrcode = sd_nvic_EnableIRQ(SWI2_IRQn);
      nvicEnableVector(SWI2_IRQn, CORTEX_PRIORITY_MASK(3));
    }
  }
  return sddp->sdderrcode;
}

/**
 * @brief   Deactivates the Softdevice peripheral.
 *
 * @param[in] sddp      pointer to the @p SoftdeviceDriver object
 *
 * @notapi
 */
uint32_t sdd_lld_stop(SoftdeviceDriver *sddp) {

  sddp->sdderrcode=0;
  if (sddp->state == SDD_READY) {
    /* Resets the peripheral.*/

    /* Disables the peripheral.*/
    if (&SDD1 == sddp)
    {
      sddp->sdderrcode = sd_softdevice_disable();
      if(!sddp->sdderrcode) return sddp->sdderrcode;

      sddp->sdderrcode = sd_nvic_DisableIRQ(SWI2_IRQn);
    }
  }
  return sddp->sdderrcode;
}

#define HRMRX_ANT_CHANNEL 0

/**
 * @brief   Opne an ANT channel to a device.
 *
 * @param[in] sddp      pointer to the @p SoftdeviceDriver object
 *
 * @notapi
 */
uint32_t sdd_lld_open_channel(SoftdeviceDriver *sddp, antDeviceProfile *p)
{
  if (sddp->state == SDD_READY)
  {
    if (&SDD1 == sddp)
    {
      // Set Network Address.
      sddp->sdderrcode = sd_ant_network_address_set(p->ant_network_number,
                                                    p->ant_network_key);

      // Set Channel Number.
      sddp->sdderrcode = sd_ant_channel_assign(HRMRX_ANT_CHANNEL,
                                               p->ant_channel_type,
                                               p->ant_network_number,
                                               p->ant_assign);
      // Set Channel ID.
      sddp->sdderrcode = sd_ant_channel_id_set(HRMRX_ANT_CHANNEL,
                                               p->ant_device_number,
                                               p->ant_device_type,
                                               p->ant_trans_type);

      // Set Channel RF frequency.
      sddp->sdderrcode = sd_ant_channel_radio_freq_set(HRMRX_ANT_CHANNEL,
                                                       p->ant_rf_channel);

      // Set Channel period.
      sddp->sdderrcode = sd_ant_channel_period_set(HRMRX_ANT_CHANNEL,
                                                   p->ant_msg_period);

      // Open Channels.
      sddp->sdderrcode = sd_ant_channel_open(HRMRX_ANT_CHANNEL);
    }
  }
  return sddp->sdderrcode;
}
#endif /* HAL_USE_SOFTDEVICE */

/** @} */
