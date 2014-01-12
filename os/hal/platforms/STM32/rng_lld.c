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
 * @file    STM32/rng_lld.c
 * @brief   STM32 RNG subsystem low level driver source.
 *
 * @addtogroup CRYPTO
 * @{
 */

#include "ch.h"
#include "hal.h"

#include <string.h>

#if HAL_USE_RNG || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief RNG driver identifier.*/
RNGDriver RNGD;

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   HASH/RNG interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(HASH_RNG_IRQHandler) {

  uint32_t rng_sr;

  CH_IRQ_PROLOGUE();

  rng_sr = RNG->SR;

  if (rng_sr & RNG_SR_SEIS) {
    RNG->SR = ~RNG_SR_SEIS;
    rngAddFlagsI(&RNGD, RNG_SEED_ERROR);
    chSysLockFromIsr();
    chEvtBroadcastI(&RNGD.error_event);
    chSysUnlockFromIsr();
  }

  if (rng_sr & RNG_SR_CEIS) {
    RNG->SR = ~RNG_SR_CEIS;
    rngAddFlagsI(&RNGD, RNG_CLOCK_ERROR);
    chSysLockFromIsr();
    chEvtBroadcastI(&RNGD.error_event);
    chSysUnlockFromIsr();
  }

  if (rng_sr & RNG_SR_DRDY) {
    uint32_t val = RNG->DR;
    if (val != RNGD.last_val) {
      if (RNGD.fill_buf_len > 0) {
        int copied = RNGD.fill_buf_len < sizeof(uint32_t) ? RNGD.fill_buf_len : sizeof(uint32_t);
        memcpy(RNGD.fill_buf, &val, copied);
        RNGD.fill_buf += copied;
        RNGD.fill_buf_len -= copied;
      }

      if (RNGD.fill_buf_len <= 0) {
        /* Our work here is done. Disable interrupt. */
        RNG->CR &= ~RNG_CR_IE;

        /* Broadcast data ready event and release waiting thread */
        chSysLockFromIsr();
        chSemSignalI(&RNGD.drdy_sem);
        chEvtBroadcastI(&RNGD.drdy_event);
        chSysUnlockFromIsr();
      }

      RNGD.last_val = val;
    }
  }

  CH_IRQ_EPILOGUE();
}


/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level RNG driver initialization.
 *
 * @notapi
 */
void rng_lld_init(void) {

  /* Driver initialization.*/
  rngObjectInit(&RNGD);
  RNGD.rng = RNG;
}

/**
 * @brief   Configures and activates the RNG peripheral.
 *
 * @param[in] rngp      pointer to the @p RNGDriver object
 *
 * @notapi
 */
void rng_lld_start(RNGDriver *rngp) {

  /* Clock activation.*/
  nvicEnableVector(HASH_RNG_IRQn,
      CORTEX_PRIORITY_MASK(STM32_HASH_RNG_IRQ_PRIORITY));
  rccEnableRNG(FALSE);

  rngp->rng->CR |= RNG_CR_RNGEN;
}

/**
 * @brief   Deactivates the RNG peripheral.
 *
 * @param[in] rngp      pointer to the @p RNGDriver object
 *
 * @notapi
 */
void rng_lld_stop(RNGDriver *rngp) {

  /* If in ready state then disables the RNG peripheral.*/
  if (rngp->state == RNG_READY) {
    nvicDisableVector(HASH_RNG_IRQn);
    rccDisableRNG(FALSE);

    rngp->rng->CR &= ~RNG_CR_IE;
    rngp->rng->CR &= ~RNG_CR_RNGEN;
  }
}

/**
 * @brief
 *
 * @param[in] rngp      pointer to the @p RNGDriver object
 *
 * @notapi
 */
void rng_lld_generate(RNGDriver *rngp, uint8_t* buf, uint32_t buf_len) {
  chMtxLock(&rngp->gen_mtx);

  rngp->fill_buf = buf;
  rngp->fill_buf_len = buf_len;

  /* Enable the interrupt and wait for the data to be generated */
  rngp->rng->CR |= RNG_CR_IE;
  chSemWait(&rngp->drdy_sem);

  chMtxUnlock();
}

#endif /* HAL_USE_RNG */

/** @} */
