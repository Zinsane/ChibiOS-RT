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
 * @file    rng.h
 * @brief   RNG Driver macros and structures.
 *
 * @addtogroup CRYPTO
 * @{
 */

#ifndef _RNG_H_
#define _RNG_H_

#if HAL_USE_RNG || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    RNG status flags
 * @{
 */
/**
 * @brief   Seed error.
 */
#define RNG_SEED_ERROR           1
/**
 * @brief   Clock error.
 */
#define RNG_CLOCK_ERROR          2
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !CH_USE_SEMAPHORES || !CH_USE_EVENTS
#error "RNG driver requires CH_USE_SEMAPHORES and CH_USE_EVENTS"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  RNG_UNINIT = 0,                           /**< Not initialized.           */
  RNG_STOP = 1,                             /**< Stopped.                   */
  RNG_READY = 2,                            /**< Ready.                     */
} rngstate_t;

#include "rng_lld.h"

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Adds some flags to the RNG status mask.
 *
 * @param[in] rngp      pointer to the @p RNGDriver object
 * @param[in] mask      flags to be added to the status mask
 *
 * @iclass
 */
#define rngAddFlagsI(rngp, mask) ((rngp)->status |= (mask))
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void rngInit(void);
  void rngObjectInit(RNGDriver *rngp);
  void rngStart(RNGDriver *rngp);
  void rngStop(RNGDriver *rngp);
  void rngGenerate(RNGDriver *rngp, uint8_t* buf, uint32_t buf_len);
  uint8_t rngNext8(RNGDriver *rngp);
  uint16_t rngNext16(RNGDriver *rngp);
  uint32_t rngNext32(RNGDriver *rngp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_RNG */

#endif /* _RNG_H_ */

/** @} */
