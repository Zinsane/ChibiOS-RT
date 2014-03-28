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
 * @file    STM32/rng_lld.h
 * @brief   STM32 RNG subsystem low level driver header.
 *
 * @addtogroup RNG
 * @{
 */

#ifndef _RNG_LLD_H_
#define _RNG_LLD_H_

#if HAL_USE_RNG || defined(__DOXYGEN__)

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
 * @brief   HASH/RNG interrupt priority level setting.
 */
#if !defined(STM32_RNG_RNG1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_HASH_RNG_IRQ_PRIORITY         14
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   RNG status flags.
 */
typedef uint32_t rngstatus_t;

/**
 * @brief   Structure representing an RNG driver.
 */
typedef struct {
  /**
   * @brief Driver state.
   */
  rngstate_t                state;
  /**
   * @brief Data ready semaphore.
   */
  Semaphore                 drdy_sem;
  /**
   * @brief Random number generation mutex.
   */
  Mutex gen_mtx;
  /**
   * @brief A new random value is available.
   */
  EventSource               drdy_event;
  /**
   * @brief A RNG bus error happened.
   */
  EventSource               error_event;
  /**
   * @brief Error flags set when an error event is broadcasted.
   */
  rngstatus_t               status;
  /**
   * @brief The last value returned by the RNG for testing integrity per FIPS PUB 140-2.
   */
  uint32_t last_val;
  /* End of the mandatory fields.*/
  /**
   * @brief Pointer to the RNG registers.
   */
  RNG_TypeDef               *rng;
  /**
   * @brief Pointer to the buffer to fill with random values.
   */
  uint8_t* fill_buf;
  /**
   * @brief Length of the buffer to fill with random values.
   */
  uint32_t fill_buf_len;
} RNGDriver;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

extern RNGDriver RNGD;

#ifdef __cplusplus
extern "C" {
#endif
  void rng_lld_init(void);
  void rng_lld_start(RNGDriver *rngp);
  void rng_lld_stop(RNGDriver *rngp);
  void rng_lld_generate(RNGDriver *rngp, uint8_t* buf, uint32_t buf_len);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_RNG */

#endif /* _RNG_LLD_H_ */

/** @} */
