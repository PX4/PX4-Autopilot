/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *       Author: Ben Dyer <ben_dyer@mac.com>
 *               Pavel Kirienko <pavel.kirienko@zubax.com>
 *               David Sidrane <david_s5@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "stm32.h"
#include "nvic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TIMER_HRT_CYCLES_PER_US (STM32_HCLK_FREQUENCY/1000000)
#define TIMER_HRT_CYCLES_PER_MS (STM32_HCLK_FREQUENCY/1000)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

typedef enum {
        //! Specifies a one-shot timer. After notification timer is discarded.
        modeOneShot         = 1,
        //! Specifies a repeating timer.
        modeRepeating       = 2,
        //! Specifies a persisten start / stop timer.
        modeTimeout         = 3,

        modeStarted         = 0x40


} bl_timer_modes_t;


typedef uint8_t bl_timer_id;
typedef uint32_t time_ms_t;
typedef volatile time_ms_t* time_ref_t;

typedef uint32_t time_hrt_cycles_t;

typedef void (*bl_timer_ontimeout)(bl_timer_id id, void *context);

typedef struct {
  void *context;
  bl_timer_ontimeout cb;

} bl_timer_cb_t;


/****************************************************************************
 * Global Variables
 ****************************************************************************/

extern bl_timer_cb_t null_cb;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void timer_init(void);
bl_timer_id timer_allocate(bl_timer_modes_t mode, time_ms_t msfromnow, bl_timer_cb_t *fc);
void timer_free(bl_timer_id id);
void timer_start(bl_timer_id id);
void timer_restart(bl_timer_id id, time_ms_t ms);
void timer_stop(bl_timer_id id);
int timer_expired(bl_timer_id id);
time_ms_t timer_tic(void);

time_ref_t timer_ref(bl_timer_id id);
static inline int timer_ref_expired(time_ref_t ref)
{
  return *ref == 0;
}



static inline time_hrt_cycles_t timer_hrt_read(void)
{
  return getreg32(NVIC_SYSTICK_CURRENT);
}

static inline time_hrt_cycles_t timer_hrt_max(void)
{
  return getreg32(NVIC_SYSTICK_RELOAD) + 1;
}

static inline time_hrt_cycles_t timer_hrt_elapsed(time_hrt_cycles_t begin, time_hrt_cycles_t end)
{
  /* It is a down count from NVIC_SYSTICK_RELOAD */

  time_hrt_cycles_t elapsed = begin - end;
  time_hrt_cycles_t reload = timer_hrt_max();

  /* Did it wrap */
  if (elapsed > reload) {
      elapsed +=  reload;
  }

  return elapsed;
}


