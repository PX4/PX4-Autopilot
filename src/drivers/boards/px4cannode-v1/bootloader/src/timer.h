#pragma once


#include "stm32.h"
#include "nvic.h"

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

extern bl_timer_cb_t null_cb;


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


#define TIMER_HRT_CYCLES_PER_US (STM32_HCLK_FREQUENCY/1000000)
#define TIMER_HRT_CYCLES_PER_MS (STM32_HCLK_FREQUENCY/1000)

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


