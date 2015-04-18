
#include <nuttx/config.h>

#include "app_config.h"

#include <sys/types.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <arch/irq.h>

#include "bl_macros.h"
#include "timer.h"
#include "nvic.h"

#include <arch/board/board.h>



typedef enum {
        OneShot         = modeOneShot,
        Repeating       = modeRepeating,
        Timeout         = modeTimeout,

        modeMsk         = 0x3 ,
        running         = modeStarted,
        inuse           = 0x80,

} bl_timer_ctl_t;

typedef struct {
  bl_timer_cb_t         usr;
  time_ms_t             count;
  time_ms_t             reload;
  bl_timer_ctl_t        ctl;
} bl_timer_t;

bl_timer_t timers[OPT_BL_NUMBER_TIMERS];

bl_timer_cb_t null_cb =
    {
       0,
       0
    };


static time_ms_t sys_tic;

time_ms_t timer_tic(void)
{
  return sys_tic;
}

void sched_process_timer(void)
{
  PROBE(1,true);
  PROBE(1,false);

  sys_tic++;
  time_ms_t ms_elapsed = (CONFIG_USEC_PER_TICK/1000);
  bl_timer_id t;
  for( t =  arraySize(timers)-1; (int8_t) t >= 0; t--) {
      if ((timers[t].ctl & (inuse|running)) == (inuse|running)) {
          if (timers[t].count != 0 && timers[t].count > ms_elapsed) {
              timers[t].count -= ms_elapsed;
          } else {
                  timers[t].count = 0;

                  switch(timers[t].ctl & ~(inuse|running)) {

                    case Timeout:
                       break;
                    case OneShot: {
                        bl_timer_cb_t user = timers[t].usr;
                        memset(&timers[t], 0, sizeof(timers[t]));
                        if (user.cb) {
                            user.cb(t, user.context);
                        }
                    }
                      break;
                    case Repeating:
                      timers[t].count = timers[t].reload;
                      if (timers[t].usr.cb) {
                          timers[t].usr.cb(t, timers[t].usr.context);
                      }
                      break;
                    default:
                       break;
                  }
              }
          }
      }
}

bl_timer_id timer_allocate(bl_timer_modes_t mode, time_ms_t msfromnow, bl_timer_cb_t *fc)
{
  bl_timer_id t;
  irqstate_t s = irqsave();

  for(t = arraySize(timers)-1; (int8_t)t >= 0; t--) {

      if ((timers[t].ctl & inuse) == 0 ) {

          timers[t].reload = msfromnow;
          timers[t].count = msfromnow;
          timers[t].usr = fc ? *fc : null_cb;
          timers[t].ctl = (mode & (modeMsk|running)) | (inuse);
          break;
      }
  }

  irqrestore(s);
  return t;
}


void timer_free(bl_timer_id id)
{
  DEBUGASSERT(id>=0 && id < arraySize(timers));
  irqstate_t s = irqsave();
  memset(&timers[id], 0, sizeof(timers[id]));
  irqrestore(s);
}

void timer_start(bl_timer_id id)
{
  DEBUGASSERT(id>=0 && id < arraySize(timers) && (timers[id].ctl & inuse));
  irqstate_t s = irqsave();
  timers[id].count = timers[id].reload;
  timers[id].ctl |= running;
  irqrestore(s);

}
void timer_stop(bl_timer_id id)
{
  DEBUGASSERT(id>=0 && id < arraySize(timers) && (timers[id].ctl & inuse));
  irqstate_t s = irqsave();
  timers[id].ctl &= ~running;
  irqrestore(s);

}

int timer_expired(bl_timer_id id)
{
  DEBUGASSERT(id>=0 && id < arraySize(timers) && (timers[id].ctl & inuse));
  irqstate_t s = irqsave();
  int rv = ((timers[id].ctl & running) && timers[id].count == 0);
  irqrestore(s);
  return rv;
}

time_ref_t timer_ref(bl_timer_id id)
{
  DEBUGASSERT(id>=0 && id < arraySize(timers) && (timers[id].ctl & inuse));
  return (time_ref_t) &timers[id].count;
}

void timer_restart(bl_timer_id id, time_ms_t ms)
{
  DEBUGASSERT(id>=0 && id < arraySize(timers) && (timers[id].ctl & inuse));
  irqstate_t s = irqsave();
  timers[id].count = timers[id].reload = ms;
  timers[id].ctl |= running;
  irqrestore(s);
}

__attribute__ ((visibility ("default")))
void timer_init(void)
{
/*  PROBE_INIT(7);
  PROBE(1,true);
  PROBE(2,true);
  PROBE(3,true);
  PROBE(1,false);
  PROBE(2,false);
  PROBE(3,false);
//  *((uint32_t *)0x40011010) = 0x100; // PROBE(3,true);
//  *((uint32_t *)0x40011014) = 0x100; // PROBE(3,false);
*/
  memset(timers,0,sizeof(timers));
}

