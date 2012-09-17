#ifndef _CAL_TIMER_H
#define _CAL_TIMER_H

/* Enable or Disable a timer */
void hwtimer_enable(int num, int on);

/* Configure pre-scaler and if timer is auto-reload */
void hwtimer_config(int num, uint8_t pre_scale, int auto_reload);

/* Load a timer with the given value */
void hwtimer_load(int num, uint16_t val);

/* Read the current timer value */
uint16_t hwtimer_read(int num);

/* Enable or disable the watchdog */
void wdog_enable(int on);

/* Reset cpu using watchdog */
void wdog_reset(void);

/* power up the timers */
void hwtimer_init(void);

#endif /* _CAL_TIMER_H */
