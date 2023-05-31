#ifndef __FF_SL_PWM_HW_H
#define __FF_SL_PWM_HW_H

#ifdef __cplusplus
extern "C"
{
#endif

bool pwmPinsInit(void);
void set_pwm_output(unsigned long channel, bool isValueHigh);
void pwmPinsDeInit(void);

#ifdef __cplusplus
}
#endif

#endif
