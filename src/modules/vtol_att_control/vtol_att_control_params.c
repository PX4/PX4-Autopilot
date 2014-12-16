#include <systemlib/param/param.h>

// number of engines
PARAM_DEFINE_INT32(VTOL_MOT_COUNT,0);
// idle pwm in multicopter mode
PARAM_DEFINE_INT32(IDLE_PWM_MC,900);
// min airspeed in multicopter mode
PARAM_DEFINE_FLOAT(VTOL_MC_AIRSPEED_MIN,2);
// max airspeed in multicopter mode
PARAM_DEFINE_FLOAT(VTOL_MC_AIRSPEED_MAX,30);
// trim airspeed in multicopter mode
PARAM_DEFINE_FLOAT(VTOL_MC_AIRSPEED_TRIM,10);

