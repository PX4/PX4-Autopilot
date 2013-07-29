#include <systemlib/param/param.h>

// 16 is max name length
PARAM_DEFINE_FLOAT(SEG_THETA2SPD_P, 10.0f); // pitch to speed
PARAM_DEFINE_FLOAT(SEG_THETA2SPD_I, 0.0f); // pitch integral to speed 
PARAM_DEFINE_FLOAT(SEG_THETA2SPD_I_MAX, 0.0f); // integral limiter
PARAM_DEFINE_FLOAT(SEG_Q2SPD, 1.0f); // pitch rate to speed

