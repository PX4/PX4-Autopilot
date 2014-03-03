#include <systemlib/param/param.h>

// 16 is max name length
PARAM_DEFINE_FLOAT(SEG_TH2V_P, 10.0f); // pitch to voltage
PARAM_DEFINE_FLOAT(SEG_TH2V_I, 0.0f); // pitch integral to voltage
PARAM_DEFINE_FLOAT(SEG_TH2V_I_MAX, 0.0f); // integral limiter
PARAM_DEFINE_FLOAT(SEG_Q2V, 1.0f); // pitch rate to voltage

