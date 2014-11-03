#include <systemlib/param/param.h>
#include <math.h>

// 16 is max name length
PARAM_DEFINE_FLOAT(INVP_TH2V, 10.0f); // pitch error to voltage
PARAM_DEFINE_FLOAT(INVP_Q2V, 1.0f); // pitch rate error to voltage

PARAM_DEFINE_FLOAT(INVP_TH_LIM_MAX, 5*M_PI/180); // pitch limit
PARAM_DEFINE_FLOAT(INVP_TH_STOP, 10*M_PI/180); // turn off motors when over

// system id
PARAM_DEFINE_FLOAT(INVP_SYSID_AMP, 0.5f); // wave amplitude, deg pitch
PARAM_DEFINE_FLOAT(INVP_SYSID_FREQ, 0.1f); // wave frquency, Hz
