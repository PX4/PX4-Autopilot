
#ifndef _PARAM_SERVER_H
#define _PARAM_SERVER_H

#include "param.h"

void param_server_init();
void param_server_set(param_t param, const void *val);
void param_server_reset(param_t param);
void param_server_reset_all();

#endif
