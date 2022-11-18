
#ifndef _PARAM_CLIENT_H
#define _PARAM_CLIENT_H

#include "param.h"

void param_client_init();
void param_client_set(param_t param, const void *val);
void param_client_set_used(param_t param);

#endif
