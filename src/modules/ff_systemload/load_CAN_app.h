#ifndef __FF_SL_LOAD_CAN_APP_H
#define __FF_SL_LOAD_CAN_APP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "load_CAN.h"
#include <stdarg.h>

#define DEBUG_OUTPUT debugPrint
#define DELAY_MS(x) usleep(x*1000)

#define DISABLE_REMOTE_KEYLOAD false

#define ESC_QTY 4

void can_module_set_all_addressing_lines(int state);
void can_module_set_addressing_line(can_module_s *module, int state);

bool assign_all_esc_boom_ids(void);

bool init_modules_app(void);
bool initAllCanPorts(void);
bool deInitAllCanPorts(void);
bool post_finish_load_callback(void);

void handle_can_module_error(can_module_s *module, enum can_module_error_code errorCode);

bool print_loaded_versions(void);

extern can_module_s modules[];

extern const size_t module_total;
extern can_handle_s *used_handles[];
extern uint8_t loaded_types[];

typedef enum {
	CAN_LOAD_IDLE,
	CAN_LOAD_IN_PROCESS,
	CAN_LOAD_FAILURE,
	CAN_LOAD_SUCCESS,
} can_load_status_e;

extern can_load_status_e can_load_status;

void debugPrint(char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif
