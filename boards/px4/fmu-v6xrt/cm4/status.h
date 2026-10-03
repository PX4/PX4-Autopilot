#pragma once

#include <stdint.h>

#include "memmap.h"

/* CM4 status block, the top CM4_STATUS_SIZE bytes of shared memory:
 * [0] state, [1] fault IPSR, [2] kicks received, [3] messages sent.
 */
#define CM4_STATE_BOOT   0xC0DE0001u	/* reset handler entered */
#define CM4_STATE_MU     0xC0DE0002u	/* MU-B configured, waiting for DRIVER_OK */
#define CM4_STATE_READY  0xC0DE0003u	/* vrings attached, services announced */
#define CM4_STATE_BADRSC 0xBAD00000u	/* resource table failed validation, [1] = reason */
#define CM4_STATE_FAULT  0xDEAD0000u	/* exception taken, [1] = IPSR */

extern volatile uint32_t *const cm4_status;

/* Application name, NUL padded, the CM4_APP_NAME_LEN bytes below the
 * status block. Written at reset from CM4_APP_NAME.
 */
extern volatile char *const cm4_app_name;
