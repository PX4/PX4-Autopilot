/*
 * agilex_types.h
 *
 * Created on: Jul 09, 2021 21:57
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef AGILEX_TYPES_H
#define AGILEX_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
	CONST_OFF = 0x00,
	CONST_ON = 0x01,
	BREATH = 0x02,
	CUSTOM = 0x03
} LightMode;

typedef struct {
	LightMode mode;
	uint8_t custom_value;
} LightOperation;

typedef enum {
	VehicleStateNormal = 0x00,
	VehicleStateEStop = 0x01,
	VehicleStateException = 0x02
} VehicleState;

typedef enum {
	//   CONTROL_MODE_STANDBY = 0x00,
	CONTROL_MODE_RC = 0x00,
	CONTROL_MODE_CAN = 0x01,
	CONTROL_MODE_UART = 0x02
} ControlMode;

typedef enum {
	//   CONTROL_MODE_STANDBY = 0x00,
	BRAKE_MODE_UNLOCK = 0x00,
	BRAKE_MODE_LOCK = 0x01
} BrakeMode;

typedef enum {
	RC_SWITCH_UP = 0,
	RC_SWITCH_MIDDLE,
	RC_SWITCH_DOWN
} RcSwitchState;

#ifdef __cplusplus
}
#endif

#endif /* AGILEX_TYPES_H */
