/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "uavcan_v1_node_client.h"

static int16_t gps_uorb_port_id = -1;
static int16_t gps_fix_port_id = -1;
static int16_t gps_aux_port_id = -1;

int32_t set_gps_uorb_port_id(uavcan_register_Value_1_0 *value)
{
	if (uavcan_register_Value_1_0_is_natural16_(value) && value->natural16.value.count == 1) { // Natural 16
		//TODO check validity
		printf("Master: set uORB portID to %i\n", value->natural16.value.elements[0]);
		gps_uorb_port_id = value->natural16.value.elements[0];
		return 0;
	}

	return -UAVCAN_REGISTER_ERROR_SERIALIZATION;
}

uavcan_register_Value_1_0 get_gps_uorb_port_id()
{
	uavcan_register_Value_1_0 value;

	value.natural16.value.elements[0] = gps_uorb_port_id;
	value.natural16.value.count = 1;
	value._tag_ = 10; // TODO does nunavut generate ENUM/defines for this??
	return value;
}

int32_t  set_gps_fix_port_id(uavcan_register_Value_1_0 *value)
{
	if (uavcan_register_Value_1_0_is_natural16_(value) && value->natural16.value.count == 1) { // Natural 16
		//TODO check validity
		printf("Master: set FIX portID to %i\n", value->natural16.value.elements[0]);
		gps_fix_port_id = value->natural16.value.elements[0];
		return 0;
	}

	return -UAVCAN_REGISTER_ERROR_SERIALIZATION;
}

uavcan_register_Value_1_0 get_gps_fix_port_id()
{
	uavcan_register_Value_1_0 value;

	value.natural16.value.elements[0] = gps_fix_port_id;
	value.natural16.value.count = 1;
	value._tag_ = 10; // TODO does nunavut generate ENUM/defines for this??
	return value;
}

int32_t set_gps_aux_port_id(uavcan_register_Value_1_0 *value)
{
	if (uavcan_register_Value_1_0_is_natural16_(value) && value->natural16.value.count == 1) { // Natural 16
		//TODO check validity
		printf("Master: set AUX portID to %i\n", value->natural16.value.elements[0]);
		gps_fix_port_id = value->natural16.value.elements[0];
		return 0;
	}

	return -UAVCAN_REGISTER_ERROR_SERIALIZATION;
}

uavcan_register_Value_1_0 get_gps_aux_port_id()
{
	uavcan_register_Value_1_0 value;

	value.natural16.value.elements[0] = gps_aux_port_id;
	value.natural16.value.count = 1;
	value._tag_ = 10; // TODO does nunavut generate ENUM/defines for this??
	return value;
}

extern "C" __EXPORT int uavcan_v1_node_client_main(int argc, char *argv[])
{
	return UavcanNodeClient::main(argc, argv);
}
