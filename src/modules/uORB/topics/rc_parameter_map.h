/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT ,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file rc_parameter_map.h
 * Maps RC channels to parameters
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#ifndef TOPIC_RC_PARAMETER_MAP_H
#define TOPIC_RC_PARAMETER_MAP_H

#include <stdint.h>
#include "../uORB.h"

#define RC_PARAM_MAP_NCHAN 3 // This limit is also hardcoded in the enum RC_CHANNELS_FUNCTION in rc_channels.h
#define PARAM_ID_LEN 16 // corresponds to MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN

/**
 * @addtogroup topics
 * @{
 */

struct rc_parameter_map_s {
	uint64_t timestamp;			/**< time at which the map was updated */

	bool valid[RC_PARAM_MAP_NCHAN];		/**< true for RC-Param channels which are mapped to a param */

	int param_index[RC_PARAM_MAP_NCHAN];	/**< corresponding param index, this
						  this field is ignored if set to -1, in this case param_id will
						  be used*/
	char param_id[RC_PARAM_MAP_NCHAN][PARAM_ID_LEN + 1];	/**< corresponding param id, null terminated */
	float scale[RC_PARAM_MAP_NCHAN];	/** scale to map the RC input [-1, 1] to a parameter value */
	float value0[RC_PARAM_MAP_NCHAN];	/** inital value around which the parameter value is changed */
	float value_min[RC_PARAM_MAP_NCHAN];	/** minimal parameter value */
	float value_max[RC_PARAM_MAP_NCHAN];	/** minimal parameter value */
};

/**
 * @}
 */

ORB_DECLARE(rc_parameter_map);

#endif
