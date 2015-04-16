/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
 *           Thomas Gubler <thomasgubler@student.ethz.ch>
 *           Julian Oes <joes@student.ethz.ch>
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

/**
 * @file subsystem_info.h
 * Definition of the subsystem info topic.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 */

#ifndef TOPIC_SUBSYSTEM_INFO_H_
#define TOPIC_SUBSYSTEM_INFO_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

enum SUBSYSTEM_TYPE {
	SUBSYSTEM_TYPE_GYRO = 1,
	SUBSYSTEM_TYPE_ACC = 2,
	SUBSYSTEM_TYPE_MAG = 4,
	SUBSYSTEM_TYPE_ABSPRESSURE = 8,
	SUBSYSTEM_TYPE_DIFFPRESSURE = 16,
	SUBSYSTEM_TYPE_GPS = 32,
	SUBSYSTEM_TYPE_OPTICALFLOW = 64,
	SUBSYSTEM_TYPE_CVPOSITION = 128,
	SUBSYSTEM_TYPE_LASERPOSITION = 256,
	SUBSYSTEM_TYPE_EXTERNALGROUNDTRUTH = 512,
	SUBSYSTEM_TYPE_ANGULARRATECONTROL = 1024,
	SUBSYSTEM_TYPE_ATTITUDESTABILIZATION = 2048,
	SUBSYSTEM_TYPE_YAWPOSITION = 4096,
	SUBSYSTEM_TYPE_ALTITUDECONTROL = 16384,
	SUBSYSTEM_TYPE_POSITIONCONTROL = 32768,
	SUBSYSTEM_TYPE_MOTORCONTROL = 65536,
	SUBSYSTEM_TYPE_RANGEFINDER = 131072
};

/**
 * @addtogroup topics
 */

/**
 * State of individual sub systems
 */
struct subsystem_info_s {
	bool present;
	bool enabled;
	bool ok;

	enum SUBSYSTEM_TYPE subsystem_type;
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(subsystem_info);

#endif /* TOPIC_SUBSYSTEM_INFO_H_ */

