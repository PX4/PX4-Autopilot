/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

/*
*  flow_px4.hpp
*
*  Created on: Dec 13, 2016
*      Author: Christoph
*/

#pragma once

#include "optical_flow.hpp"
#include "px4flow.hpp"
#include <memory.h>

#define DEFAULT_SEARCH_SIZE 6
#define DEFAULT_FLOW_FEATURE_THRESHOLD 30
#define DEFAULT_FLOW_VALUE_THRESHOLD 3000

class OpticalFlowPX4 : public OpticalFlow
{

private:

	PX4Flow *px4_flow;
	bool initialized;
	uint8_t *img_old;

public:

	OpticalFlowPX4(float f_length_x, float f_length_y, int ouput_rate = DEFAULT_OUTPUT_RATE,
		       int img_width = DEFAULT_IMAGE_WIDTH, int img_height = DEFAULT_IMAGE_HEIGHT, int search_size = DEFAULT_SEARCH_SIZE,
		       int flow_feature_threshold = DEFAULT_FLOW_FEATURE_THRESHOLD,
		       int flow_value_threshold = DEFAULT_FLOW_VALUE_THRESHOLD);
	~OpticalFlowPX4();

	int calcFlow(uint8_t *img_current, const uint32_t &img_time_us, int &dt_us,
		     float &flow_x, float &flow_y);

};
