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
*  flow_px4.cpp
*
*  Created on: Dec 13, 2016
*      Author: Christoph
*
****************************************************************************
* PX4Flow flow calculation
****************************************************************************/

#include "flow_px4.hpp"
#include <iostream>

OpticalFlowPX4::OpticalFlowPX4(float f_length_x, float f_length_y, int ouput_rate, int img_width, int img_height,
			       int search_size, int flow_feature_threshold, int flow_value_threshold)
{
	setImageWidth(img_width);
	setImageHeight(img_height);
	setFocalLengthX(f_length_x);
	setFocalLengthY(f_length_y);
	setOutputRate(ouput_rate);

	initLimitRate();

	//init the PX4Flow instance
	px4_flow = new PX4Flow(img_width, search_size, flow_feature_threshold, flow_value_threshold);
	initialized = false;
	img_old = new uint8_t[image_width * image_height];
}

OpticalFlowPX4::~OpticalFlowPX4(void)
{

}

int OpticalFlowPX4::calcFlow(uint8_t *img_current, const uint32_t &img_time_us, int &dt_us, float &flow_x,
			     float &flow_y)
{

	if (!initialized) {
		//first call of the function -> copy image for flow calculation
		memcpy(img_old, img_current, image_width * image_height * sizeof(uint8_t));
		initialized = true;
		return 0;
	}

	//not needed
	float x_gyro_rate = 0;
	float y_gyro_rate = 0;
	float z_gyro_rate = 0;

	int flow_quality = px4_flow->compute_flow(img_old, img_current,
			   x_gyro_rate, y_gyro_rate, z_gyro_rate, &flow_x, &flow_y);

	memcpy(img_old, img_current, image_width * image_height * sizeof(uint8_t));

	flow_quality = limitRate(flow_quality, img_time_us, &dt_us, &flow_x, &flow_y);

	flow_x = atan2(flow_x, focal_length_x); //convert pixel flow to angular flow
	flow_y = atan2(flow_y, focal_length_y); //convert pixel flow to angular flow

	return flow_quality;


}
