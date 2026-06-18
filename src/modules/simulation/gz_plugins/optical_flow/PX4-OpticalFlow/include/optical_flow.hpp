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
*  optical_flow.h
*
*  Created on: Dec 13, 2016
*      Author: Christoph
*/

#pragma once

#include <stdint.h>
#include <iostream>
#include <cmath>

#define DEFAULT_OUTPUT_RATE 15
#define DEFAULT_IMAGE_WIDTH 64
#define DEFAULT_IMAGE_HEIGHT 64

class OpticalFlow
{

protected:
	//params which can be set
	int image_width;
	int image_height;
	float focal_length_x; //[pixel]
	float focal_length_y; //[pixel]
	int output_rate;
	float sum_flow_x;
	float sum_flow_y;
	int sum_flow_quality;
	int valid_frame_count;

	void initLimitRate();
	int limitRate(int flow_quality, const uint32_t frame_time_us, int *dt_us,
		      float *flow_x, float *flow_y);

public:

	virtual ~OpticalFlow(){};

	inline void setImageWidth(int img_width) { image_width = img_width; };
	inline void setImageHeight(int img_height) { image_height = img_height; };
	inline void setFocalLengthX(float f_lengh) { focal_length_x = f_lengh; };
	inline void setFocalLengthY(float f_lengh) { focal_length_y = f_lengh; };
	inline void setOutputRate(int out_rate) { output_rate = out_rate; };   //TODO check valid range 10-20?

	inline int getImageWidth() { return image_width; };
	inline int getImageHeight() { return image_height; };
	inline int getFocalLengthX() { return focal_length_x; };
	inline int getFocalLengthy() { return focal_length_y; };
	inline int getOutputRate() { return output_rate; };

	virtual int calcFlow(uint8_t *img_current, const uint32_t &img_time_us, int &dt_us, float &flow_x, float &flow_y) = 0;

};
