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
*  px4flow.hpp
*
*  Created on: Dec 21, 2016
*      Author: Christoph
*/

#pragma once

#include <stdint.h>

class PX4Flow
{

private:
	//params which can be set
	uint32_t image_width;
	uint32_t search_size;
	uint32_t flow_feature_threshold;
	uint32_t flow_value_threshold;

	uint32_t __USAD8(uint32_t val1, uint32_t val2);
	uint32_t __USADA8(uint32_t val1, uint32_t val2, uint32_t val3);
	uint32_t __UHADD8(uint32_t val1, uint32_t val2);
	uint32_t compute_diff(uint8_t *image, uint16_t offX, uint16_t offY, uint16_t row_size);
	uint32_t compute_subpixel(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y,
				  uint16_t off2X, uint16_t off2Y, uint32_t *acc, uint16_t row_size);
	uint32_t compute_sad_8x8(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y,
				 uint16_t off2X, uint16_t off2Y, uint16_t row_size);


public:

	PX4Flow(uint32_t image_width_, uint32_t search_size_,
		uint32_t flow_feature_threshold_, uint32_t flow_value_threshold_);
	uint8_t compute_flow(uint8_t *image1, uint8_t *image2, float x_rate, float y_rate,
			     float z_rate, float *pixel_flow_x, float *pixel_flow_y);

};
