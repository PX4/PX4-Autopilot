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
*  flow_opencv.hpp
*
*  Created on: Dec 13, 2016
*      Author: Christoph
*/

#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

#include "optical_flow.hpp"
#include "trackFeatures.h"

#define DEFAULT_NUMBER_OF_FEATURES 20
#define DEFAULT_CONFIDENCE_MULTIPLIER 1.645f //90% confidence interval

class OpticalFlowOpenCV : public OpticalFlow
{

private:
	//params which can be set
	int num_features;
	float confidence_multiplier;
	cv::Mat_<float> camera_matrix;
	cv::Mat_<float> camera_distortion;
	//general
	std::vector<int> updateVector;
	std::vector<cv::Point2f> features_current, features_previous, features_tmp, useless;
	bool set_camera_matrix;
	bool set_camera_distortion;

public:

	inline void setNumFeatures(int n_feat) { num_features = n_feat; };
	inline void setConfMultiplier(float conf_multi) { confidence_multiplier = conf_multi; };
	void setCameraMatrix(float focal_len_x, float focal_len_y, float principal_point_x, float principal_point_y);
	void setCameraDistortion(float k1, float k2, float k3, float p1 = 0.0f, float p2 = 0.0f);

	inline int getNumFeatures() { return num_features; };
	inline int getConfMultiplier() { return confidence_multiplier; };

	OpticalFlowOpenCV(float f_length_x, float f_length_y, int output_rate = DEFAULT_OUTPUT_RATE,
			  int img_width = DEFAULT_IMAGE_WIDTH, int img_height = DEFAULT_IMAGE_HEIGHT, int num_feat = DEFAULT_NUMBER_OF_FEATURES,
			  float conf_multi = DEFAULT_CONFIDENCE_MULTIPLIER);
	~OpticalFlowOpenCV();

	int calcFlow(uint8_t *img_current, const uint32_t &img_time_us, int &dt_us,
		     float &flow_x, float &flow_y);

};
