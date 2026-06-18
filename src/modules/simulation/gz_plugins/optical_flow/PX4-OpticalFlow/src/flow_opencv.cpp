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
*  flow_opencv.cpp
*
*  Created on: Dec 13, 2016
*      Author: Christoph
*/

#include "flow_opencv.hpp"

/****************************************************************************
 * OpenCV optical flow calculation
 ****************************************************************************/

OpticalFlowOpenCV::OpticalFlowOpenCV(float f_length_x, float f_length_y, int ouput_rate, int img_width, int img_height,
				     int num_feat,
				     float conf_multi) :
	num_features(num_feat),
	confidence_multiplier(conf_multi)
{
	setImageWidth(img_width);
	setImageHeight(img_height);
	setFocalLengthX(f_length_x);
	setFocalLengthY(f_length_y);
	setOutputRate(ouput_rate);

	set_camera_matrix = false;
	set_camera_distortion = false;
	camera_matrix.create(3, 3);
	camera_distortion.create(1, 5);

	initLimitRate();
}

OpticalFlowOpenCV::~OpticalFlowOpenCV(void)
{

}

void OpticalFlowOpenCV::setCameraMatrix(float focal_len_x, float focal_len_y,
					float principal_point_x, float principal_point_y)
{
	camera_matrix <<   focal_len_x, 0.0f, principal_point_x,
		      0.0f, focal_len_y, principal_point_y,
		      0.0f, 0.0f, 1.0f;

	set_camera_matrix = true;
}

void OpticalFlowOpenCV::setCameraDistortion(float k1, float k2, float k3, float p1, float p2)
{
	camera_distortion <<   k1, k2, p1, p2, k3;

	set_camera_distortion = true;
}

int OpticalFlowOpenCV::calcFlow(uint8_t *img_current, const uint32_t &img_time_us, int &dt_us,
				float &flow_x, float &flow_y)
{

	if (updateVector.empty()) {
		updateVector.resize(num_features, 2);
	}

	int meancount = 0;
	float pixel_flow_x_mean = 0.0;
	float pixel_flow_y_mean = 0.0;
	float pixel_flow_x_stddev = 0.0;
	float pixel_flow_y_stddev = 0.0;

	cv::Mat frame_gray = cv::Mat(image_height, image_width, CV_8UC1);
	frame_gray.data = (uchar *)img_current;

	trackFeatures(frame_gray, frame_gray, features_current, useless, updateVector, 0);

	if (set_camera_matrix && set_camera_distortion) {
		features_tmp = features_current;
		cv::undistortPoints(features_tmp, features_current, camera_matrix, camera_distortion);

		// cv::undistortPoints returns normalized coordinates... -> convert
		for (int i = 0; i < num_features; i++) {
			features_current[i].x = features_current[i].x * camera_matrix(0, 0) +
						camera_matrix(0, 2);
			features_current[i].y = features_current[i].y * camera_matrix(1, 1) +
						camera_matrix(1, 2);
		}
	}

	if (!features_current.empty() && !features_previous.empty()) {
		//calculate pixel flow
		for (int i = 0; i < updateVector.size(); i++) {
			//just use active features
			if (updateVector[i] == 1) {
				pixel_flow_x_mean += features_current[i].x - features_previous[i].x;
				pixel_flow_y_mean += features_current[i].y - features_previous[i].y;
				meancount++;
			}
		}

		//check if there are active features
		if (meancount) {
			pixel_flow_x_mean /= meancount;
			pixel_flow_y_mean /= meancount;

			//calculate variance
			for (int i = 0; i < updateVector.size(); i++) {
				if (updateVector[i] == 1) {
					pixel_flow_x_stddev += powf(features_current[i].x - features_previous[i].x - pixel_flow_x_mean, 2);
					pixel_flow_y_stddev += powf(features_current[i].y - features_previous[i].y - pixel_flow_y_mean, 2);
				}
			}

			//convert to standard deviation
			pixel_flow_x_stddev = sqrt(pixel_flow_x_stddev / meancount);
			pixel_flow_y_stddev = sqrt(pixel_flow_y_stddev / meancount);

			//recalculate pixel flow with 90% confidence interval
			float temp_flow_x_mean = 0.0;
			float temp_flow_y_mean = 0.0;
			meancount = 0;

			for (int i = 0; i < updateVector.size(); i++) {
				//check if active
				if (updateVector[i] == 1) {
					//flow of feature i
					float temp_flow_x = features_current[i].x - features_previous[i].x;
					float temp_flow_y = features_current[i].y - features_previous[i].y;
					//check if inside confidence interval

					if (fabs(temp_flow_x - pixel_flow_x_mean) < pixel_flow_x_stddev * confidence_multiplier &&
					    fabs(temp_flow_y - pixel_flow_y_mean) < pixel_flow_y_stddev * confidence_multiplier) {
						temp_flow_x_mean += temp_flow_x;
						temp_flow_y_mean += temp_flow_y;
						meancount++;

					} else {
						updateVector[i] = 0;
					}
				}
			}

			if (meancount) {
				//new mean
				pixel_flow_x_mean = temp_flow_x_mean / meancount;
				pixel_flow_y_mean = temp_flow_y_mean / meancount;
			}
		}
	}

	//remember features
	features_previous = features_current;

	//update feature status
	for (int i = 0; i < updateVector.size(); i++) {
		//new and now active
		if (updateVector[i] == 2) {
			updateVector[i] = 1;
		}

		//inactive
		if (updateVector[i] == 0) {
			updateVector[i] = 2;
		}
	}

	//output
	flow_x = pixel_flow_x_mean;
	flow_y = pixel_flow_y_mean;

	int flow_quality = round(255.0 * meancount / updateVector.size());

	flow_quality = limitRate(flow_quality, img_time_us, &dt_us, &flow_x, &flow_y);

	flow_x = atan2(flow_x, focal_length_x); //convert pixel flow to angular flow
	flow_y = atan2(flow_y, focal_length_y); //convert pixel flow to angular flow

	return flow_quality;
}
