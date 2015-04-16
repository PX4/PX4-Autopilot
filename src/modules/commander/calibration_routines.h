/****************************************************************************
 *
 *   Copyright (c) 2012 PX4 Development Team. All rights reserved.
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
 * @file calibration_routines.h
 * Calibration routines definitions.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

/**
 * Least-squares fit of a sphere to a set of points.
 *
 * Fits a sphere to a set of points on the sphere surface.
 *
 * @param x point coordinates on the X axis
 * @param y point coordinates on the Y axis
 * @param z point coordinates on the Z axis
 * @param size number of points
 * @param max_iterations abort if maximum number of iterations have been reached. If unsure, set to 100.
 * @param delta abort if error is below delta. If unsure, set to 0 to run max_iterations times.
 * @param sphere_x coordinate of the sphere center on the X axis
 * @param sphere_y coordinate of the sphere center on the Y axis
 * @param sphere_z coordinate of the sphere center on the Z axis
 * @param sphere_radius sphere radius
 *
 * @return 0 on success, 1 on failure
 */
int sphere_fit_least_squares(const float x[], const float y[], const float z[],
			     unsigned int size, unsigned int max_iterations, float delta, float *sphere_x, float *sphere_y, float *sphere_z,
			     float *sphere_radius);

// FIXME: Change the name
static const unsigned max_accel_sens = 3;

// The order of these cannot change since the calibration calculations depend on them in this order
enum detect_orientation_return {
	DETECT_ORIENTATION_TAIL_DOWN,
	DETECT_ORIENTATION_NOSE_DOWN,
	DETECT_ORIENTATION_LEFT,
	DETECT_ORIENTATION_RIGHT,
	DETECT_ORIENTATION_UPSIDE_DOWN,
	DETECT_ORIENTATION_RIGHTSIDE_UP,
	DETECT_ORIENTATION_ERROR
};
static const unsigned detect_orientation_side_count = 6;

/**
 * Wait for vehicle to become still and detect it's orientation.
 *
 * @param mavlink_fd the MAVLink file descriptor to print output to
 * @param accel_sub Subscription to onboard accel
 *
 * @return detect_orientation)_return according to orientation when vehicle is still and ready for measurements,
 * DETECT_ORIENTATION_ERROR if vehicle is not still after 30s or orientation error is more than 5m/s^2
 */
enum detect_orientation_return detect_orientation(int mavlink_fd, int accel_sub);


/**
 * Returns the human readable string representation of the orientation
 *
 * @param orientation Orientation to return string for, "error" if buffer is too small
 *
 * @return str Returned orientation string
 */
const char* detect_orientation_str(enum detect_orientation_return orientation);

typedef int (*calibration_from_orientation_worker_t)(detect_orientation_return orientation, void* worker_data);

int calibrate_from_orientation(int mavlink_fd,
			       bool side_data_collected[detect_orientation_side_count],
			       calibration_from_orientation_worker_t calibration_worker,
			       void* worker_data);
