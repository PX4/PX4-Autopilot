/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file mag_calibration.cpp
 *
 * Magnetometer calibration routine
 */

#include "mag_calibration.h"
#include "commander_helper.h"
#include "calibration_routines.h"
#include "calibration_messages.h"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_tone_alarm.h>
#include <matrix/math.hpp>
#include <lib/conversion/rotation.h>
#include <lib/systemlib/mavlink_log.h>
#include <lib/parameters/param.h>
#include <lib/systemlib/err.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_gyro.h>

using namespace matrix;

static constexpr char sensor_name[] = "mag";
static constexpr unsigned MAX_MAGS = 4;
static constexpr float mag_sphere_radius = 0.2f;
static unsigned int calibration_sides = 6;			///< The total number of sides
static constexpr unsigned int calibration_total_points = 240;		///< The total points per magnetometer
static constexpr unsigned int calibraton_duration_seconds = 42; 	///< The total duration the routine is allowed to take

static constexpr float MAG_MAX_OFFSET_LEN =
	1.3f;	///< The maximum measurement range is ~1.9 Ga, the earth field is ~0.6 Ga, so an offset larger than ~1.3 Ga means the mag will saturate in some directions.

int32_t	device_ids[MAX_MAGS];
bool internal[MAX_MAGS];
int device_prio_max = 0;
int32_t device_id_primary = 0;
static unsigned _last_mag_progress = 0;

calibrate_return mag_calibrate_all(orb_advert_t *mavlink_log_pub, int32_t cal_mask);

/// Data passed to calibration worker routine
typedef struct  {
	orb_advert_t	*mavlink_log_pub;
	unsigned	done_count;
	int		sub_mag[MAX_MAGS];
	unsigned int	calibration_points_perside;
	unsigned int	calibration_interval_perside_seconds;
	uint64_t	calibration_interval_perside_useconds;
	unsigned int	calibration_counter_total[MAX_MAGS];
	bool		side_data_collected[detect_orientation_side_count];
	float		*x[MAX_MAGS];
	float		*y[MAX_MAGS];
	float		*z[MAX_MAGS];
} mag_worker_data_t;


int do_mag_calibration(orb_advert_t *mavlink_log_pub)
{
	calibration_log_info(mavlink_log_pub, CAL_QGC_STARTED_MSG, sensor_name);

	struct mag_calibration_s mscale_null;
	mscale_null.x_offset = 0.0f;
	mscale_null.x_scale = 1.0f;
	mscale_null.y_offset = 0.0f;
	mscale_null.y_scale = 1.0f;
	mscale_null.z_offset = 0.0f;
	mscale_null.z_scale = 1.0f;

	int result = PX4_OK;

	// Determine which mags are available and reset each

	char str[30];

	// reset the learned EKF mag in-flight bias offsets which have been learned for the previous
	//  sensor calibration and will be invalidated by a new sensor calibration
	(void)sprintf(str, "EKF2_MAGBIAS_X");
	result = param_set_no_notification(param_find(str), &mscale_null.x_offset);

	if (result != PX4_OK) {
		PX4_ERR("unable to reset %s", str);
	}

	(void)sprintf(str, "EKF2_MAGBIAS_Y");
	result = param_set_no_notification(param_find(str), &mscale_null.y_offset);

	if (result != PX4_OK) {
		PX4_ERR("unable to reset %s", str);
	}

	(void)sprintf(str, "EKF2_MAGBIAS_Z");
	result = param_set_no_notification(param_find(str), &mscale_null.z_offset);

	if (result != PX4_OK) {
		PX4_ERR("unable to reset %s", str);
	}

	for (size_t i = 0; i < MAX_MAGS; i++) {
		device_ids[i] = 0; // signals no mag
	}

	_last_mag_progress = 0;

	// Collect: As defined by configuration
	// start with a full mask, all six bits set
	int32_t cal_mask = (1 << 6) - 1;
	param_get(param_find("CAL_MAG_SIDES"), &cal_mask);

	// keep and update the existing calibration when we are not doing a full 6-axis calibration
	const bool append_to_existing_calibration = cal_mask < ((1 << 6) - 1);
	(void)append_to_existing_calibration;

	for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
#if 1 // TODO: replace all IOCTL usage
		// Reset mag id to mag not available
		(void)sprintf(str, "CAL_MAG%u_ID", cur_mag);
		result = param_set_no_notification(param_find(str), &(device_ids[cur_mag]));

		if (result != PX4_OK) {
			calibration_log_info(mavlink_log_pub, "[cal] Unable to reset CAL_MAG%u_ID", cur_mag);
			break;
		}

#else
		(void)sprintf(str, "CAL_MAG%u_XOFF", cur_mag);
		result = param_set_no_notification(param_find(str), &mscale_null.x_offset);

		if (result != PX4_OK) {
			PX4_ERR("unable to reset %s", str);
		}

		(void)sprintf(str, "CAL_MAG%u_YOFF", cur_mag);
		result = param_set_no_notification(param_find(str), &mscale_null.y_offset);

		if (result != PX4_OK) {
			PX4_ERR("unable to reset %s", str);
		}

		(void)sprintf(str, "CAL_MAG%u_ZOFF", cur_mag);
		result = param_set_no_notification(param_find(str), &mscale_null.z_offset);

		if (result != PX4_OK) {
			PX4_ERR("unable to reset %s", str);
		}

		(void)sprintf(str, "CAL_MAG%u_XSCALE", cur_mag);
		result = param_set_no_notification(param_find(str), &mscale_null.x_scale);

		if (result != PX4_OK) {
			PX4_ERR("unable to reset %s", str);
		}

		(void)sprintf(str, "CAL_MAG%u_YSCALE", cur_mag);
		result = param_set_no_notification(param_find(str), &mscale_null.y_scale);

		if (result != PX4_OK) {
			PX4_ERR("unable to reset %s", str);
		}

		(void)sprintf(str, "CAL_MAG%u_ZSCALE", cur_mag);
		result = param_set_no_notification(param_find(str), &mscale_null.z_scale);

		if (result != PX4_OK) {
			PX4_ERR("unable to reset %s", str);
		}

#endif

		param_notify_changes();

#if 1 // TODO: replace all IOCTL usage
		// Attempt to open mag
		(void)sprintf(str, "%s%u", MAG_BASE_DEVICE_PATH, cur_mag);
		int fd = px4_open(str, O_RDONLY);

		if (fd < 0) {
			continue;
		}

		// Get device id for this mag
		device_ids[cur_mag] = px4_ioctl(fd, DEVIOCGDEVICEID, 0);
		internal[cur_mag] = (px4_ioctl(fd, MAGIOCGEXTERNAL, 0) <= 0);

		if (!append_to_existing_calibration) {
			// Reset mag scale & offset
			result = px4_ioctl(fd, MAGIOCSSCALE, (long unsigned int)&mscale_null);

			if (result != PX4_OK) {
				calibration_log_critical(mavlink_log_pub, CAL_ERROR_RESET_CAL_MSG, cur_mag);
			}
		}


		px4_close(fd);
#endif
	}

	// Calibrate all mags at the same time
	if (result == PX4_OK) {
		switch (mag_calibrate_all(mavlink_log_pub, cal_mask)) {
		case calibrate_return_cancelled:
			// Cancel message already displayed, we're done here
			result = PX4_ERROR;
			break;

		case calibrate_return_ok:
			/* if there is a any preflight-check system response, let the barrage of messages through */
			px4_usleep(200000);

			calibration_log_info(mavlink_log_pub, CAL_QGC_PROGRESS_MSG, 100);
			px4_usleep(20000);
			calibration_log_info(mavlink_log_pub, CAL_QGC_DONE_MSG, sensor_name);
			px4_usleep(20000);
			break;

		default:
			calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, sensor_name);
			px4_usleep(20000);
			break;
		}
	}

	/* give this message enough time to propagate */
	px4_usleep(600000);

	return result;
}

static bool reject_sample(float sx, float sy, float sz, float x[], float y[], float z[], unsigned count,
			  unsigned max_count)
{
	float min_sample_dist = fabsf(5.4f * mag_sphere_radius / sqrtf(max_count)) / 3.0f;

	for (size_t i = 0; i < count; i++) {
		float dx = sx - x[i];
		float dy = sy - y[i];
		float dz = sz - z[i];
		float dist = sqrtf(dx * dx + dy * dy + dz * dz);

		if (dist < min_sample_dist) {
			return true;
		}
	}

	return false;
}

static unsigned progress_percentage(mag_worker_data_t *worker_data)
{
	return 100 * ((float)worker_data->done_count) / calibration_sides;
}

// Returns calibrate_return_error if any parameter is not finite
// Logs if parameters are out of range
static calibrate_return check_calibration_result(float offset_x, float offset_y, float offset_z,
		float sphere_radius,
		float diag_x, float diag_y, float diag_z,
		float offdiag_x, float offdiag_y, float offdiag_z,
		orb_advert_t *mavlink_log_pub, size_t cur_mag)
{
	float must_be_finite[] = {offset_x, offset_y, offset_z,
				  sphere_radius,
				  diag_x, diag_y, diag_z,
				  offdiag_x, offdiag_y, offdiag_z
				 };

	float should_be_not_huge[] = {offset_x, offset_y, offset_z};
	float should_be_positive[] = {sphere_radius, diag_x, diag_y, diag_z};

	// Make sure every parameter is finite
	const int num_finite = sizeof(must_be_finite) / sizeof(*must_be_finite);

	for (unsigned i = 0; i < num_finite; ++i) {
		if (!PX4_ISFINITE(must_be_finite[i])) {
			calibration_log_emergency(mavlink_log_pub,
						  "ERROR: Retry calibration (sphere NaN, #%zu)", cur_mag);
			return calibrate_return_error;
		}
	}

	// Notify if offsets are too large
	const int num_not_huge = sizeof(should_be_not_huge) / sizeof(*should_be_not_huge);

	for (unsigned i = 0; i < num_not_huge; ++i) {
		if (fabsf(should_be_not_huge[i]) > MAG_MAX_OFFSET_LEN) {
			calibration_log_critical(mavlink_log_pub, "Warning: %s mag with large offsets",
						 (internal[cur_mag]) ? "autopilot, internal" : "GPS unit, external");
			break;
		}
	}

	// Notify if a parameter which should be positive is non-positive
	const int num_positive = sizeof(should_be_positive) / sizeof(*should_be_positive);

	for (unsigned i = 0; i < num_positive; ++i) {
		if (should_be_positive[i] <= 0.0f) {
			calibration_log_critical(mavlink_log_pub, "Warning: %s mag with non-positive scale",
						 (internal[cur_mag]) ? "autopilot, internal" : "GPS unit, external");
			break;
		}
	}

	return calibrate_return_ok;
}

static calibrate_return mag_calibration_worker(detect_orientation_return orientation, int cancel_sub, void *data)
{
	calibrate_return result = calibrate_return_ok;

	unsigned int calibration_counter_side;

	mag_worker_data_t *worker_data = (mag_worker_data_t *)(data);

	// notify user to start rotating
	set_tune(TONE_SINGLE_BEEP_TUNE);

	calibration_log_info(worker_data->mavlink_log_pub, "[cal] Rotate vehicle around the detected orientation");
	calibration_log_info(worker_data->mavlink_log_pub, "[cal] Continue rotation for %s %u s",
			     detect_orientation_str(orientation), worker_data->calibration_interval_perside_seconds);

	/*
	 * Detect if the system is rotating.
	 *
	 * We're detecting this as a general rotation on any axis, not necessary on the one we
	 * asked the user for. This is because we really just need two roughly orthogonal axes
	 * for a good result, so we're not constraining the user more than we have to.
	 */

	hrt_abstime detection_deadline = hrt_absolute_time() + worker_data->calibration_interval_perside_useconds * 5;
	hrt_abstime last_gyro = 0;
	float gyro_x_integral = 0.0f;
	float gyro_y_integral = 0.0f;
	float gyro_z_integral = 0.0f;

	const float gyro_int_thresh_rad = 0.5f;

	int sub_gyro = orb_subscribe(ORB_ID(sensor_gyro));

	while (fabsf(gyro_x_integral) < gyro_int_thresh_rad &&
	       fabsf(gyro_y_integral) < gyro_int_thresh_rad &&
	       fabsf(gyro_z_integral) < gyro_int_thresh_rad) {

		/* abort on request */
		if (calibrate_cancel_check(worker_data->mavlink_log_pub, cancel_sub)) {
			result = calibrate_return_cancelled;
			px4_close(sub_gyro);
			return result;
		}

		/* abort with timeout */
		if (hrt_absolute_time() > detection_deadline) {
			result = calibrate_return_error;
			warnx("int: %8.4f, %8.4f, %8.4f", (double)gyro_x_integral, (double)gyro_y_integral, (double)gyro_z_integral);
			calibration_log_critical(worker_data->mavlink_log_pub, "Failed: This calibration requires rotation.");
			break;
		}

		/* Wait clocking for new data on all gyro */
		px4_pollfd_struct_t fds[1];
		fds[0].fd = sub_gyro;
		fds[0].events = POLLIN;
		size_t fd_count = 1;

		int poll_ret = px4_poll(fds, fd_count, 1000);

		if (poll_ret > 0) {
			sensor_gyro_s gyro{};
			orb_copy(ORB_ID(sensor_gyro), sub_gyro, &gyro);

			/* ensure we have a valid first timestamp */
			if (last_gyro > 0) {

				/* integrate */
				float delta_t = (gyro.timestamp - last_gyro) / 1e6f;
				gyro_x_integral += gyro.x * delta_t;
				gyro_y_integral += gyro.y * delta_t;
				gyro_z_integral += gyro.z * delta_t;
			}

			last_gyro = gyro.timestamp;
		}
	}

	px4_close(sub_gyro);

	uint64_t calibration_deadline = hrt_absolute_time() + worker_data->calibration_interval_perside_useconds;
	unsigned poll_errcount = 0;

	calibration_counter_side = 0;

	while (hrt_absolute_time() < calibration_deadline &&
	       calibration_counter_side < worker_data->calibration_points_perside) {

		if (calibrate_cancel_check(worker_data->mavlink_log_pub, cancel_sub)) {
			result = calibrate_return_cancelled;
			break;
		}

		// Wait clocking for new data on all mags
		px4_pollfd_struct_t fds[MAX_MAGS];
		size_t fd_count = 0;

		for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
			if (worker_data->sub_mag[cur_mag] >= 0 && device_ids[cur_mag] != 0) {
				fds[fd_count].fd = worker_data->sub_mag[cur_mag];
				fds[fd_count].events = POLLIN;
				fd_count++;
			}
		}

		int poll_ret = px4_poll(fds, fd_count, 1000);

		if (poll_ret > 0) {

			bool rejected = false;

			Vector3f new_samples[MAX_MAGS] {};

			for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {

				if (worker_data->sub_mag[cur_mag] >= 0) {
					sensor_mag_s mag{};

					orb_copy(ORB_ID(sensor_mag), worker_data->sub_mag[cur_mag], &mag);

					// Check if this measurement is good to go in
					bool reject = reject_sample(mag.x, mag.y, mag.z,
								    worker_data->x[cur_mag], worker_data->y[cur_mag], worker_data->z[cur_mag],
								    worker_data->calibration_counter_total[cur_mag],
								    calibration_sides * worker_data->calibration_points_perside);

					if (reject) {
						rejected = true;
						PX4_DEBUG("Mag: %d rejected X: %.3f Y: %.3f Z: %.3f", cur_mag, (double)mag.x, (double)mag.y, (double)mag.z);

					} else {
						new_samples[cur_mag](0) = mag.x;
						new_samples[cur_mag](1) = mag.y;
						new_samples[cur_mag](2) = mag.z;
					}
				}
			}

			// Keep calibration of all mags in lockstep
			if (!rejected) {
				for (size_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
					if (worker_data->sub_mag[cur_mag] >= 0) {
						worker_data->x[cur_mag][worker_data->calibration_counter_total[cur_mag]] = new_samples[cur_mag](0);
						worker_data->y[cur_mag][worker_data->calibration_counter_total[cur_mag]] = new_samples[cur_mag](1);
						worker_data->z[cur_mag][worker_data->calibration_counter_total[cur_mag]] = new_samples[cur_mag](2);
						worker_data->calibration_counter_total[cur_mag]++;
					}
				}

				calibration_counter_side++;

				unsigned new_progress = progress_percentage(worker_data) +
							(unsigned)((100 / calibration_sides) * ((float)calibration_counter_side / (float)
									worker_data->calibration_points_perside));

				if (new_progress - _last_mag_progress > 3) {
					// Progress indicator for side
					calibration_log_info(worker_data->mavlink_log_pub,
							     "[cal] %s side calibration: progress <%u>",
							     detect_orientation_str(orientation), new_progress);
					px4_usleep(10000);

					_last_mag_progress = new_progress;
				}
			}

		} else {
			poll_errcount++;
		}

		if (poll_errcount > worker_data->calibration_points_perside * 3) {
			result = calibrate_return_error;
			calibration_log_info(worker_data->mavlink_log_pub, CAL_ERROR_SENSOR_MSG);
			break;
		}
	}

	if (result == calibrate_return_ok) {
		calibration_log_info(worker_data->mavlink_log_pub, "[cal] %s side done, rotate to a different side",
				     detect_orientation_str(orientation));

		worker_data->done_count++;
		px4_usleep(20000);
		calibration_log_info(worker_data->mavlink_log_pub, CAL_QGC_PROGRESS_MSG, progress_percentage(worker_data));
	}

	return result;
}

calibrate_return mag_calibrate_all(orb_advert_t *mavlink_log_pub, int32_t cal_mask)
{
	calibrate_return result = calibrate_return_ok;

	mag_worker_data_t worker_data;

	worker_data.mavlink_log_pub = mavlink_log_pub;
	worker_data.done_count = 0;
	worker_data.calibration_points_perside = calibration_total_points / detect_orientation_side_count;
	worker_data.calibration_interval_perside_seconds = calibraton_duration_seconds / detect_orientation_side_count;
	worker_data.calibration_interval_perside_useconds = worker_data.calibration_interval_perside_seconds * 1000 * 1000;

	calibration_sides = 0;

	for (unsigned i = 0; i < (sizeof(worker_data.side_data_collected) / sizeof(worker_data.side_data_collected[0])); i++) {

		if ((cal_mask & (1 << i)) > 0) {
			// mark as missing
			worker_data.side_data_collected[i] = false;
			calibration_sides++;

		} else {
			// mark as completed from the beginning
			worker_data.side_data_collected[i] = true;

			calibration_log_info(mavlink_log_pub,
					     "[cal] %s side done, rotate to a different side",
					     detect_orientation_str(static_cast<enum detect_orientation_return>(i)));
			px4_usleep(100000);
		}
	}

	for (size_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
		// Initialize to no subscription
		worker_data.sub_mag[cur_mag] = -1;

		// Initialize to no memory allocated
		worker_data.x[cur_mag] = nullptr;
		worker_data.y[cur_mag] = nullptr;
		worker_data.z[cur_mag] = nullptr;
		worker_data.calibration_counter_total[cur_mag] = 0;
	}

	const unsigned int calibration_points_maxcount = calibration_sides * worker_data.calibration_points_perside;

	// Get actual mag count and alloate only as much memory as needed
	const unsigned orb_mag_count = orb_group_count(ORB_ID(sensor_mag));

	// Warn that we will not calibrate more than MAX_MAGS magnetometers
	if (orb_mag_count > MAX_MAGS) {
		calibration_log_critical(mavlink_log_pub, "Detected %u mags, but will calibrate only %u", orb_mag_count, MAX_MAGS);
	}

	for (size_t cur_mag = 0; cur_mag < orb_mag_count && cur_mag < MAX_MAGS; cur_mag++) {
		worker_data.x[cur_mag] = new float[calibration_points_maxcount];
		worker_data.y[cur_mag] = new float[calibration_points_maxcount];
		worker_data.z[cur_mag] = new float[calibration_points_maxcount];

		if (worker_data.x[cur_mag] == nullptr || worker_data.y[cur_mag] == nullptr || worker_data.z[cur_mag] == nullptr) {
			calibration_log_critical(mavlink_log_pub, "ERROR: out of memory");
			result = calibrate_return_error;
		}
	}


	// Setup subscriptions to mag sensors
	if (result == calibrate_return_ok) {

		// We should not try to subscribe if the topic doesn't actually exist and can be counted.
		for (unsigned cur_mag = 0; cur_mag < orb_mag_count && cur_mag < MAX_MAGS; cur_mag++) {

			// Lock in to correct ORB instance
			bool found_cur_mag = false;

			for (unsigned i = 0; i < orb_mag_count && !found_cur_mag; i++) {
				worker_data.sub_mag[cur_mag] = orb_subscribe_multi(ORB_ID(sensor_mag), i);

				sensor_mag_s report{};
				orb_copy(ORB_ID(sensor_mag), worker_data.sub_mag[cur_mag], &report);

#if 1 // TODO: replace all IOCTL usage

				// For NuttX, we get the UNIQUE device ID from the sensor driver via an IOCTL
				// and match it up with the one from the uORB subscription, because the
				// instance ordering of uORB and the order of the FDs may not be the same.

				if (report.device_id == (uint32_t)device_ids[cur_mag]) {
					// Device IDs match, correct ORB instance for this mag
					found_cur_mag = true;

				} else {
					orb_unsubscribe(worker_data.sub_mag[cur_mag]);
				}

#else

				// For the DriverFramework drivers, we fill device ID (this is the first time) by copying one report.
				device_ids[cur_mag] = report.device_id;
				found_cur_mag = true;

#endif
			}

			if (!found_cur_mag) {
				calibration_log_critical(mavlink_log_pub, "Mag #%u (ID %u) no matching uORB devid", cur_mag, device_ids[cur_mag]);
				result = calibrate_return_error;
				break;
			}

			if (device_ids[cur_mag] != 0) {
				// Get priority
				ORB_PRIO prio = ORB_PRIO_UNINITIALIZED;
				orb_priority(worker_data.sub_mag[cur_mag], &prio);

				if (prio > device_prio_max) {
					device_prio_max = prio;
					device_id_primary = device_ids[cur_mag];
				}

			} else {
				calibration_log_critical(mavlink_log_pub, "Mag #%u no device id, abort", cur_mag);
				result = calibrate_return_error;
				break;
			}
		}
	}

	// Limit update rate to get equally spaced measurements over time (in ms)
	if (result == calibrate_return_ok) {
		for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
			if (device_ids[cur_mag] != 0) {
				// Mag in this slot is available
				unsigned int orb_interval_msecs = (worker_data.calibration_interval_perside_useconds / 1000) /
								  worker_data.calibration_points_perside;

				//calibration_log_info(mavlink_log_pub, "Orb interval %u msecs", orb_interval_msecs);
				orb_set_interval(worker_data.sub_mag[cur_mag], orb_interval_msecs);
			}
		}

	}

	if (result == calibrate_return_ok) {
		int cancel_sub  = calibrate_cancel_subscribe();

		result = calibrate_from_orientation(mavlink_log_pub,                    // uORB handle to write output
						    cancel_sub,                         // Subscription to vehicle_command for cancel support
						    worker_data.side_data_collected,    // Sides to calibrate
						    mag_calibration_worker,             // Calibration worker
						    &worker_data,			// Opaque data for calibration worked
						    true);				// true: lenient still detection
		calibrate_cancel_unsubscribe(cancel_sub);
	}

	// Close subscriptions
	for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
		if (worker_data.sub_mag[cur_mag] >= 0) {
			px4_close(worker_data.sub_mag[cur_mag]);
		}
	}

	// Calculate calibration values for each mag
	float sphere_x[MAX_MAGS] {};
	float sphere_y[MAX_MAGS] {};
	float sphere_z[MAX_MAGS] {};
	float sphere_radius[MAX_MAGS] {};
	float diag_x[MAX_MAGS] {};
	float diag_y[MAX_MAGS] {};
	float diag_z[MAX_MAGS] {};
	float offdiag_x[MAX_MAGS] {};
	float offdiag_y[MAX_MAGS] {};
	float offdiag_z[MAX_MAGS] {};

	for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
		sphere_radius[cur_mag] = 0.2f;
		diag_x[cur_mag] = 1.0f;
		diag_y[cur_mag] = 1.0f;
		diag_z[cur_mag] = 1.0f;
	}

	// Sphere fit the data to get calibration values
	if (result == calibrate_return_ok) {
		for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
			if (device_ids[cur_mag] != 0) {
				// Mag in this slot is available and we should have values for it to calibrate

				// Estimate only the offsets if two-sided calibration is selected, as the problem is not constrained
				// enough to reliably estimate both scales and offsets with 2 sides only (even if the existing calibration
				// is already close)
				bool sphere_fit_only = calibration_sides <= 2;
				ellipsoid_fit_least_squares(worker_data.x[cur_mag], worker_data.y[cur_mag], worker_data.z[cur_mag],
							    worker_data.calibration_counter_total[cur_mag], 100,
							    &sphere_x[cur_mag], &sphere_y[cur_mag], &sphere_z[cur_mag],
							    &sphere_radius[cur_mag],
							    &diag_x[cur_mag], &diag_y[cur_mag], &diag_z[cur_mag],
							    &offdiag_x[cur_mag], &offdiag_y[cur_mag], &offdiag_z[cur_mag], sphere_fit_only);

				result = check_calibration_result(sphere_x[cur_mag], sphere_y[cur_mag], sphere_z[cur_mag],
								  sphere_radius[cur_mag],
								  diag_x[cur_mag], diag_y[cur_mag], diag_z[cur_mag],
								  offdiag_x[cur_mag], offdiag_y[cur_mag], offdiag_z[cur_mag],
								  mavlink_log_pub, cur_mag);

				if (result == calibrate_return_error) {
					break;
				}
			}
		}
	}

	// Print uncalibrated data points
	if (result == calibrate_return_ok) {

		// DO NOT REMOVE! Critical validation data!

		// printf("RAW DATA:\n--------------------\n");
		// for (size_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {

		// 	if (worker_data.calibration_counter_total[cur_mag] == 0) {
		// 		continue;
		// 	}

		// 	printf("RAW: MAG %u with %u samples:\n", (unsigned)cur_mag, (unsigned)worker_data.calibration_counter_total[cur_mag]);

		// 	for (size_t i = 0; i < worker_data.calibration_counter_total[cur_mag]; i++) {
		// 		float x = worker_data.x[cur_mag][i];
		// 		float y = worker_data.y[cur_mag][i];
		// 		float z = worker_data.z[cur_mag][i];
		// 		printf("%8.4f, %8.4f, %8.4f\n", (double)x, (double)y, (double)z);
		// 	}

		// 	printf(">>>>>>>\n");
		// }

		// printf("CALIBRATED DATA:\n--------------------\n");
		// for (size_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {

		// 	if (worker_data.calibration_counter_total[cur_mag] == 0) {
		// 		continue;
		// 	}

		// 	printf("Calibrated: MAG %u with %u samples:\n", (unsigned)cur_mag, (unsigned)worker_data.calibration_counter_total[cur_mag]);

		// 	for (size_t i = 0; i < worker_data.calibration_counter_total[cur_mag]; i++) {
		// 		float x = worker_data.x[cur_mag][i] - sphere_x[cur_mag];
		// 		float y = worker_data.y[cur_mag][i] - sphere_y[cur_mag];
		// 		float z = worker_data.z[cur_mag][i] - sphere_z[cur_mag];
		// 		printf("%8.4f, %8.4f, %8.4f\n", (double)x, (double)y, (double)z);
		// 	}

		// 	printf("SPHERE RADIUS: %8.4f\n", (double)sphere_radius[cur_mag]);
		// 	printf(">>>>>>>\n");
		// }
	}


	// Attempt to automatically determine external mag rotations
	int32_t param_cal_mag_rot_auto = 0;
	param_get(param_find("CAL_MAG_ROT_AUTO"), &param_cal_mag_rot_auto);

	if ((result == calibrate_return_ok) && (calibration_sides >= 3) && (param_cal_mag_rot_auto == 1)) {

		// find first internal mag to use as reference
		int internal_index = -1;

		for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
			if (internal[cur_mag] && (device_ids[cur_mag] != 0)) {
				internal_index = cur_mag;
				break;
			}
		}

		// only proceed if there's a valid internal
		if (internal_index >= 0) {

			// apply new calibrations to all raw sensor data before comparison
			for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
				if (device_ids[cur_mag] != 0) {
					for (unsigned i = 0; i < worker_data.calibration_counter_total[cur_mag]; i++) {
						// apply calibration
						float x = (worker_data.x[cur_mag][i] - sphere_x[cur_mag]) * diag_x[cur_mag];
						float y = (worker_data.y[cur_mag][i] - sphere_y[cur_mag]) * diag_y[cur_mag];
						float z = (worker_data.z[cur_mag][i] - sphere_z[cur_mag]) * diag_z[cur_mag];

						// store back in work_data
						worker_data.x[cur_mag][i] = x;
						worker_data.y[cur_mag][i] = y;
						worker_data.z[cur_mag][i] = z;
					}
				}
			}

			// rotate internal mag data to board
			param_t board_rotation_h = param_find("SENS_BOARD_ROT");
			int32_t board_rotation_int = 0;
			param_get(board_rotation_h, &(board_rotation_int));
			const enum Rotation board_rotation_id = (enum Rotation)board_rotation_int;

			if (board_rotation_int != ROTATION_NONE) {
				for (unsigned i = 0; i < worker_data.calibration_counter_total[internal_index]; i++) {
					rotate_3f(board_rotation_id,
						  worker_data.x[internal_index][i],
						  worker_data.y[internal_index][i],
						  worker_data.z[internal_index][i]
						 );
				}
			}

			for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
				if (!internal[cur_mag] && (device_ids[cur_mag] != 0)) {

					const int last_sample_index = math::min(worker_data.calibration_counter_total[internal_index],
										worker_data.calibration_counter_total[cur_mag]);

					float diff_sum[ROTATION_MAX] {};

					float min_diff = FLT_MAX;
					int32_t best_rotation = ROTATION_NONE;

					for (int r = ROTATION_NONE; r < ROTATION_MAX; r++) {
						for (int i = 0; i < last_sample_index; i++) {

							float x = worker_data.x[cur_mag][i];
							float y = worker_data.y[cur_mag][i];
							float z = worker_data.z[cur_mag][i];
							rotate_3f((enum Rotation)r, x, y, z);

							Vector3f diff = Vector3f{x, y, z} - Vector3f{worker_data.x[internal_index][i], worker_data.y[internal_index][i], worker_data.z[internal_index][i]};

							diff_sum[r] += diff.norm();
						}

						if (diff_sum[r] < min_diff) {
							min_diff = diff_sum[r];
							best_rotation = r;
						}
					}


					// Check that the best rotation is at least twice as good as the next best
					bool smallest_check_passed = true;

					for (int r = ROTATION_NONE; r < ROTATION_MAX; r++) {
						if (r != best_rotation) {
							if (diff_sum[r] < (min_diff * 2.f)) {
								smallest_check_passed = false;
							}
						}
					}


					// Check that the average error across all samples (relative to internal mag) is less than the minimum earth field (~ 0.25 Gauss)
					const float mag_error_ga = (min_diff / last_sample_index);
					bool total_error_check_passed = (mag_error_ga < 0.25f);

					if (smallest_check_passed && total_error_check_passed) {
						// TODO: compare with existing rotation?
						PX4_INFO("External Mag: %d, determined rotation: %d", cur_mag, best_rotation);

						char str[20] {};
						sprintf(str, "CAL_MAG%u_ID", cur_mag);
						param_set_no_notification(param_find(str), &device_ids[cur_mag]);

						sprintf(str, "CAL_MAG%u_ROT", cur_mag);
						param_set_no_notification(param_find(str), &best_rotation);
					}

					for (int r = ROTATION_NONE; r < ROTATION_MAX; r++) {
						PX4_DEBUG("Mag: %d, rotation: %d error: %.6f", cur_mag, r, (double)diff_sum[r]);
					}
				}
			}
		}
	}

	// Data points are no longer needed
	for (size_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
		delete[] worker_data.x[cur_mag];
		delete[] worker_data.y[cur_mag];
		delete[] worker_data.z[cur_mag];
	}

	if (result == calibrate_return_ok) {

		for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
			if (device_ids[cur_mag] != 0) {
				mag_calibration_s mscale;

#if 1 // TODO: replace all IOCTL usage
				int fd_mag = -1;
				char str[20] {};
				(void)sprintf(str, "%s%u", MAG_BASE_DEVICE_PATH, cur_mag);
				fd_mag = px4_open(str, 0);

				if (fd_mag < 0) {
					calibration_log_critical(mavlink_log_pub, "ERROR: unable to open mag device #%u", cur_mag);
					result = calibrate_return_error;
				}

#endif

				if (result == calibrate_return_ok) {

#if 1 // TODO: replace all IOCTL usage

					// Read existing calibration
					if (px4_ioctl(fd_mag, MAGIOCGSCALE, (long unsigned int)&mscale) != PX4_OK) {
						calibration_log_critical(mavlink_log_pub, CAL_ERROR_READ_CAL_MSG);
						result = calibrate_return_error;
					}

					// Update calibration
					// The formula for applying the calibration is:
					//   mag_value = (mag_readout - (offset_existing + offset_new/scale_existing)) * scale_existing * scale_new
					mscale.x_offset = mscale.x_offset + sphere_x[cur_mag] / mscale.x_scale;
					mscale.y_offset = mscale.y_offset + sphere_y[cur_mag] / mscale.y_scale;
					mscale.z_offset = mscale.z_offset + sphere_z[cur_mag] / mscale.z_scale;
					mscale.x_scale = mscale.x_scale * diag_x[cur_mag];
					mscale.y_scale = mscale.y_scale * diag_y[cur_mag];
					mscale.z_scale = mscale.z_scale * diag_z[cur_mag];

					if (px4_ioctl(fd_mag, MAGIOCSSCALE, (long unsigned int)&mscale) != PX4_OK) {
						calibration_log_critical(mavlink_log_pub, CAL_ERROR_APPLY_CAL_MSG);
						result = calibrate_return_error;
					}

#else
					mscale.x_offset = sphere_x[cur_mag];
					mscale.y_offset = sphere_y[cur_mag];
					mscale.z_offset = sphere_z[cur_mag];
					mscale.x_scale = diag_x[cur_mag];
					mscale.y_scale = diag_y[cur_mag];
					mscale.z_scale = diag_z[cur_mag];
#endif
				}

#if 1 // TODO: replace all IOCTL usage

				// Mag device no longer needed
				if (fd_mag >= 0) {
					px4_close(fd_mag);
				}

#endif

				if (result == calibrate_return_ok) {
					bool failed = false;

					/* set parameters */

					(void)sprintf(str, "CAL_MAG%u_ID", cur_mag);
					failed |= (PX4_OK != param_set_no_notification(param_find(str), &(device_ids[cur_mag])));
					(void)sprintf(str, "CAL_MAG%u_XOFF", cur_mag);
					failed |= (PX4_OK != param_set_no_notification(param_find(str), &(mscale.x_offset)));
					(void)sprintf(str, "CAL_MAG%u_YOFF", cur_mag);
					failed |= (PX4_OK != param_set_no_notification(param_find(str), &(mscale.y_offset)));
					(void)sprintf(str, "CAL_MAG%u_ZOFF", cur_mag);
					failed |= (PX4_OK != param_set_no_notification(param_find(str), &(mscale.z_offset)));

					(void)sprintf(str, "CAL_MAG%u_XSCALE", cur_mag);
					failed |= (PX4_OK != param_set_no_notification(param_find(str), &(mscale.x_scale)));
					(void)sprintf(str, "CAL_MAG%u_YSCALE", cur_mag);
					failed |= (PX4_OK != param_set_no_notification(param_find(str), &(mscale.y_scale)));
					(void)sprintf(str, "CAL_MAG%u_ZSCALE", cur_mag);
					failed |= (PX4_OK != param_set_no_notification(param_find(str), &(mscale.z_scale)));

					if (failed) {
						calibration_log_critical(mavlink_log_pub, CAL_ERROR_SET_PARAMS_MSG);
						result = calibrate_return_error;

					} else {
						PX4_INFO("[cal] mag #%u off: x:%.2f y:%.2f z:%.2f Ga",
							 cur_mag, (double)mscale.x_offset, (double)mscale.y_offset, (double)mscale.z_offset);

						PX4_INFO("[cal] mag #%u scale: x:%.2f y:%.2f z:%.2f",
							 cur_mag, (double)mscale.x_scale, (double)mscale.y_scale, (double)mscale.z_scale);

						px4_usleep(200000);
					}
				}
			}
		}

		// Trigger a param set on the last step so the whole
		// system updates
		(void)param_set(param_find("CAL_MAG_PRIME"), &(device_id_primary));
	}

	return result;
}
