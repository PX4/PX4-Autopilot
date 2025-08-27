#include "VehicleGNSSHeading.hpp"

#include <px4_platform_common/log.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>

namespace sensors
{
VehicleGNSSHeading::VehicleGNSSHeading() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_vehicle_gnss_heading_pub.advertise();
}

VehicleGNSSHeading::~VehicleGNSSHeading()
{
	Stop();
	perf_free(_cycle_perf);
}

bool VehicleGNSSHeading::Start()
{
	// force initial updates
	ParametersUpdate(true);

	_sensor_gnss_relative_sub.registerCallback();

	ScheduleNow();

	return true;
}

void VehicleGNSSHeading::Stop()
{
	Deinit();
	_sensor_gnss_relative_sub.unregisterCallback();
}

void VehicleGNSSHeading::ParametersUpdate(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
	}

	const float bx = _params_gpsrel_px.get();
	const float by = _params_gpsrel_py.get();
	const float bz = _params_gpsrel_pz.get();

	const float baseline_length = sqrtf(bx * bx + by * by + bz * bz);

	if (baseline_length < 0.3f) {
		PX4_ERR("[vehicle_gnss_heading]Expected baseline length is too small: %.2f m", (double)baseline_length);
		_expected_baseline_length_m = NAN;
		_baseline_body_angle = NAN;
		return;
	}

	_expected_baseline_length_m = baseline_length;
	_baseline_body_angle = atan2f(by, bx);
}

void VehicleGNSSHeading::Run()
{
	perf_begin(_cycle_perf);
	ParametersUpdate();

	if (PX4_ISFINITE(_expected_baseline_length_m) && PX4_ISFINITE(_baseline_body_angle)
	    && _sensor_gnss_relative_sub.updated()) {
		sensor_gnss_relative_s d{};
		_sensor_gnss_relative_sub.copy(&d);

		// Basic validity
		if (!PX4_ISFINITE(d.heading) || !PX4_ISFINITE(d.heading_accuracy) ||
		    !PX4_ISFINITE(d.position_length) || !d.heading_valid || !d.relative_position_valid) {
			perf_end(_cycle_perf);
			return;
		}

		if (_params_gpsyaw_crq.get() == 1 && (!d.carrier_solution_floating || !d.carrier_solution_fixed)) {
			perf_end(_cycle_perf);
			return;
		}

		if (_params_gpsyaw_crq.get() == 2 && !d.carrier_solution_fixed) {
			perf_end(_cycle_perf);
			return;
		}

		// The reported baseline length compared to expected is a good indication of accuracy.
		// Additionally, it prevents potentially large errors if the wrong RTCM stream is sent to the receiver.
		const float gate_len = _params_gpsyaw_lrq.get();

		if (PX4_ISFINITE(gate_len) && gate_len >= 0.0f) {
			const float len_diff = fabsf(d.position_length - _expected_baseline_length_m);

			if (len_diff > gate_len) {
				perf_end(_cycle_perf);
				return;
			}
		}

		const hrt_abstime prev_valid = _last_valid_timestamp;
		_last_valid_timestamp = d.timestamp;

		// Reject if the delta time to previous valid update is too large, ublox receivers can output stale data
		// when in rover mode. F9P rovers publish every 125 ms, we accept up to 200 ms additional delay.
		if (prev_valid == 0 || d.timestamp - prev_valid > 325_ms) {
			perf_end(_cycle_perf);
			return;
		}

		// Rotate into vehicle frame using configured antenna position
		const float vehicle_yaw = matrix::wrap_pi(d.heading - _baseline_body_angle);

		// publish
		vehicle_gnss_heading_s out{
			.timestamp = d.timestamp,
			.timestamp_sample = d.timestamp_sample,
			.heading = vehicle_yaw,
			.heading_accuracy = d.heading_accuracy
		};

		_vehicle_gnss_heading_pub.publish(out);
	}

	perf_end(_cycle_perf);
}

void VehicleGNSSHeading::PrintStatus()
{
	PX4_INFO("VehicleGNSSHeading status:");

	const float bx = _params_gpsrel_px.get();
	const float by = _params_gpsrel_py.get();
	const float bz = _params_gpsrel_pz.get();
	const float gate_len = _params_gpsyaw_lrq.get();
	const int32_t crq = _params_gpsyaw_crq.get();

	PX4_INFO("  Antenna baseline (body) [m]: bx=%.2f by=%.2f bz=%.2f", (double)bx, (double)by, (double)bz);
	PX4_INFO("  Expected baseline length [m]: %s%.2f",
		 isfinite(_expected_baseline_length_m) ? "" : "(invalid) ",
		 (double)_expected_baseline_length_m);

	if (PX4_ISFINITE(gate_len) && gate_len >= 0.f) {
		PX4_INFO("  Baseline length gate [m]: %.2f m", (double)gate_len);

	} else {
		PX4_INFO("  Baseline length gate: disabled");
	}

	PX4_INFO("  Required carrier solution: %d (0=None, 1=Floating or Fixed, 2=Fixed)", (int)crq);

	if (_last_valid_timestamp != 0) {
		const float since_last_valid_s = (hrt_absolute_time() - _last_valid_timestamp) * 1e-6f;
		PX4_INFO("  Last valid GNSS-relative message: %.3f s ago", (double)since_last_valid_s);

	} else {
		PX4_INFO("  Last valid GNSS-relative message: none yet");
	}

	perf_print_counter(_cycle_perf);
}

}; // namespace sensors
