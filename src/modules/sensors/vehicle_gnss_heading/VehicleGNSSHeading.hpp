#pragma once

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gnss_relative.h>
#include <uORB/topics/vehicle_gnss_heading.h>

using namespace time_literals;

namespace sensors
{
class VehicleGNSSHeading : public ModuleParams, public px4::ScheduledWorkItem
{
public:

	VehicleGNSSHeading();
	~VehicleGNSSHeading() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	void Run() override;

	void ParametersUpdate(bool force = false);

	uORB::Publication<vehicle_gnss_heading_s> _vehicle_gnss_heading_pub{ORB_ID(vehicle_gnss_heading)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// Only one subscription callback for GNSS relative, since PX4 currently only supports two receivers,
	// i.e. one base/rover pair.
	uORB::SubscriptionCallbackWorkItem _sensor_gnss_relative_sub{this, ORB_ID(sensor_gnss_relative), 0};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SENS_GNSSREL_PX>) _params_gpsrel_px,
		(ParamFloat<px4::params::SENS_GNSSREL_PY>) _params_gpsrel_py,
		(ParamFloat<px4::params::SENS_GNSSREL_PZ>) _params_gpsrel_pz,
		(ParamFloat<px4::params::SENS_GNSSHDG_LRQ>) _params_gpsyaw_lrq,
		(ParamInt<px4::params::SENS_GNSSHDG_CRQ>) _params_gpsyaw_crq
	)

	float _expected_baseline_length_m{NAN};
	float _baseline_body_angle{NAN};
	hrt_abstime _last_valid_timestamp{0};
};
}; // namespace sensors
