

// #include <drivers/drv_hrt.h>
// #include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
// #include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
// #include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
// #include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/bitmask.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/sensor_gps.h>
#include <lib/geo/geo.h>
#include <matrix/math.hpp>


static constexpr uint8_t ACCEL_COUNT_MAX = 4;
static constexpr uint8_t GYRO_COUNT_MAX  = 4;
static constexpr uint8_t MAG_COUNT_MAX   = 4;
static constexpr uint8_t BARO_COUNT_MAX  = 4;


static constexpr float RELATIVE_GPS_DRIFT = 0.1f;

class FailureInjection
{

public:
	FailureInjection() {};
	void test();


	void check_failure_injections();
	bool handle_gps_failure(sensor_gps_s &gps);
	bool handle_gps_alt_failure(sensor_gps_s &gps);

	bool is_accel_blocked(const int instance=0) { return _accel_blocked[instance]; }
	bool is_accel_stuck(const int instance=0) { return _accel_stuck[instance]; }

private:

	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s> _command_ack_pub{ORB_ID(vehicle_command_ack)};



	// GPS
	bool _gps_drift{false};
	bool _gps_alt_drift{false};
	MapProjection _mp;
	bool _has_drift_ref{false};
	matrix::Vector2f _gps_drift_pos;
	double _gps_alt_offset{0};
	hrt_abstime _last_gps_timestamp{0};
	hrt_abstime _t0{0};

	bool _gps_blocked{false};
	bool _gps_alt_blocked{false};
	bool _gps_stuck{false};
	bool _gps_alt_stuck{false};
	bool _gps_wrong{false};
	bool _gps_alt_wrong{false};
	sensor_gps_s _gps_prev{};


	// accel
	bool _accel_blocked[ACCEL_COUNT_MAX] {};
	bool _accel_stuck[ACCEL_COUNT_MAX] {};

};
