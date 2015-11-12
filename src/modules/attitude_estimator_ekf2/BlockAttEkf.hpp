#pragma once

#include <controllib/blocks.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/control_state.h>

#include <matrix/math.hpp>
#include <poll.h>

using namespace matrix;
using namespace control;

class BlockAttEkf : public control::SuperBlock
{
public:
	BlockAttEkf();
	void update();
	void predict();
	void correct_mag_accel();
	void publish_attitude();

private:
	// publications
	uORB::Publication<control_state_s> _pub_control;
	uORB::Publication<vehicle_attitude_s> _pub_att;

	// subscriptions
	uORB::Subscription<sensor_combined_s> _sub_sensor;
	uORB::Subscription<parameter_update_s> _sub_param_update;
	uORB::Subscription<vehicle_global_position_s> _sub_pos;

	// params
	BlockParamFloat _omega_stddev;
	BlockParamFloat _bias_dot_stddev;
	BlockParamFloat _mag_stddev;;
	BlockParamFloat _accel_stddev;


	// derivative
	control::BlockDerivative _omega_dot_x;
	control::BlockDerivative _omega_dot_y;
	control::BlockDerivative _omega_dot_z;

	// derivative
	control::BlockDerivative _a_x;
	control::BlockDerivative _a_y;
	control::BlockDerivative _a_z;

	// states
	Quatf _q_nr; // quaternion from nav to ref frame
	Vector3f _b_gyro; // gyro bias
	Vector3f _omega_nr; // gyro angular velocity

	// covariance matrix
	SquareMatrix<float, 6> _P; // covariance matrix

	// misc
	uint64_t _timestamp; // time of last update
	uint64_t _gyro_timestamp;
	uint64_t _mag_timestamp;
	uint64_t _accel_timestamp;

	struct pollfd _polls[1];
};
