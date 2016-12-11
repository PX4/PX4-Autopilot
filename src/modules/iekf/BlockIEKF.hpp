#include "ros/ros.hpp"
#include <uORB/topics/vehicle_attitude.h>

/**
 * State enum
 */
enum class X : uint8_t {
	q_nb_0 = 0, q_nb_1, q_nb_2, q_nb_3,
	vel_N, vel_E, vel_D,
	gyro_bias_bx, gyro_bias_by, gyro_bias_bz,
	accel_scale,
	pos_N, pos_E, pos_D,
	terrain_alt,
	baro_bias,
	n
};

/**
 * Error state enum
 * used for linearization
 */
enum class Xe : uint8_t {
	rot_bx = 0, rot_by, rot_bz,
	vel_N, vel_E, vel_D,
	gyro_bias_bx, gyro_bias_by, gyro_bias_bz,
	accel_scale,
	pos_N, pos_E, pos_D,
	terrain_alt,
	baro_bias,
	n
};

/**
 * Input enum
 */
enum class U : uint8_t {
	omega_nb_bx = 0, omega_nb_by, omega_nb_bz,
	a_bx, a_by, a_bz,
	n
};

/**
 * Accel measurement
 */
enum class Y_accel : uint8_t {
	bx = 0, by, bz,
	n
};

/**
 * GPS measurement
 */
enum class Y_gps : uint8_t {
	pos_N = 0, pos_E, pos_D,
	vel_N, vel_E, vel_D,
	n
};

/**
 * Baro measurement
 */
enum class Y_baro : uint8_t {
	asl = 0, n
};

/**
 * Magnetometer measurement
 */
enum class Y_mag : uint8_t {
	bx = 0, by, bz, n
};


class BlockIEKF
{
public:
	BlockIEKF();
	void update();
	bool ok() { return _nh.ok(); }
	void callback(const vehicle_attitude_s *msg);
private:
	ros::NodeHandle _nh;
	ros::Subscriber _sub_test;
	ros::Publisher _pub_test;
};
