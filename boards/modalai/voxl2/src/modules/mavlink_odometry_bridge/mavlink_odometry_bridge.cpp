/****************************************************************************
 *
 *   Copyright (c) 2024 ModalAI, inc. All rights reserved.
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
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <mavlink.h>
#include <px4_log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <uORB/uORB.h>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/modal_io_mavlink_data.h>

#define VIO_MAGIC_NUMBER (0x5455524)
#define VIO_STATE_FAILED            0
#define VIO_STATE_INITIALIZING      1
#define VIO_STATE_OK                2

typedef struct vio_data_t{
    uint32_t magic_number;         ///< Unique 32-bit number used to signal the beginning of a VIO packet while parsing a data stream.
    int32_t quality;                 ///< Quality is be >0 in normal use with a larger number indicating higher quality. A positive quality does not guarantee the algorithm has initialized completely.
    int64_t timestamp_ns;          ///< Timestamp in clock_monotonic system time of the provided pose.
    float T_imu_wrt_vio[3];        ///< Translation of the IMU with respect to VIO frame in meters.
    float R_imu_to_vio[3][3];      ///< Rotation matrix from IMU to VIO frame.
    float pose_covariance[21];     ///<  Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.*/
    float vel_imu_wrt_vio[3];      ///< Velocity of the imu with respect to the VIO frame.
    float velocity_covariance[21]; ///<  Row-major representation of a 6x6 velocity cross-covariance matrix upper right triangle (states: vx, vy, vz, rollspeed, pitchspeed, yawspeed; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.*/
    float imu_angular_vel[3];      ///< Angular velocity of the IMU about its X Y and Z axes respectively. Essentially filtered gyro values with internal biases applied.
    float gravity_vector[3];       ///< Estimation of the current gravity vector in VIO frame. Use this to estimate the rotation between VIO frame and a gravity-aligned VIO frame if desired.
    float T_cam_wrt_imu[3];        ///< Location of the optical center of the camera with respect to the IMU.
    float R_cam_to_imu[3][3];      ///< Rotation matrix from camera frame to IMU frame.
    uint32_t error_code;           ///< bitmask that can indicate multiple errors. may still contain errors if state==VIO_STATE_OK
    uint16_t n_feature_points;     ///< Number of optical feature points currently being tracked.
    uint8_t state;                 ///< This is used to check the overall state of the algorithm. Can be VIO_STATE_FAILED, VIO_STATE_INITIALIZING, or VIO_STATE_OK.
    uint8_t reserved;              ///< extra byte reserved for future use
} __attribute__((packed)) vio_data_t;

class MavlinkOdometryBridge : public ModuleBase<MavlinkOdometryBridge>, public px4::WorkItem
{
public:

	MavlinkOdometryBridge();
	~MavlinkOdometryBridge() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	int64_t TimeMonotonic_ns();

	uORB::SubscriptionCallbackWorkItem _mavlink_data_sub{this, ORB_ID(modal_io_mavlink_data)};

	modal_io_mavlink_data_s _mavlink_odometry{};

	int pipe_fd;

};

MavlinkOdometryBridge::MavlinkOdometryBridge() :
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

bool MavlinkOdometryBridge::init()
{
	if (!_mavlink_data_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

    if (mkfifo("/run/mpa/hitl_vio", 0666)) {
        if (errno != EEXIST) {
            PX4_ERR("Error in pipe_server_create calling mkfifo");
			return false;
        }
    }

    pipe_fd = open("/run/mpa/hitl_vio", O_RDWR);
    if (pipe_fd < 0) {
        PX4_ERR("Error in pipe_server_create opening request path");
		return false;
    }

	return true;
}

int64_t MavlinkOdometryBridge::TimeMonotonic_ns()
{
	struct timespec ts;
	if(clock_gettime(CLOCK_MONOTONIC, &ts)){
		fprintf(stderr,"ERROR calling clock_gettime\n");
		return -1;
	}
	return (int64_t)ts.tv_sec*1000000000 + (int64_t)ts.tv_nsec;
}

void MavlinkOdometryBridge::Run()
{
	if (should_exit()) {
		_mavlink_data_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_mavlink_data_sub.updated()) {
		if (_mavlink_data_sub.update(&_mavlink_odometry)) {
			mavlink_odometry_t *odom_in = (mavlink_odometry_t*) _mavlink_odometry.odometry;

			if (_mavlink_odometry.dump_message) {
				// uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
				// float x; /*< [m] X Position*/
				// float y; /*< [m] Y Position*/
				// float z; /*< [m] Z Position*/
				// float q[4]; /*<  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)*/
				// float vx; /*< [m/s] X linear speed*/
				// float vy; /*< [m/s] Y linear speed*/
				// float vz; /*< [m/s] Z linear speed*/
				// float rollspeed; /*< [rad/s] Roll angular speed*/
				// float pitchspeed; /*< [rad/s] Pitch angular speed*/
				// float yawspeed; /*< [rad/s] Yaw angular speed*/
				// uint8_t frame_id; /*<  Coordinate frame of reference for the pose data.*/
				// uint8_t child_frame_id; /*<  Coordinate frame of reference for the velocity in free space (twist) data.*/
				// uint8_t reset_counter; /*<  Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.*/
				// uint8_t estimator_type; /*<  Type of estimator that is providing the odometry.*/
				// int8_t quality; /*< [%] Optional odometry quality metric as a percentage. -1 = odometry has failed, 0 = unknown/unset quality, 1 = worst quality, 100 = best quality*/

				PX4_INFO("Timestamp: %lu", odom_in->time_usec);
				PX4_INFO("x, y, z: %f, %f, %f", (double) odom_in->x, (double) odom_in->y, (double) odom_in->z);
				PX4_INFO("q: %f, %f, %f, %f", (double) odom_in->q[0], (double) odom_in->q[1], (double) odom_in->q[2], (double) odom_in->q[3]);
				PX4_INFO("vx, vy, vz: %f, %f, %f", (double) odom_in->vx, (double) odom_in->vy, (double) odom_in->vz);
				PX4_INFO("quality %d", odom_in->quality);
			}

			vio_data_t s;
			memset(&s,0,sizeof(s));

			s.magic_number = VIO_MAGIC_NUMBER;
			s.error_code = 0;
			s.state = VIO_STATE_OK;
			s.timestamp_ns = TimeMonotonic_ns();
			// s.timestamp_ns = _mavlink_odometry.timestamp * 1000;

			s.T_imu_wrt_vio[0] = odom_in->x;
			s.T_imu_wrt_vio[1] = odom_in->y;
			s.T_imu_wrt_vio[2] = odom_in->z;

			float w = odom_in->q[0], x = odom_in->q[1], y = odom_in->q[2], z = odom_in->q[3];
			float xx = x * x, yy = y * y, zz = z * z;
			float xy = x * y, xz = x * z, yz = y * z;
			float wx = w * x, wy = w * y, wz = w * z;

			s.R_imu_to_vio[0][0] = 1 - 2 * (yy + zz);
			s.R_imu_to_vio[0][1] = 2 * (xy - wz);
			s.R_imu_to_vio[0][2] = 2 * (xz + wy);

			s.R_imu_to_vio[1][0] = 2 * (xy + wz);
			s.R_imu_to_vio[1][1] = 1 - 2 * (xx + zz);
			s.R_imu_to_vio[1][2] = 2 * (yz - wx);

			s.R_imu_to_vio[2][0] = 2 * (xz - wy);
			s.R_imu_to_vio[2][1] = 2 * (yz + wx);
			s.R_imu_to_vio[2][2] = 1 - 2 * (xx + yy);

			float gravity_vector[3] = {0,0,1};
			memcpy(s.gravity_vector, gravity_vector, sizeof(gravity_vector));

			float linearVelocity[3] = {odom_in->vx, odom_in->vy, odom_in->vz};
			float angularVelocity[3] = {odom_in->rollspeed, odom_in->pitchspeed, odom_in->yawspeed};
			
			memcpy(s.vel_imu_wrt_vio,	linearVelocity,		sizeof(float)*3);
			memcpy(s.imu_angular_vel,	angularVelocity,	sizeof(float)*3);

			// pose covariance diagonals, 6 entries
			s.pose_covariance[0] =  (float)odom_in->pose_covariance[0];
			s.pose_covariance[6] =  (float)odom_in->pose_covariance[6];
			s.pose_covariance[11] = (float)odom_in->pose_covariance[11];
			s.pose_covariance[15] = (float)odom_in->pose_covariance[15];
			s.pose_covariance[18] = (float)odom_in->pose_covariance[18];
			s.pose_covariance[20] = (float)odom_in->pose_covariance[20];

			// velocity covariance diagonals, 3 entries
			s.velocity_covariance[0] = (float)odom_in->pose_covariance[0];
			s.velocity_covariance[6] = (float)odom_in->pose_covariance[6];
			s.velocity_covariance[11] = (float)odom_in->pose_covariance[11];

			int quality = odom_in->quality;
			if(odom_in->quality>=100) quality = 100;
			// if(odom_in->quality<1) quality = -1;
			if(odom_in->quality<50) quality = 50;

			s.quality = quality;

			int result = write(pipe_fd, (char*)&s, sizeof(vio_data_t));

			// write was good!
			if(result != sizeof(vio_data_t)) {
				PX4_ERR("Pipe write failed!");
			}
		}
	}
}

int MavlinkOdometryBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MavlinkOdometryBridge::task_spawn(int argc, char *argv[])
{
	MavlinkOdometryBridge *instance = new MavlinkOdometryBridge();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MavlinkOdometryBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Mavlink odometry bridge

)DESCR_STR");

	// PRINT_MODULE_USAGE_NAME("mavlink_odometry_bridge");
	PRINT_MODULE_USAGE_NAME("mavlink_odometry_bridge", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mavlink_odometry_bridge_main(int argc, char *argv[])
{
	return MavlinkOdometryBridge::main(argc, argv);
}
