

#pragma once

#include <Integrator.hpp>

#include <lib/sensor_calibration/Accelerometer.hpp>
#include <lib/sensor_calibration/Gyroscope.hpp>
#include <lib/mathlib/math/WelfordMean.hpp>
#include <lib/mathlib/math/WelfordMeanVector.hpp>

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_imu_status.h>

namespace sensors
{

struct IMU {

	struct InFlightCalibration {
		uint8_t instance{0};
		matrix::Vector3f offset{};
		matrix::Vector3f bias_variance{};
		bool valid{false};
	};

	struct {

		calibration::Accelerometer calibration{};
		math::WelfordMeanVector<float, 3> raw_mean{};




		// FIFO only
		matrix::Vector3f integral{};
		int16_t last_raw_sample[3] {};
		float fifo_scale{1.f};

		math::WelfordMean<float> mean_publish_interval_us{}; // only needed for FIFO


		// redundant for FIFO case
		hrt_abstime timestamp_sample_last{0};
		uint32_t device_id{0};
		uint32_t error_count{0};
		math::WelfordMean<float> temperature{};              // redundant for FIFO case
		math::WelfordMean<float> mean_sample_interval_us{};
		bool interval_configured{false};


		// non-FIFO only
		sensors::Integrator integrator{};




		matrix::Vector3f estimated_bias{};
		matrix::Vector3f estimated_bias_variance{};

		uint32_t clipping_total[3] {}; // clipping
		uint8_t clipping_flags{0};

		matrix::Vector3f acceleration_prev{}; // acceleration from the previous IMU measurement for vibration metrics

		float vibration_metric{0.f}; // high frequency vibration level in the accelerometer data (m/s/s)


		InFlightCalibration learned_calibration[3] {};

	} accel{};

	struct {
		hrt_abstime timestamp_sample_last{0};
		uint32_t device_id{0};
		calibration::Gyroscope calibration{};
		math::WelfordMean<float> mean_publish_interval_us{};
		math::WelfordMean<float> mean_sample_interval_us{};
		math::WelfordMeanVector<float, 3> raw_mean{};
		math::WelfordMean<float> temperature{};

		uint32_t error_count{0};
		sensors::IntegratorConing integrator{};

		matrix::Vector3f estimated_bias{};
		matrix::Vector3f estimated_bias_variance{};

		uint32_t clipping_total[3] {}; // clipping
		uint8_t clipping_flags{0};

		matrix::Vector3f angular_velocity_prev{}; // angular velocity from the previous IMU measurement for vibration metrics

		float vibration_metric{0.f}; // high frequency vibration level in the gyro data (rad/s)

		float coning_norm_accum{0}; // average IMU delta angle coning correction (rad^2)
		float coning_norm_accum_total_time_s{0};

		bool interval_configured{false};

		InFlightCalibration learned_calibration[3] {};
	} gyro{};

	bool primary{false};

	uORB::PublicationMulti<vehicle_imu_s> vehicle_imu_pub{ORB_ID(vehicle_imu)};
	uORB::PublicationMulti<vehicle_imu_status_s> vehicle_imu_status_pub{ORB_ID(vehicle_imu_status)};

	hrt_abstime time_last_status_publication{0};

	hrt_abstime time_last_valid{0};
	hrt_abstime time_last_invalid{0};

	// last valid
	// last invalid


	hrt_abstime time_last_move_detect_us{0};
	hrt_abstime time_still_detect_us{0};
	bool at_rest{false};


	// noise when still?

};

} // namespace sensors
