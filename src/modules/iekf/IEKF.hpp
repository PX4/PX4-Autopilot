/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

//#include "controllib/blocks.hpp"
#include "ros/ros.hpp"
#include "matrix/math.hpp"
#include <lib/geo/geo.h>

// local includes
#include "Origin.hpp"
#include "Sensor.hpp"
#include "constants.hpp"

// subscriptions
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/vision_position_estimate.h>
#include <uORB/topics/att_pos_mocap.h>
#include <uORB/topics/vehicle_land_detected.h>
//#include <uORB/topics/ekf2_innovations.h>
#include <uORB/topics/parameter_update.h>

// publications
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_state.h>
#include <uORB/topics/estimator_state_std.h>
#include <uORB/topics/estimator_innov.h>
#include <uORB/topics/estimator_innov_std.h>

using namespace matrix;
//using namespace control;

/**
 * Main class for invariant extended kalman filter
 *
 * inspired by: https://hal.archives-ouvertes.fr/hal-00494342/document
 *
 * See: https://github.com/jgoppert/iekf_analysisa for derivation/ simulation
 */
class IEKF /* : public SuperBlock */
{
public:
	IEKF();

	// methods
	//
	void update();
	Vector<float, X_n> dynamics(
		float t, const Vector<float, X_n> &x,
		const Vector<float, U_n> &u) const;
	Vector3f transAccelFrameB() const;
	Vector3f rotAccelFrameB() const;
	void callbackImu(const sensor_combined_s *msg);
	void updateParams();
	void callbackParamUpdate(const parameter_update_s *msg);
	void initializeAttitude(const sensor_combined_s *msg);
	void predictState(const actuator_controls_s *msg);
	void predictCovariance(uint64_t timestamp);
	void reconstructPosiiveP();
	Vector<float, X_n> computeErrorCorrection(const Vector<float, Xe_n> &d_xe) const;
	void correctionLogic(Vector<float, X_n> &dx) const;
	void boundP();
	void boundX();
	void publish();
	void normalizeQuaternion();
	void stateSpaceParamUpdate();
	void stateSpaceStateUpdate();

	// sensor corrections/ callbacks
	void correctAccel(const sensor_combined_s *msg);
	void correctMag(const sensor_combined_s *msg);
	void correctBaro(const sensor_combined_s *msg);
	void correctGyro(const sensor_combined_s *msg);
	void correctGps(const vehicle_gps_position_s *msg);
	void correctAirspeed(const airspeed_s *msg);
	void correctFlow(const optical_flow_s *msg);
	void correctDistance(const distance_sensor_s *msg);
	void correctVision(const vision_position_estimate_s *msg);
	void correctMocap(const att_pos_mocap_s *msg);
	void callbackLand(const vehicle_land_detected_s *msg);
	void correctLand(uint64_t timestamp);

	// getters/ setters
	bool ok() { return _nh.ok(); }
	void setP(const SquareMatrix<float, Xe_n> &P)
	{
		_P = P;
		boundP();
	}
	inline void incrementP(const SquareMatrix<float, Xe_n> &dP)
	{
		_P += dP;
		boundP();
	}
	void setX(const Vector<float, X_n> &x)
	{
		_x = x;
		boundX();
	}
	void incrementX(Vector<float, X_n> &dx)
	{
		correctionLogic(dx);
		_x += dx;
		boundX();
	}
	inline bool getLanded() const
	{
		return _landed && (!getTerrainValid() || (getAgl() < 0.5f));
	}
	inline bool getAttitudeValid() const
	{
		return _attitudeInitialized &&
		       ((_P(Xe_rot_N, Xe_rot_N)
			 + _P(Xe_rot_E, Xe_rot_E)
			 + _P(Xe_rot_D, Xe_rot_D)) < 1.0f);

	};
	inline bool getVelocityXYValid() const
	{
		return ((_P(Xe_vel_N, Xe_vel_N)
			 + _P(Xe_vel_E, Xe_vel_E)) < 0.2f);
	};
	inline bool getVelocityZValid() const
	{
		return _P(Xe_vel_D, Xe_vel_D) < 1.0f;
	};

	inline bool getPositionXYValid() const
	{
		// only require velocity valid for missions
		return _origin.xyInitialized()
		       && ((_P(Xe_vel_N, Xe_vel_N)
			    + _P(Xe_vel_E, Xe_vel_E)) < 2.0f);
	};
	inline bool getAltitudeValid() const
	{
		return _origin.altInitialized()
		       && (_P(Xe_asl, Xe_asl) < 1.0f);
	};
	inline bool getAglValid() const
	{
		return _origin.altInitialized()
		       && (_P(Xe_asl, Xe_asl) < 1.0f)
		       && (_P(Xe_terrain_asl, Xe_terrain_asl) < 1.0f);
	};
	inline bool getTerrainValid() const
	{
		return (_P(Xe_terrain_asl, Xe_terrain_asl) < 1.0f);
	};
	inline float getAgl() const
	{
		return _x(X_asl) - _x(X_terrain_asl);
	}
	inline float getAltAboveOrigin() const
	{
		return _x(X_asl) - _origin.getAlt();
	};
	inline bool getGyroSaturated() const
	{
		return _gyroSaturated;
	}
	inline bool getAccelSaturated() const
	{
		return _accelSaturated;
	}
	inline Quatf getQuaternionNB() const
	{
		return Quatf(_x(X_q_nb_0), _x(X_q_nb_1),
			     _x(X_q_nb_2), _x(X_q_nb_3));
	}
	inline Dcmf computeDcmNB() const
	{
		return getQuaternionNB();
	}
	inline Vector3f getGyroBiasFrameB() const
	{
		return Vector3f(_x(X_gyro_bias_bX), _x(X_gyro_bias_bY), _x(X_gyro_bias_bZ));
	}
	inline Vector3f getGyroRawFrameB() const
	{
		return Vector3f(_u(U_omega_nb_bX), _u(U_omega_nb_bY), _u(U_omega_nb_bZ));
	}
	inline Vector3f getAngularVelocityNBFrameB() const
	{
		return getGyroRawFrameB() - getGyroBiasFrameB();
	}
	inline Vector3f getGroundVelocity() const
	{
		return Vector3f(_x(X_vel_N), _x(X_vel_E), _x(X_vel_D));
	}
	inline Vector3f getAccelerationFrameB() const
	{
		Vector3f a_b(_u(U_accel_bX), _u(U_accel_bY), _u(U_accel_bZ));
		Vector3f a_bias_b(_x(X_accel_bias_bX), _x(X_accel_bias_bY), _x(X_accel_bias_bZ));
		return a_b - a_bias_b;
	}
	inline void nullPositionCorrection(Vector<float, Xe_n> &dxe)
	{
		dxe(Xe_vel_N) = 0;
		dxe(Xe_vel_E) = 0;
		dxe(Xe_vel_D) = 0;
		dxe(Xe_pos_N) = 0;
		dxe(Xe_pos_E) = 0;
		dxe(Xe_asl) = 0;
		dxe(Xe_terrain_asl) = 0;
		dxe(Xe_accel_bias_N) = 0;
		dxe(Xe_accel_bias_E) = 0;
		dxe(Xe_accel_bias_D) = 0;
	}
	inline void nullAttitudeCorrection(Vector<float, Xe_n> &dxe)
	{
		dxe(Xe_rot_N) = 0;
		dxe(Xe_rot_E) = 0;
		dxe(Xe_rot_D) = 0;
		dxe(Xe_gyro_bias_N) = 0;
		dxe(Xe_gyro_bias_E) = 0;
		dxe(Xe_gyro_bias_D) = 0;
	}

private:
	IEKF(const IEKF &other);
	const IEKF &operator=(const IEKF &other);
	ros::NodeHandle _nh;

	// sensors
	Sensor _sensorAccel;
	Sensor _sensorMag;
	Sensor _sensorGyro;
	Sensor _sensorBaro;
	Sensor _sensorGps;
	Sensor _sensorAirspeed;
	Sensor _sensorFlow;
	Sensor _sensorSonar;
	Sensor _sensorLidar;
	Sensor _sensorVision;
	Sensor _sensorMocap;
	Sensor _sensorLand;

	// subscriptions
	ros::Subscriber _subImu;
	ros::Subscriber _subGps;
	ros::Subscriber _subAirspeed;
	ros::Subscriber _subFlow;
	ros::Subscriber _subDistance;
	ros::Subscriber _subVision;
	ros::Subscriber _subMocap;
	ros::Subscriber _subLand;
	ros::Subscriber _subParamUpdate;
	ros::Subscriber _subActuatorControls;

	// publishers
	ros::Publisher _pubAttitude;
	ros::Publisher _pubLocalPosition;
	ros::Publisher _pubGlobalPosition;
	ros::Publisher _pubControlState;
	ros::Publisher _pubEstimatorStatus;

	ros::Publisher _pubEstimatorState;
	ros::Publisher _pubEstimatorStateStd;
	ros::Publisher _pubEstimatorInnov;
	ros::Publisher _pubEstimatorInnovStd;


	// data
	Vector<float, X_n> _x0; 		// initial state vector
	Vector<float, X_n> _xMin; 		// lower bound vector
	Vector<float, X_n> _xMax; 		// upper bound vector
	Vector<float, Xe_n> _P0Diag; 	// initial state diagonal
	Vector<float, X_n> _x; 		// state vector
	SquareMatrix<float, Xe_n> _P; 	// covariance matrix
	Vector<float, U_n> _u; 		// input vector
	Vector3f _g_n; 					// expected gravity in navigation frame
	Origin _origin; 				// origin of local coordinate system
	float _baroAsl; 				// pressure altitude from baro
	float _baroOffset; 				// pressure offset
	bool _landed;					// are we landed?
	bool _freefall;					// are we freefalling?
	uint64_t _gpsUSec; 				// gps internal timestamp
	bool _attitudeInitialized;		// attitude initializtion complete
	uint64_t _stateTimestamp;		// state prediction timestamp
	uint64_t _imuTimestamp;			// imu combined sensor timestamp
	uint64_t _covarianceTimestamp;; // covariance prediction timestamp
	int _imuLowRateIndex;
	int _predictLowRateIndex;
	bool _accelSaturated;
	bool _gyroSaturated;
	SquareMatrix<float, Xe_n> _A;
	SquareMatrix<float, Xe_n> _Q;
	Vector<float, Xe_n> _dxe; 		// 	error vector
	SquareMatrix<float, Xe_n> _dP; 	// change in covariance matrix used for checking
	Vector<float, Innov_n>  _innov;
	Vector<float, Innov_n>  _innovStd;
	enum {
		COV_STEP_AP = 0,
		COV_STEP_PAT = 1,
		COV_STEP_Q = 2,
	};
	uint16_t _imuOverruns;
	uint16_t _predictOverruns;

	// params
	float _gyro_nd;
	float _gyro_rw_nd;
	float _gyro_rw_ct;
	float _accel_nd;
	float _accel_rw_nd;
	float _accel_rw_ct;
	float _baro_nd;
	float _baro_rw_nd;
	float _baro_rw_ct;
	float _mag_nd;
	float _mag_rw_nd;
	float _mag_rw_ct;
	float _mag_decl_deg;
	float _gps_xy_nd;
	float _gps_z_nd;
	float _gps_vxy_nd;
	float _gps_vz_nd;
	float _flow_nd;
	float _lidar_nd;
	float _sonar_nd;
	float _land_vxy_nd;
	float _land_vz_nd;
	float _land_agl_nd;
	float _pn_xy_nd;
	float _pn_vxy_nd;
	float _pn_z_nd;
	float _pn_vz_nd;
	float _pn_rot_nd;
	float _pn_t_asl_nd;
	float _pn_t_asl_s_nd;
};
