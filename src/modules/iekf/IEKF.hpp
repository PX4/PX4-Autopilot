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

#include "controllib/blocks.hpp"
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

using namespace control;
using namespace matrix;
using namespace uORB;

/**
 * Main class for invariant extended kalman filter
 *
 * inspired by: https://hal.archives-ouvertes.fr/hal-00494342/document
 *
 * See: https://github.com/jgoppert/iekf_analysisa for derivation/ simulation
 */
class IEKF : public SuperBlock
{
public:
	IEKF();

	// methods
	//
	void update();
	Vector<float, X::n> dynamics(
		float t, const Vector<float, X::n> &x,
		const Vector<float, U::n> &u) const;
	void callbackImu(const sensor_combined_s *msg);
	void updateParams();
	void callbackParamUpdate(const parameter_update_s *msg);
	void initializeAttitude(const sensor_combined_s *msg);
	void predictState(const sensor_combined_s *msg);
	void predictCovariance(const sensor_combined_s *msg);
	void reconstructPosiiveP();
	Vector<float, X::n> computeErrorCorrection(const Vector<float, Xe::n> &d_xe) const;
	void correctionLogic(Vector<float, X::n> &dx) const;
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
	void correctGps(const vehicle_gps_position_s *msg);
	void correctAirspeed(const airspeed_s *msg);
	void correctFlow(const optical_flow_s *msg);
	void correctDistance(const distance_sensor_s *msg);
	void correctVision(const vehicle_local_position_s *msg);
	void correctMocap(const att_pos_mocap_s *msg);
	void callbackLand(const vehicle_land_detected_s *msg);
	void correctLand(uint64_t timestamp);

	// getters/ setters
	void setP(const SquareMatrix<float, Xe::n> &P)
	{
		_P = P;
		boundP();
	}
	inline void incrementP(const SquareMatrix<float, Xe::n> &dP)
	{
		_P += dP;
		boundP();
	}
	void setX(const Vector<float, X::n> &x)
	{
		_x = x;
		boundX();
	}
	void incrementX(Vector<float, X::n> &dx)
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
		       ((_P(Xe::rot_N, Xe::rot_N)
			 + _P(Xe::rot_E, Xe::rot_E)
			 + _P(Xe::rot_D, Xe::rot_D)) < 1.0f);

	};
	inline bool getVelocityXYValid() const
	{
		return ((_P(Xe::vel_N, Xe::vel_N)
			 + _P(Xe::vel_E, Xe::vel_E)) < 0.2f);
	};
	inline bool getVelocityZValid() const
	{
		return _P(Xe::vel_D, Xe::vel_D) < 1.0f;
	};

	inline bool getPositionXYValid() const
	{
		// only require velocity valid for missions
		return _origin.xyInitialized()
		       && ((_P(Xe::vel_N, Xe::vel_N)
			    + _P(Xe::vel_E, Xe::vel_E)) < 2.0f);
	};
	inline bool getAltitudeValid() const
	{
		return _origin.altInitialized()
		       && (_P(Xe::asl, Xe::asl) < 1.0f);
	};
	inline bool getAglValid() const
	{
		return _origin.altInitialized()
		       && (_P(Xe::asl, Xe::asl) < 1.0f)
		       && (_P(Xe::terrain_asl, Xe::terrain_asl) < 1.0f);
	};
	inline bool getTerrainValid() const
	{
		return (_P(Xe::terrain_asl, Xe::terrain_asl) < 1.0f);
	};
	inline float getAgl() const
	{
		return _x(X::asl) - _x(X::terrain_asl);
	}
	inline float getAltAboveOrigin() const
	{
		return _x(X::asl) - _origin.getAlt();
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
		return Quatf(_x(X::q_nb_0), _x(X::q_nb_1),
			     _x(X::q_nb_2), _x(X::q_nb_3));
	}
	inline Dcmf computeDcmNB() const
	{
		return getQuaternionNB();
	}
	inline Vector3f getGyroBiasFrameB() const
	{
		return Vector3f(_x(X::gyro_bias_bX), _x(X::gyro_bias_bY), _x(X::gyro_bias_bZ));
	}
	inline Vector3f getGyroRawFrameB() const
	{
		return Vector3f(_u(U::omega_nb_bX), _u(U::omega_nb_bY), _u(U::omega_nb_bZ));
	}
	inline Vector3f getAngularVelocityNBFrameB() const
	{
		return getGyroRawFrameB() - getGyroBiasFrameB();
	}
	inline Vector3f getGroundVelocity() const
	{
		return Vector3f(_x(X::vel_N), _x(X::vel_E), _x(X::vel_D));
	}
	inline Vector3f getAccelerationFrameB() const
	{
		Vector3f a_b(_u(U::accel_bX), _u(U::accel_bY), _u(U::accel_bZ));
		Vector3f a_bias_b(_x(X::accel_bias_bX), _x(X::accel_bias_bY), _x(X::accel_bias_bZ));
		return a_b - a_bias_b;
	}
	inline void nullAltitudeCorrection(Vector<float, Xe::n> &dxe)
	{
		dxe(Xe::vel_D) = 0;
		dxe(Xe::asl) = 0;
		dxe(Xe::terrain_asl) = 0;
		dxe(Xe::accel_bias_D) = 0;
	}
	inline void nullPositionCorrection(Vector<float, Xe::n> &dxe)
	{
		dxe(Xe::vel_N) = 0;
		dxe(Xe::vel_E) = 0;
		dxe(Xe::vel_D) = 0;
		dxe(Xe::pos_N) = 0;
		dxe(Xe::pos_E) = 0;
		dxe(Xe::asl) = 0;
		dxe(Xe::terrain_asl) = 0;
		dxe(Xe::accel_bias_N) = 0;
		dxe(Xe::accel_bias_E) = 0;
		dxe(Xe::accel_bias_D) = 0;
	}
	inline void nullAttitudeCorrection(Vector<float, Xe::n> &dxe)
	{
		dxe(Xe::rot_N) = 0;
		dxe(Xe::rot_E) = 0;
		dxe(Xe::rot_D) = 0;
		dxe(Xe::gyro_bias_N) = 0;
		dxe(Xe::gyro_bias_E) = 0;
		dxe(Xe::gyro_bias_D) = 0;
	}

private:
	IEKF(const IEKF &other);
	const IEKF &operator=(const IEKF &other);

	// sensors
	Sensor _sensorAccel;
	Sensor _sensorMag;
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
	SubscriptionTiny _subImu;
	SubscriptionTiny _subGps;
	SubscriptionTiny _subAirspeed;
	SubscriptionTiny _subFlow;
	SubscriptionTiny _subDistance;
	SubscriptionTiny _subVision;
	SubscriptionTiny _subMocap;
	SubscriptionTiny _subLand;
	SubscriptionTiny _subParamUpdate;

	// publishers
	PublicationTiny _pubAttitude;
	PublicationTiny _pubLocalPosition;
	PublicationTiny _pubGlobalPosition;
	PublicationTiny _pubControlState;
	PublicationTiny _pubEstimatorStatus;

	// data
	Vector<float, X::n> _x0; 		// initial state vector
	Vector<float, X::n> _xMin; 		// lower bound vector
	Vector<float, X::n> _xMax; 		// upper bound vector
	Vector<float, Xe::n> _P0Diag; 	// initial state diagonal
	Vector<float, X::n> _x; 		// state vector
	SquareMatrix<float, Xe::n> _P; 	// covariance matrix
	Vector<float, U::n> _u; 		// input vector
	Vector3f _g_n; 					// expected gravity in navigation frame
	Origin _origin; 				// origin of local coordinate system
	float _baroAsl; 				// pressure altitude from baro
	float _baroOffset; 				// pressure offset
	bool _landed;					// are we landed?
	bool _freefall;					// are we freefalling?
	uint64_t _gpsUSec; 				// gps internal timestamp
	bool _attitudeInitialized;		// attitude initializtion complete
	uint64_t _stateTimestamp;		// state prediction timestamp
	uint64_t _covarianceTimestamp;; // covariance prediction timestamp
	int _imuLowRateIndex;
	bool _accelSaturated;
	bool _gyroSaturated;
	SquareMatrix<float, Xe::n> _A;
	SquareMatrix<float, Xe::n> _Q;
	Vector<float, Xe::n> _dxe; 		// 	error vector
	SquareMatrix<float, Xe::n> _dP; 	// change in covariance matrix used for checking
	Vector<float, Innov::n>  _innov;
	Vector<float, Innov::n>  _innovStd;
	uint16_t _overruns;

	// params
	BlockParamFloat _gyro_nd;
	BlockParamFloat _gyro_rw_nd;
	BlockParamFloat _gyro_rw_ct;
	BlockParamFloat _accel_nd;
	BlockParamFloat _accel_rw_nd;
	BlockParamFloat _accel_rw_ct;
	BlockParamFloat _baro_nd;
	BlockParamFloat _baro_rw_nd;
	BlockParamFloat _baro_rw_ct;
	BlockParamFloat _mag_nd;
	BlockParamFloat _mag_rw_nd;
	BlockParamFloat _mag_rw_ct;
	BlockParamFloat _mag_decl_deg;
	BlockParamFloat _gps_xy_nd;
	BlockParamFloat _gps_z_nd;
	BlockParamFloat _gps_vxy_nd;
	BlockParamFloat _gps_vz_nd;
	BlockParamFloat _vision_xy_nd;
	BlockParamFloat _vision_z_nd;
	BlockParamFloat _vision_vxy_nd;
	BlockParamFloat _vision_vz_nd;
	BlockParamFloat _flow_nd;
	BlockParamFloat _lidar_nd;
	BlockParamFloat _sonar_nd;
	BlockParamFloat _land_vxy_nd;
	BlockParamFloat _land_vz_nd;
	BlockParamFloat _land_agl_nd;
	BlockParamFloat _pn_xy_nd;
	BlockParamFloat _pn_vxy_nd;
	BlockParamFloat _pn_z_nd;
	BlockParamFloat _pn_vz_nd;
	BlockParamFloat _pn_rot_nd;
	BlockParamFloat _pn_t_asl_nd;
	BlockParamFloat _pn_t_asl_s_nd;
	BlockParamFloat _rate_accel;
	BlockParamFloat _rate_mag;
	BlockParamFloat _rate_baro;
	BlockParamFloat _rate_gps;
	BlockParamFloat _rate_airspeed;
	BlockParamFloat _rate_flow;
	BlockParamFloat _rate_sonar;
	BlockParamFloat _rate_lidar;
	BlockParamFloat _rate_vision;
	BlockParamFloat _rate_mocap;
	BlockParamFloat _rate_land;
};
