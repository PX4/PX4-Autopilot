/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#pragma once

#include "Common.hpp"

#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/health_report.h>
#include <uORB/topics/failsafe_flags.h>

#include "checks/accelerometerCheck.hpp"
#include "checks/airspeedCheck.hpp"
#include "checks/armPermissionCheck.hpp"
#include "checks/baroCheck.hpp"
#include "checks/cpuResourceCheck.hpp"
#include "checks/distanceSensorChecks.hpp"
#include "checks/opticalFlowCheck.hpp"
#include "checks/escCheck.hpp"
#include "checks/estimatorCheck.hpp"
#include "checks/failureDetectorCheck.hpp"
#include "checks/navigatorCheck.hpp"
#include "checks/gyroCheck.hpp"
#include "checks/imuConsistencyCheck.hpp"
#include "checks/loggerCheck.hpp"
#include "checks/magnetometerCheck.hpp"
#include "checks/manualControlCheck.hpp"
#include "checks/homePositionCheck.hpp"
#include "checks/modeCheck.hpp"
#include "checks/parachuteCheck.hpp"
#include "checks/powerCheck.hpp"
#include "checks/rcCalibrationCheck.hpp"
#include "checks/sdcardCheck.hpp"
#include "checks/systemCheck.hpp"
#include "checks/batteryCheck.hpp"
#include "checks/windCheck.hpp"
#include "checks/geofenceCheck.hpp"
#include "checks/flightTimeCheck.hpp"
#include "checks/missionCheck.hpp"
#include "checks/rcAndDataLinkCheck.hpp"
#include "checks/vtolCheck.hpp"
#include "checks/offboardCheck.hpp"
#include "checks/openDroneIDCheck.hpp"
#include "checks/externalChecks.hpp"

class HealthAndArmingChecks : public ModuleParams
{
public:
	HealthAndArmingChecks(ModuleParams *parent, vehicle_status_s &status);
	~HealthAndArmingChecks() = default;

	/**
	 * Run arming checks and report if necessary.
	 * This should be called regularly (e.g. 1Hz).
	 * @param force_reporting if true, force reporting even if nothing changed
	 * @param is_arming_request if true, then we are running the checks based on an actual arming request
	 * @return true if there was a report (also when force_reporting=true)
	 */
	bool update(bool force_reporting = false, bool is_arming_request = false);

	bool reportIfUnreportedDifferences();

	/**
	 * Whether arming is possible for a given navigation mode
	 */
	bool canArm(uint8_t nav_state) const { return _reporter.canArm(nav_state); }

	/**
	 * Whether switching into a given navigation mode is possible
	 */
	bool canRun(uint8_t nav_state) const { return _reporter.canRun(nav_state); }

	/**
	 * Query the mode requirements: check if a mode prevents arming
	 */
	bool modePreventsArming(uint8_t nav_state) const { return _reporter.modePreventsArming(nav_state); }

	const failsafe_flags_s &failsafeFlags() const { return _failsafe_flags; }

#ifndef CONSTRAINED_FLASH
	ExternalChecks &externalChecks() { return _external_checks; }
#endif

protected:
	void updateParams() override;
private:
	failsafe_flags_s _failsafe_flags{};

	Context _context;
	Report _reporter{_failsafe_flags};
	orb_advert_t _mavlink_log_pub{nullptr};

	uORB::Publication<health_report_s> _health_report_pub{ORB_ID(health_report)};
	uORB::Publication<failsafe_flags_s> _failsafe_flags_pub{ORB_ID(failsafe_flags)};

	// all checks
	AccelerometerChecks _accelerometer_checks;
	AirspeedChecks _airspeed_checks;
	ArmPermissionChecks _arm_permission_checks;
	BaroChecks _baro_checks;
	CpuResourceChecks _cpu_resource_checks;
	DistanceSensorChecks _distance_sensor_checks;
	OpticalFlowCheck _optical_flow_check;
	EscChecks _esc_checks;
	EstimatorChecks _estimator_checks;
	FailureDetectorChecks _failure_detector_checks;
	NavigatorChecks _navigator_checks;
	GyroChecks _gyro_checks;
	ImuConsistencyChecks _imu_consistency_checks;
	LoggerChecks _logger_checks;
	MagnetometerChecks _magnetometer_checks;
	ManualControlChecks _manual_control_checks;
	HomePositionChecks _home_position_checks;
	ModeChecks _mode_checks;
	OpenDroneIDChecks _open_drone_id_checks;
	ParachuteChecks _parachute_checks;
	PowerChecks _power_checks;
	RcCalibrationChecks _rc_calibration_checks;
	SdCardChecks _sd_card_checks;
	SystemChecks _system_checks;
	BatteryChecks _battery_checks;
	WindChecks _wind_checks;
	GeofenceChecks _geofence_checks;
	FlightTimeChecks _flight_time_checks;
	MissionChecks _mission_checks;
	RcAndDataLinkChecks _rc_and_data_link_checks;
	VtolChecks _vtol_checks;
	OffboardChecks _offboard_checks;
#ifndef CONSTRAINED_FLASH
	ExternalChecks _external_checks;
#endif

	HealthAndArmingCheckBase *_checks[40] = {
#ifndef CONSTRAINED_FLASH
		&_external_checks,
#endif
		&_accelerometer_checks,
		&_airspeed_checks,
		&_arm_permission_checks,
		&_baro_checks,
		&_cpu_resource_checks,
		&_distance_sensor_checks,
		&_optical_flow_check,
		&_esc_checks,
		&_estimator_checks,
		&_failure_detector_checks,
		&_navigator_checks,
		&_gyro_checks,
		&_imu_consistency_checks,
		&_logger_checks,
		&_magnetometer_checks,
		&_manual_control_checks,
		&_home_position_checks,
		&_mission_checks,
		&_offboard_checks, // must be after _estimator_checks
		&_mode_checks, // must be after _estimator_checks, _home_position_checks, _mission_checks, _offboard_checks, _external_checks
		&_open_drone_id_checks,
		&_parachute_checks,
		&_power_checks,
		&_rc_calibration_checks,
		&_sd_card_checks,
		&_system_checks, // must be after _estimator_checks & _home_position_checks
		&_battery_checks,
		&_wind_checks,
		&_geofence_checks, // must be after _home_position_checks
		&_flight_time_checks,
		&_rc_and_data_link_checks,
		&_vtol_checks,
	};
};
