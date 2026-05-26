/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file SimulatorXPlane.hpp
 *
 * Native X-Plane UDP simulator backend for PX4 SITL.
 * Speaks X-Plane's DATA@ / RREF / DREF protocol directly — no plugin needed.
 *
 * Channel mapping is loaded from a JSON file at runtime:
 *   etc/init.d-posix/models/<model_name>.json
 *
 * JSON format (ArduCopter-compatible, # line comments allowed):
 *   {
 *     "settings": { "debug": 0 },
 *     "sim/operation/override/override_joystick": { "type": "fixed", "value": 1 },
 *     "sim/flightmodel/engine/ENGN_thro_use[0]": { "type": "range", "channel": 0, "range": 1.0 },
 *     "sim/flightmodel/controls/wing1l_ail1def": { "type": "angle", "channel": 1, "range": 20.0 },
 *     "sim/flightmodel/engine/ENGN_running[0]":  { "type": "running", "channel": 0 }
 *   }
 *
 * Types:
 *   fixed   – always send "value" (for X-Plane overrides)
 *   range   – send output[channel] * range, clamped [0, range]  (motors)
 *   angle   – send output[channel] * range  (servos, output is -1..1)
 *   running – send 1 when armed & output[channel] > 0.01, else 0
 *
 * Channels are 0-indexed, matching actuator_outputs.output[channel].
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include <netinet/in.h>

using namespace time_literals;

// X-Plane DATA@ group identifiers
enum XPlaneGroup : uint32_t {
	XP_GRP_TIMES    = 1,
	XP_GRP_GLOAD    = 4,
	XP_GRP_ANG_VEL  = 16,
	XP_GRP_ATTITUDE = 17,
	XP_GRP_LLA      = 20,
	XP_GRP_VEL      = 21,
};

// RREF subscription codes (chosen by us; returned verbatim in RREF responses)
enum XPlaneRrefCode : uint32_t {
	RREF_VERSION = 1,  // sim/version/xplane_internal_version  (e.g. 110000 = XP11, 120000 = XP12)
	RREF_PRAD    = 2,  // sim/flightmodel/position/Prad  [rad/s]
	RREF_QRAD    = 3,  // sim/flightmodel/position/Qrad  [rad/s]
	RREF_RRAD    = 4,  // sim/flightmodel/position/Rrad  [rad/s]
	RREF_GAXIL   = 5,  // sim/flightmodel/forces/g_axil  [g, fore-aft]
	RREF_GSIDE   = 6,  // sim/flightmodel/forces/g_side  [g, left-right]
	RREF_GNRML   = 7,  // sim/flightmodel/forces/g_nrml  [g, normal/up]
	RREF_VX      = 8,  // sim/flightmodel/position/local_vx  [m/s, east]
	RREF_VY      = 9,  // sim/flightmodel/position/local_vy  [m/s, up]
	RREF_VZ      = 10, // sim/flightmodel/position/local_vz  [m/s, south]
	RREF_LAT     = 11, // sim/flightmodel/position/latitude  [deg]
	RREF_LON     = 12, // sim/flightmodel/position/longitude [deg]
	RREF_ALT     = 13, // sim/flightmodel/position/elevation [m MSL]
	RREF_THETA   = 14, // sim/flightmodel/position/theta  [deg pitch]
	RREF_PHI     = 15, // sim/flightmodel/position/phi    [deg roll]
	RREF_PSI     = 16, // sim/flightmodel/position/psi    [deg true heading]
};

static constexpr float GRAVITY_MSS = 9.80665f;
static constexpr float FEET_TO_M   = 0.3048f;

// Earth magnetic field NED is computed at runtime from the WMM using the
// vehicle's actual GPS position (see SimulatorXPlane::update_mag_earth()).

// ── DREF map entry ────────────────────────────────────────────────────────────

struct DRefEntry {
	char  name[256];

	enum class Type : uint8_t {
		FIXED   = 0,   // always send 'value'
		RANGE   = 1,   // output[ch] * scale, clamped [0, scale]
		ANGLE   = 2,   // output[ch] * scale, output is -1..1
		RUNNING = 3,   // 1 if armed & output[ch] > 0.01, else 0
	} type{Type::FIXED};

	int8_t  channel{-1};   // actuator_outputs.output[] index, -1 for FIXED
	float   scale{1.0f};   // range / multiplier
	float   value{0.0f};   // for FIXED type

	DRefEntry *next{nullptr};
};


class SimulatorXPlane final : public ModuleParams
{
public:
	static int start(int argc, char *argv[]);

	~SimulatorXPlane();

private:
	SimulatorXPlane(const char *xplane_ip, uint16_t xplane_port,
			uint16_t bind_port, const char *model_name);

	void run();
	void send();
	static void *sending_trampoline(void *);

	// ── UDP helpers ──────────────────────────────────────────────────────────
	void udp_send(const void *data, size_t len);
	void send_dref(const char *name, float value);
	void send_dsel();
	void send_rref_subscribe();

	// ── DREF map (loaded from JSON) ──────────────────────────────────────────
	bool  load_dref_map(const char *model_name);
	void  free_dref_map();
	void  send_fixed_drefs();      // send FIXED entries (overrides, re-sent periodically)
	void  send_actuator_drefs();   // send RANGE/ANGLE/RUNNING entries from actuator outputs

	// ── X-Plane packet parsers ───────────────────────────────────────────────
	void handle_rref(const uint8_t *pkt, size_t len);
	void handle_data_at(const uint8_t *pkt, size_t len);
	void apply_data_group(uint32_t gid, const float *d);

	// ── Sensor publishing ────────────────────────────────────────────────────
	void publish_imu(hrt_abstime t);
	void publish_mag(hrt_abstime t);
	void publish_baro(hrt_abstime t);
	void publish_gps(hrt_abstime t);
	void mag_body(float &bx, float &by, float &bz) const;
	void update_mag_earth();   // refresh earth-frame mag NED from GPS via WMM

	// ── Simulated sensors ────────────────────────────────────────────────────
	PX4Accelerometer _px4_accel{1310988, ROTATION_NONE};
	PX4Gyroscope     _px4_gyro {1310988, ROTATION_NONE};
	PX4Magnetometer  _px4_mag  {197388,  ROTATION_NONE};

	uORB::PublicationMulti<sensor_baro_s> _baro_pub{ORB_ID(sensor_baro)};
	uORB::PublicationMulti<sensor_gps_s>  *_gps_pub{nullptr};

	// ── Actuator subscription ────────────────────────────────────────────────
	int                _actuator_sub{-1};
	actuator_outputs_s _actuator_outputs{};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	vehicle_status_s   _vehicle_status{};

	// ── UDP socket ───────────────────────────────────────────────────────────
	int                _fd{-1};
	struct sockaddr_in _xplane_addr{};
	char               _xplane_ip[32]{};
	uint16_t           _xplane_port{49000};
	uint16_t           _bind_port{49005};

	// ── DREF map ─────────────────────────────────────────────────────────────
	DRefEntry *_drefs{nullptr};       // linked list of all entries
	int        _n_drefs{0};
	bool       _debug_map{false};

	// ── X-Plane version ──────────────────────────────────────────────────────
	uint32_t _xplane_version{0};   // e.g. 110000 = XP11, 120100 = XP12.1
	bool is_xplane12() const { return _xplane_version / 10000 >= 12; }

	// ── Sensor state ─────────────────────────────────────────────────────────
	float _gyro_x{0}, _gyro_y{0}, _gyro_z{0};
	float _accel_x{0}, _accel_y{0}, _accel_z{-GRAVITY_MSS};
	float _pitch_rad{0}, _roll_rad{0}, _yaw_rad{0};
	float _lat_deg{0}, _lon_deg{0}, _alt_m{0};
	float _vel_n{0}, _vel_e{0}, _vel_d{0};

	// Earth magnetic field in NED (Gauss). Refreshed from GPS via WMM when the
	// vehicle moves significantly. Zeroed until first position fix.
	float _mag_ned_n{0.f}, _mag_ned_e{0.f}, _mag_ned_d{0.f};
	float _mag_earth_lat{0.f}, _mag_earth_lon{0.f};   // last position used to compute mag earth
	bool  _mag_earth_valid{false};

	// Gyro bias compensation. X-Plane's Prad/Qrad/Rrad can report sustained
	// nonzero body rates for an aircraft that is visually stationary (model
	// physics quirk on the ground; we measured ~-16°/s on Z for ehang184).
	//
	// Real drones calibrate gyro bias during boot when known stationary. We
	// do the same per-axis: accumulate first N samples, compute the mean,
	// then subtract from every subsequent sample.
	//
	// Variance gate: if the std-dev of a calibration window exceeds
	// GYRO_BIAS_STDDEV_LIMIT, the aircraft was clearly moving — discard the
	// window and restart accumulation. Prevents the failure where SITL is
	// restarted while a previous crash still tumbles the airframe and the
	// "bias" absorbs the actual rotation.
	// 500 samples ≈ 6.25 s at 80 Hz. 200 samples (2.5 s) sometimes locked on a
	// momentary lull when stationary X-Plane gyro oscillates around its true
	// bias — observed e.g. Z=−0.000 lock followed by visible heading drift.
	// Longer window averages over more oscillation cycles → mean tracks the
	// real bias.
	static constexpr int   GYRO_BIAS_SAMPLES       = 500;     // ~6.25 s at 80 Hz RREF
	static constexpr float GYRO_BIAS_STDDEV_LIMIT  = 0.05f;   // rad/s ≈ 2.9°/s
	float _gyro_bias_x{0.f},     _gyro_bias_y{0.f},     _gyro_bias_z{0.f};
	float _gyro_bias_sum_x{0.f}, _gyro_bias_sum_y{0.f}, _gyro_bias_sum_z{0.f};
	float _gyro_bias_sum_sq_x{0.f}, _gyro_bias_sum_sq_y{0.f}, _gyro_bias_sum_sq_z{0.f};
	int   _gyro_bias_count_x{0}, _gyro_bias_count_y{0}, _gyro_bias_count_z{0};
	bool  _gyro_bias_locked_x{false}, _gyro_bias_locked_y{false}, _gyro_bias_locked_z{false};
	bool  _gyro_bias_reported{false};
	uint32_t _gyro_bias_rejects{0};   // count of discarded windows

	// Accelerometer magnitude calibration. Mirrors the px4xplane plugin's
	// AccelCalibration: X-Plane aircraft models can report |accel| ≠ 1g when
	// stationary (most VTOL models offend; quad model less so). The fix is
	// MAGNITUDE SCALING — compute scale = expected_g / measured_|accel| while
	// stationary, then multiply every subsequent accel sample. Scaling
	// preserves direction so it works at all attitudes (offset subtraction
	// would only be correct at the calibration attitude).
	static constexpr int   ACCEL_CAL_SAMPLES        = 200;     // ~1 s at 200 Hz / 2.5 s at 80 Hz
	static constexpr int   ACCEL_CAL_WAIT_SAMPLES   = 50;      // settling before sampling
	static constexpr float ACCEL_CAL_STATIONARY_VEL = 0.5f;    // m/s ground-speed threshold
	float  _accel_scale_factor{1.0f};
	float  _accel_cal_sum_mag{0.f};
	int    _accel_cal_count{0};
	int    _accel_cal_stationary_count{0};
	bool   _accel_calibrated{false};

	// Sensor low-pass filter. X-Plane physics produces high-frequency gyro/
	// accel jitter (we measured σ≈17°/s on stationary models) that gets
	// faithfully republished without smoothing — drives EKF into unstable
	// states, leads to vehicle crashes. The px4xplane plugin uses a Kalman
	// filter on [value, velocity] state for this; we use a simpler 1st-order
	// IIR that achieves similar smoothing at lower computational cost.
	// α tradeoff: lower = smoother but more phase lag → controller computes
	// corrections on STALE attitude during fast maneuvers → vehicle diverges.
	// α=0.5 (50ms lag at 40Hz) was crashing on takeoff; α=0.8 keeps response
	// snappy (~11ms lag) while still attenuating the highest-frequency jitter.
	static constexpr float SENSOR_LPF_ALPHA = 0.8f;
	float _gyro_filt_x{0.f},  _gyro_filt_y{0.f},  _gyro_filt_z{0.f};
	float _accel_filt_x{0.f}, _accel_filt_y{0.f}, _accel_filt_z{-GRAVITY_MSS};
	bool  _sensor_filt_initialized{false};

	uint8_t _rref_gyro_mask{0};
	uint8_t _rref_accel_mask{0};
	uint8_t _rref_vel_mask{0};   // bits 0,1,2 = VX,VY,VZ received
	uint8_t _rref_pos_mask{0};   // bits 0,1,2 = LAT,LON,ALT received
	uint8_t _rref_att_mask{0};   // bits 0,1,2 = THETA,PHI,PSI received
	bool    _has_pos{false};
	bool    _has_vel{false};
	bool    _has_att{false};

	// ── Timing ───────────────────────────────────────────────────────────────
	hrt_abstime _t_last_dsel{0};
	hrt_abstime _t_last_rref{0};
	hrt_abstime _t_last_fixed{0};
	hrt_abstime _t_last_baro{0};
	hrt_abstime _t_last_gps{0};
	hrt_abstime _t_last_rate{0};

	// ── Frame rate counters ───────────────────────────────────────────────────
	uint32_t _rref_pkt_count{0};   // RREF packets received since last report
	uint32_t _data_pkt_count{0};   // DATA@ packets received since last report

	perf_counter_t _perf_interval{perf_alloc(PC_INTERVAL, MODULE_NAME": xplane interval")};
};
