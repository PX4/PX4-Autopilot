// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.

#include "AnelloParser.hpp"

#include <cerrno>
#include <cstring>
#include <lib/geo/geo.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace time_literals;

class Anello : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;
	explicit Anello(const char *port) :
		ModuleParams(nullptr),
		ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
		_serial(port, _param_baud.get()),
		_attitude_pub(_param_mode.get() == 1 ? ORB_ID(vehicle_attitude) : ORB_ID(external_ins_attitude)),
		_local_pub(_param_mode.get() == 1 ? ORB_ID(vehicle_local_position) : ORB_ID(external_ins_local_position)),
		_global_pub(_param_mode.get() == 1 ? ORB_ID(vehicle_global_position) : ORB_ID(external_ins_global_position))
	{
	}

	~Anello() override { ScheduleClear(); _serial.close(); }
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int, char *[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);

	int print_status() override
	{
		const hrt_abstime now = hrt_absolute_time();
		PX4_INFO("port: %s, baud: %u, mode: %s", _serial.getPort(), (unsigned)_serial.getBaudrate(),
			 _param_mode.get() == 1 ? "primary" : "monitor");
		PX4_INFO("attitude: %s, position: %s, uncertainty configured: %s",
			 _parser.attitudeFresh(now) ? "fresh" : "missing/stale",
			 _parser.positionFresh(now) ? "fresh" : "missing/stale", accuracyConfigured() ? "yes" : "no");
		PX4_INFO("read cycles: %u, received bytes: %u", (unsigned)_cycles, (unsigned)_bytes);
		PX4_INFO("read errors: %u, last errno: %d", (unsigned)_read_errors, _last_read_errno);
		PX4_INFO("accepted attitude: %u, position: %u", (unsigned)_attitude_count, (unsigned)_position_count);
		return PX4_OK;
	}

private:
	bool init()
	{
		if (_param_mode.get() == 1) {
			if (!parameterEquals("EKF2_EN", 0) || !parameterEquals("ATT_EN", 0)
			    || !parameterEquals("SENS_IMU_MODE", 1)) {
				PX4_ERR("primary requires EKF2_EN=0, ATT_EN=0, SENS_IMU_MODE=1 and reboot");
				return false;
			}

			// Parameters alone do not stop an already running estimator.
			if (orb_exists(ORB_ID(vehicle_attitude), 0) == PX4_OK || orb_exists(ORB_ID(vehicle_local_position), 0) == PX4_OK
			    || orb_exists(ORB_ID(vehicle_global_position), 0) == PX4_OK) {
				PX4_ERR("vehicle estimate already advertised; reboot without other estimators");
				return false;
			}
		}

		// NuttX file descriptors belong to a task group. Open the UART in Run(),
		// in the same work-queue context that reads and closes it.
		ScheduleNow();
		return true;
	}

	static bool parameterEquals(const char *name, int32_t expected)
	{
		const param_t handle = param_find_no_notification(name);
		int32_t value = 0;
		// An absent optional estimator enable parameter means it is not built.
		return handle == PARAM_INVALID ? expected == 0 : param_get(handle, &value) == 0 && value == expected;
	}

	bool accuracyConfigured() const
	{
		return PX4_ISFINITE(_param_eph.get()) && _param_eph.get() > 0.f
		       && PX4_ISFINITE(_param_epv.get()) && _param_epv.get() > 0.f
		       && PX4_ISFINITE(_param_ev.get()) && _param_ev.get() > 0.f;
	}

	void Run() override
	{
		if (should_exit()) {
			ScheduleClear();
			exit_and_cleanup(desc);
			return;
		}

		if (!_serial.isOpen()) {
			if (!_serial.open()) {
				PX4_ERR("cannot open %s (errno %d)", _serial.getPort(), errno);
				// Serial::open can retain a descriptor if configuration fails.
				_serial.close();
				ScheduleClear();
				exit_and_cleanup(desc);
				return;
			}

			_serial.flush();
			ScheduleOnInterval(5_ms);
		}

		++_cycles;

		// Bound work even if a sender continuously fills the UART.
		uint8_t buffer[256];
		constexpr size_t MaxBytesPerRun = 4096;
		bool position_updated = false;

		for (size_t read_total = 0; read_total < MaxBytesPerRun;) {
			const ssize_t count = _serial.read(buffer, sizeof(buffer));

			if (count <= 0) {
				if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
					_last_read_errno = errno;
					++_read_errors;
				}

				break;
			}

			read_total += count;
			_bytes += count;
			const hrt_abstime now = hrt_absolute_time();

			for (ssize_t i = 0; i < count; ++i) {
				switch (_parser.parse(buffer[i], now)) {
				case anello::Parser::Update::Attitude: {
						vehicle_attitude_s attitude{};
						attitude.timestamp_sample = _parser.attitude_time;
						_parser.attitude.copyTo(attitude.q);
						attitude.timestamp = now;
						_attitude_pub.publish(attitude);
						_attitude_valid = true;
						++_attitude_count;
						break;
					}

				case anello::Parser::Update::Position:
					position_updated = true;
					++_position_count;
					break;

				case anello::Parser::Update::None:
					break;
				}
			}
		}

		const hrt_abstime now = hrt_absolute_time();
		const bool attitude_fresh = _parser.attitudeFresh(now);

		if (_attitude_valid && !attitude_fresh) {
			// A zero quaternion explicitly invalidates attitude at our 250 ms deadline.
			vehicle_attitude_s invalid{};
			invalid.timestamp_sample = _parser.attitude_time;
			invalid.timestamp = now;
			_attitude_pub.publish(invalid);
			_attitude_valid = false;
		}

		const bool navigation_valid = attitude_fresh && _parser.positionFresh(now) && accuracyConfigured();

		if (position_updated || navigation_valid != _navigation_valid) {
			publishPosition(now, navigation_valid);
			_navigation_valid = navigation_valid;
		}

		if (_param_mode.get() == 1 && now - _last_status >= 100_ms) {
			publishStatus(now, attitude_fresh, navigation_valid);
			_last_status = now;
		}
	}

	void publishPosition(hrt_abstime now, bool valid)
	{
		if (!_reference.isInitialized() && _parser.positionFresh(now) && _parser.attitudeFresh(now)) {
			_reference.initReference(_parser.latitude, _parser.longitude, now);
			_reference_altitude = _parser.altitude;
		}

		vehicle_local_position_s local{};
		local.timestamp_sample = _parser.position_time;
		local.xy_valid = local.z_valid = local.v_xy_valid = local.v_z_valid = valid && _reference.isInitialized();

		if (_reference.isInitialized()) {
			const matrix::Vector2f xy = _reference.project(_parser.latitude, _parser.longitude);
			local.x = xy(0);
			local.y = xy(1);
			local.z = _reference_altitude - _parser.altitude;
			local.xy_global = local.z_global = true;
			local.ref_timestamp = _reference.getProjectionReferenceTimestamp();
			local.ref_lat = _reference.getProjectionReferenceLat();
			local.ref_lon = _reference.getProjectionReferenceLon();
			local.ref_alt = _reference_altitude;
		}

		local.vx = _parser.velocity(0);
		local.vy = _parser.velocity(1);
		local.vz = local.z_deriv = _parser.velocity(2);
		local.ax = local.ay = local.az = NAN; // No acceleration in this protocol.
		local.heading = matrix::Eulerf(_parser.attitude).psi();
		local.heading_good_for_control = _parser.attitudeFresh(now);
		local.heading_var = local.tilt_var = local.unaided_heading = NAN;
		local.dist_bottom = local.dist_bottom_var = NAN;
		local.eph = accuracyConfigured() ? _param_eph.get() : INFINITY;
		local.epv = accuracyConfigured() ? _param_epv.get() : INFINITY;
		local.evh = local.evv = accuracyConfigured() ? _param_ev.get() : INFINITY;
		local.vxy_max = local.vz_max = local.hagl_min = local.hagl_max_z = local.hagl_max_xy = INFINITY;
		local.timestamp = now;
		_local_pub.publish(local);

		vehicle_global_position_s global{};
		global.timestamp_sample = _parser.position_time;
		global.lat = _parser.latitude;
		global.lon = _parser.longitude;
		global.alt = _parser.altitude;
		global.alt_ellipsoid = global.terrain_alt = NAN;
		global.lat_lon_valid = global.alt_valid = local.xy_valid;
		global.eph = local.eph;
		global.epv = local.epv;
		global.timestamp = now;
		_global_pub.publish(global);
	}

	void publishStatus(hrt_abstime now, bool attitude_valid, bool navigation_valid)
	{
		estimator_status_s status{};
		status.timestamp_sample = _parser.attitude_time;
		// Unknown innovation tests are never presented as measured zeroes.
		status.hdg_test_ratio = status.vel_test_ratio = status.pos_test_ratio = status.hgt_test_ratio = NAN;
		status.tas_test_ratio = status.hagl_test_ratio = status.beta_test_ratio = NAN;
		status.pos_horiz_accuracy = accuracyConfigured() ? _param_eph.get() : INFINITY;
		status.pos_vert_accuracy = accuracyConfigured() ? _param_epv.get() : INFINITY;
		status.pre_flt_fail_innov_heading = !attitude_valid;
		status.pre_flt_fail_innov_height = !navigation_valid;
		status.pre_flt_fail_innov_pos_horiz = !navigation_valid;
		status.pre_flt_fail_innov_vel_horiz = !navigation_valid;
		status.pre_flt_fail_innov_vel_vert = !navigation_valid;
		// estimator_status solution bits: attitude, horizontal/vertical velocity,
		// relative/absolute horizontal position, absolute vertical position.
		constexpr uint16_t AttitudeValid = 1u;
		constexpr uint16_t NavigationValid = (1u << 1) | (1u << 2) | (1u << 3) | (1u << 4) | (1u << 5);
		status.solution_status_flags = (attitude_valid ? AttitudeValid : 0) | (navigation_valid ? NavigationValid : 0);
		// Do not claim GNSS fusion, sensor IDs or alignment information the input does not carry.
		status.timestamp = now;
		_status_pub.publish(status);
	}

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::ANELLO_MODE>) _param_mode,
		(ParamInt<px4::params::ANELLO_BAUD>) _param_baud,
		(ParamFloat<px4::params::ANELLO_EPH>) _param_eph,
		(ParamFloat<px4::params::ANELLO_EPV>) _param_epv,
		(ParamFloat<px4::params::ANELLO_EV>) _param_ev
	)
	device::Serial _serial;
	anello::Parser _parser;
	MapProjection _reference;
	float _reference_altitude{0.f};
	bool _attitude_valid{false};
	bool _navigation_valid{false};
	uint32_t _cycles{0};
	uint32_t _bytes{0};
	uint32_t _read_errors{0};
	int _last_read_errno{0};
	uint32_t _attitude_count{0};
	uint32_t _position_count{0};
	hrt_abstime _last_status{0};
	uORB::Publication<vehicle_attitude_s> _attitude_pub;
	uORB::Publication<vehicle_local_position_s> _local_pub;
	uORB::Publication<vehicle_global_position_s> _global_pub;
	uORB::Publication<estimator_status_s> _status_pub{ORB_ID(estimator_status)};
};

ModuleBase::Descriptor Anello::desc{task_spawn, custom_command, print_usage};

int Anello::task_spawn(int argc, char *argv[])
{
	int index = 1;
	const char *argument = nullptr;
	const char *port = nullptr;
	int option;

	while ((option = px4_getopt(argc, argv, "d:", &index, &argument)) != EOF) {
		if (option != 'd') {
			return print_usage("invalid argument");
		}

		port = argument;
	}

	if (!port) {
		return print_usage("serial device required");
	}

	Anello *instance = new Anello(port);

	if (!instance) {
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	if (!instance->init()) {
		delete instance;
		desc.object.store(nullptr);
		desc.task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int Anello::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION("ANELLO Maritime INS: receive fused MAVLink navigation on a dedicated serial port.");
	PRINT_MODULE_USAGE_NAME("anello", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("ins");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return reason ? PX4_ERROR : PX4_OK;
}

extern "C" __EXPORT int anello_main(int argc, char *argv[])
{
	return ModuleBase::main(Anello::desc, argc, argv);
}
