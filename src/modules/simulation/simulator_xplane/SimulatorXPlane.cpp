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

#define MODULE_NAME "simulator_xplane"

#include "SimulatorXPlane.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>                          // CONSTANTS_ONE_G
#include <lib/world_magnetic_model/geo_mag_declination.h>
#include <drivers/device/Device.hpp>
#include <lib/drivers/device/Device.hpp>

#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

// Tiny white Gaussian noise (std=1, polar Box-Muller). Sensor publishes need a
// drop of noise so PX4's DataValidator does not flag the stream as STALE after
// 100 identical samples — which happens any time the aircraft is at rest on
// the ground (altitude, gyro, accel, mag all return the exact same float).
static float xplane_wgn()
{
	static float V1{0.f}, V2{0.f}, S{0.f};
	static bool  phase{true};

	if (phase) {
		float U1, U2;
		do {
			U1 = (float)rand() / (float)RAND_MAX;
			U2 = (float)rand() / (float)RAND_MAX;
			V1 = 2.f * U1 - 1.f;
			V2 = 2.f * U2 - 1.f;
			S  = V1 * V1 + V2 * V2;
		} while (S >= 1.f || fabsf(S) < 1e-8f);
		phase = false;
		return V1 * sqrtf(-2.f * logf(S) / S);
	}

	phase = true;
	return V2 * sqrtf(-2.f * logf(S) / S);
}

// ── X-Plane RREF dataref subscriptions ───────────────────────────────────────
// Indices match XPlaneRrefCode enum values (code = index + RREF_VERSION).

static const char *RREF_NAMES[] = {
	"sim/version/xplane_internal_version",  // code RREF_VERSION=1
	"sim/flightmodel/position/Prad",        // code RREF_PRAD=2
	"sim/flightmodel/position/Qrad",        // code RREF_QRAD=3
	"sim/flightmodel/position/Rrad",        // code RREF_RRAD=4
	"sim/flightmodel/forces/g_axil",        // code RREF_GAXIL=5
	"sim/flightmodel/forces/g_side",        // code RREF_GSIDE=6
	"sim/flightmodel/forces/g_nrml",        // code RREF_GNRML=7
	"sim/flightmodel/position/local_vx",    // code RREF_VX=8   (east, m/s)
	"sim/flightmodel/position/local_vy",    // code RREF_VY=9   (up,   m/s)
	"sim/flightmodel/position/local_vz",    // code RREF_VZ=10  (south,m/s)
	"sim/flightmodel/position/latitude",    // code RREF_LAT=11 (deg)
	"sim/flightmodel/position/longitude",   // code RREF_LON=12 (deg)
	"sim/flightmodel/position/elevation",   // code RREF_ALT=13 (m MSL)
	"sim/flightmodel/position/theta",       // code RREF_THETA=14 (pitch deg)
	"sim/flightmodel/position/phi",         // code RREF_PHI=15   (roll deg)
	"sim/flightmodel/position/psi",         // code RREF_PSI=16   (true heading deg)
};
static constexpr int N_RREF = sizeof(RREF_NAMES) / sizeof(RREF_NAMES[0]);

// DATA@ groups to request (position & velocity come via RREF instead)
static const uint32_t DATA_GROUPS[] = {
	XP_GRP_TIMES, XP_GRP_GLOAD, XP_GRP_ANG_VEL, XP_GRP_ATTITUDE,
};
static constexpr size_t N_DATA_GROUPS = sizeof(DATA_GROUPS) / sizeof(DATA_GROUPS[0]);

// ── Module state ──────────────────────────────────────────────────────────────
static int              g_task{-1};
static SimulatorXPlane *g_instance{nullptr};

// ════════════════════════════════════════════════════════════════════════════
// Minimal JSON parser for xplane map files
//
// Handles the ArduCopter-compatible format:
//   • # ... line comments (stripped before parsing)
//   • Top-level object: { "dref_name": { "field": value, ... }, ... }
//   • Field values: quoted strings or bare numbers (int / float)
// ════════════════════════════════════════════════════════════════════════════

struct MapFields {
	char   type[32];
	int    channel;
	float  range;
	float  value;
	bool   has_channel;
	bool   has_range;
	bool   has_value;
};

// Skip whitespace
static const char *skip_ws(const char *p)
{
	while (*p && isspace((unsigned char)*p)) { p++; }
	return p;
}

// Parse a JSON string into buf[buf_len].  p must point at the opening '"'.
// Returns pointer past the closing '"', or nullptr on error.
static const char *parse_string(const char *p, char *buf, size_t buf_len)
{
	if (*p != '"') { return nullptr; }
	p++;
	size_t i = 0;
	while (*p && *p != '"') {
		if (*p == '\\') { p++; } // skip escape prefix
		if (i + 1 < buf_len) { buf[i++] = *p; }
		p++;
	}
	if (*p != '"') { return nullptr; }
	buf[i] = '\0';
	return p + 1;
}

// Parse a JSON number (integer or float) into *out.
// Returns pointer past the number, or nullptr on error.
static const char *parse_number(const char *p, float *out)
{
	char *end = nullptr;
	*out = strtof(p, &end);
	return (end > p) ? end : nullptr;
}

// Parse one inner object { "field": value, ... } into MapFields.
// p must point at '{'.  Returns pointer past '}'.
static const char *parse_map_fields(const char *p, MapFields *f)
{
	memset(f, 0, sizeof(*f));
	f->channel = -1;

	if (*p != '{') { return nullptr; }
	p++;

	while (true) {
		p = skip_ws(p);
		if (!*p) { return nullptr; }
		if (*p == '}') { return p + 1; }
		if (*p == ',') { p++; continue; }

		// Parse field name
		char key[64];
		p = parse_string(p, key, sizeof(key));
		if (!p) { return nullptr; }

		p = skip_ws(p);
		if (*p != ':') { return nullptr; }
		p++;
		p = skip_ws(p);

		// Parse field value
		if (*p == '"') {
			char sval[64];
			p = parse_string(p, sval, sizeof(sval));
			if (!p) { return nullptr; }
			if (strcmp(key, "type") == 0) {
				strncpy(f->type, sval, sizeof(f->type) - 1);
			}
		} else {
			float num = 0.0f;
			p = parse_number(p, &num);
			if (!p) { return nullptr; }
			if      (strcmp(key, "channel") == 0) { f->channel = (int)num; f->has_channel = true; }
			else if (strcmp(key, "range")   == 0) { f->range   = num;      f->has_range   = true; }
			else if (strcmp(key, "value")   == 0) { f->value   = num;      f->has_value   = true; }
			else if (strcmp(key, "debug")   == 0) { /* handled by caller */ }
		}
	}
}

// ── load_dref_map ─────────────────────────────────────────────────────────────

bool SimulatorXPlane::load_dref_map(const char *model_name)
{
	// Build path: etc/init.d-posix/models/<model_name>.json
	char path[256];
	snprintf(path, sizeof(path), "etc/init.d-posix/models/%s.json", model_name);

	FILE *f = fopen(path, "r");
	if (!f) {
		PX4_WARN("DREF map not found: %s (using built-in defaults)", path);
		return false;
	}

	// Read entire file
	fseek(f, 0, SEEK_END);
	long fsize = ftell(f);
	fseek(f, 0, SEEK_SET);
	if (fsize <= 0 || fsize > 65536) { fclose(f); return false; }

	char *raw = (char *)malloc((size_t)fsize + 1);
	if (!raw) { fclose(f); return false; }
	size_t nread = fread(raw, 1, (size_t)fsize, f);
	raw[nread] = '\0';
	fclose(f);

	// Strip # line comments → replace with spaces so positions stay valid
	for (char *c = raw; *c; c++) {
		if (*c == '#') {
			while (*c && *c != '\n') { *c++ = ' '; }
		}
	}

	// Parse: expect top-level { "key": { ... }, ... }
	const char *p = skip_ws(raw);
	if (*p != '{') {
		PX4_ERR("DREF map: expected '{' at start of %s", path);
		free(raw);
		return false;
	}
	p++;

	int count = 0;

	while (true) {
		p = skip_ws(p);
		if (!*p) { break; }
		if (*p == '}') { break; }
		if (*p == ',') { p++; continue; }

		// Parse top-level key (dataref name or "settings")
		char dref_name[256];
		p = parse_string(p, dref_name, sizeof(dref_name));
		if (!p) { PX4_ERR("DREF map: bad key near char %ld", (long)(p - raw)); break; }

		p = skip_ws(p);
		if (*p != ':') { break; }
		p++;
		p = skip_ws(p);

		// Parse the value object
		MapFields fields{};
		p = parse_map_fields(p, &fields);
		if (!p) { PX4_ERR("DREF map: bad value for '%s'", dref_name); break; }

		// Skip the "settings" pseudo-entry (just read debug flag)
		if (strcmp(dref_name, "settings") == 0) {
			_debug_map = (fields.value > 0.0f) || (fields.value < 0.0f) || (fields.range > 0.0f) || (fields.range < 0.0f);
			continue;
		}

		// Build a DRefEntry
		DRefEntry *e = new DRefEntry{};
		strncpy(e->name, dref_name, sizeof(e->name) - 1);
		e->channel = (int8_t)fields.channel;

		if (strcmp(fields.type, "fixed") == 0) {
			e->type  = DRefEntry::Type::FIXED;
			e->value = fields.value;
			e->scale = 0.0f;
		} else if (strcmp(fields.type, "range") == 0) {
			e->type  = DRefEntry::Type::RANGE;
			e->scale = fields.has_range ? fields.range : 1.0f;
		} else if (strcmp(fields.type, "angle") == 0) {
			e->type  = DRefEntry::Type::ANGLE;
			e->scale = fields.has_range ? fields.range : 1.0f;
		} else if (strcmp(fields.type, "running") == 0) {
			e->type  = DRefEntry::Type::RUNNING;
			e->scale = 1.0f;
		} else {
			PX4_WARN("DREF map: unknown type '%s' for '%s', skipping", fields.type, dref_name);
			delete e;
			continue;
		}

		// Append to linked list
		e->next = _drefs;
		_drefs  = e;
		count++;

		if (_debug_map) {
			PX4_INFO("DREF: ch=%d type=%-8s scale=%.2f val=%.2f  %s",
				 e->channel, fields.type, (double)e->scale, (double)e->value, e->name);
		}
	}

	free(raw);
	_n_drefs = count;
	PX4_INFO("Loaded %d DREFs from %s", count, path);
	return count > 0;
}

void SimulatorXPlane::free_dref_map()
{
	DRefEntry *e = _drefs;
	while (e) {
		DRefEntry *next = e->next;
		delete e;
		e = next;
	}
	_drefs   = nullptr;
	_n_drefs = 0;
}

// ── constructor / destructor ─────────────────────────────────────────────────

SimulatorXPlane::SimulatorXPlane(const char *xplane_ip, uint16_t xplane_port,
				 uint16_t bind_port, const char *model_name)
	: ModuleParams(nullptr),
	  _xplane_port(xplane_port),
	  _bind_port(bind_port)
{
	strncpy(_xplane_ip, xplane_ip, sizeof(_xplane_ip) - 1);

	if (!load_dref_map(model_name)) {
		// Fall back to built-in quad defaults so the module is always usable
		PX4_INFO("Using built-in quad DREF defaults");
		const struct { const char *name; DRefEntry::Type type; int ch; float scale; float val; } defaults[] = {
			{ "sim/operation/override/override_joystick",      DRefEntry::Type::FIXED,   -1, 0.0f, 1.0f },
			{ "sim/operation/override/override_throttles",     DRefEntry::Type::FIXED,   -1, 0.0f, 1.0f },
			{ "sim/operation/override/override_flightcontrol", DRefEntry::Type::FIXED,   -1, 0.0f, 1.0f },
			{ "sim/flightmodel/engine/ENGN_running[0]",        DRefEntry::Type::RUNNING,  0, 1.0f, 0.0f },
			{ "sim/flightmodel/engine/ENGN_running[1]",        DRefEntry::Type::RUNNING,  1, 1.0f, 0.0f },
			{ "sim/flightmodel/engine/ENGN_running[2]",        DRefEntry::Type::RUNNING,  2, 1.0f, 0.0f },
			{ "sim/flightmodel/engine/ENGN_running[3]",        DRefEntry::Type::RUNNING,  3, 1.0f, 0.0f },
			{ "sim/flightmodel/engine/ENGN_thro_use[0]",       DRefEntry::Type::RANGE,    0, 1.0f, 0.0f },
			{ "sim/flightmodel/engine/ENGN_thro_use[1]",       DRefEntry::Type::RANGE,    1, 1.0f, 0.0f },
			{ "sim/flightmodel/engine/ENGN_thro_use[2]",       DRefEntry::Type::RANGE,    2, 1.0f, 0.0f },
			{ "sim/flightmodel/engine/ENGN_thro_use[3]",       DRefEntry::Type::RANGE,    3, 1.0f, 0.0f },
		};
		for (const auto &d : defaults) {
			DRefEntry *e = new DRefEntry{};
			strncpy(e->name, d.name, sizeof(e->name) - 1);
			e->type    = d.type;
			e->channel = (int8_t)d.ch;
			e->scale   = d.scale;
			e->value   = d.val;
			e->next    = _drefs;
			_drefs     = e;
			_n_drefs++;
		}
	}
}

SimulatorXPlane::~SimulatorXPlane()
{
	free_dref_map();
	if (_fd >= 0) { ::close(_fd); }
	if (_gps_pub) { delete _gps_pub; }
	perf_free(_perf_interval);
	g_instance = nullptr;
}

// ── UDP helpers ───────────────────────────────────────────────────────────────

void SimulatorXPlane::udp_send(const void *data, size_t len)
{
	if (_fd < 0) { return; }
	::sendto(_fd, data, len, 0, (struct sockaddr *)&_xplane_addr, sizeof(_xplane_addr));
}

void SimulatorXPlane::send_dref(const char *name, float value)
{
	// "DREF\0" (5) + float32_le (4) + 500-byte null-padded name = 509
	uint8_t pkt[510];
	strcpy((char *)pkt,     "DREF\0");
	memcpy(pkt + 5, &value, 4);
	memset(pkt + 9, 0, 500);
	size_t nlen = strlen(name);
	if (nlen > 499) { nlen = 499; }
	strcpy((char *)pkt + 9, name);
	udp_send(pkt, sizeof(pkt));
}

void SimulatorXPlane::send_dsel()
{
	uint8_t pkt[5 + N_DATA_GROUPS * 4];
	memcpy(pkt, "DSEL\0", 5);
	for (size_t i = 0; i < N_DATA_GROUPS; i++) {
		uint32_t gid = DATA_GROUPS[i];
		memcpy(pkt + 5 + i * 4, &gid, 4);
	}
	udp_send(pkt, sizeof(pkt));
}

void SimulatorXPlane::send_rref_subscribe()
{
	// "RREF\0"(5) + rate(4) + code(4) + name(400) = 413
	// code = RREF_VERSION + i  so the enum values are returned verbatim
	for (int i = 0; i < N_RREF; i++) {
		uint32_t rate = (i == 0) ? 1 : 100;   // version at 1 Hz, sensors at 100 Hz
		uint32_t code = (uint32_t)(RREF_VERSION + i);
		uint8_t  pkt[413];
		strcpy((char *)pkt,      "RREF\0");
		memcpy(pkt + 5,  &rate,    4);
		memcpy(pkt + 9,  &code,    4);
		memset(pkt + 13, 0,        400);
		size_t nlen = strlen(RREF_NAMES[i]);
		if (nlen > 399) { nlen = 399; }
		memcpy(pkt + 13, RREF_NAMES[i], nlen);
		udp_send(pkt, sizeof(pkt));
	}
}

// ── DREF map sending ──────────────────────────────────────────────────────────

void SimulatorXPlane::send_fixed_drefs()
{
	for (DRefEntry *e = _drefs; e; e = e->next) {
		if (e->type == DRefEntry::Type::FIXED) {
			send_dref(e->name, e->value);
		}
	}
}

void SimulatorXPlane::send_actuator_drefs()
{
	_vehicle_status_sub.update(&_vehicle_status);
	const bool armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

	for (DRefEntry *e = _drefs; e; e = e->next) {
		if (e->type == DRefEntry::Type::FIXED) { continue; }
		if (e->channel < 0 || e->channel >= (int)actuator_outputs_s::NUM_ACTUATOR_OUTPUTS) { continue; }

		const float out = _actuator_outputs.output[e->channel];
		float       val = 0.0f;

		switch (e->type) {
		case DRefEntry::Type::RANGE:
			// Motor output: 0..1 → 0..scale
			val = math::constrain(out * e->scale, 0.0f, e->scale);
			break;

		case DRefEntry::Type::ANGLE:
			// Servo output: -1..1 → -scale..+scale (degrees)
			val = out * e->scale;
			break;

		case DRefEntry::Type::RUNNING:
			val = (armed && out > 0.01f) ? 1.0f : 0.0f;
			break;

		default:
			break;
		}

		send_dref(e->name, val);
	}
}

// ── X-Plane packet parsers ────────────────────────────────────────────────────

void SimulatorXPlane::handle_rref(const uint8_t *pkt, size_t len)
{
	size_t off = 5;
	const hrt_abstime t = hrt_absolute_time();
	bool imu_updated = false;
	_rref_pkt_count++;

	while (off + 8 <= len) {
		uint32_t code;
		float    val;
		memcpy(&code, pkt + off,     4);
		memcpy(&val,  pkt + off + 4, 4);
		off += 8;

		switch (code) {
		case RREF_VERSION:
			if (_xplane_version == 0) {
				_xplane_version = (uint32_t)val;
				PX4_INFO("X-Plane version %u.%u detected",
					 _xplane_version / 10000,
					 (_xplane_version % 10000) / 100);
			} else {
				_xplane_version = (uint32_t)val;
			}
			break;
		case RREF_PRAD:  _gyro_x = val;                 _rref_gyro_mask  |= 0x1; imu_updated = true; break;
		case RREF_QRAD:  _gyro_y = val;                 _rref_gyro_mask  |= 0x2; imu_updated = true; break;
		case RREF_RRAD:  _gyro_z = val;                 _rref_gyro_mask  |= 0x4; imu_updated = true; break;
		case RREF_GAXIL: _accel_x = -val * GRAVITY_MSS; _rref_accel_mask |= 0x1; imu_updated = true; break;
		case RREF_GSIDE: _accel_y =  val * GRAVITY_MSS; _rref_accel_mask |= 0x2; imu_updated = true; break;
		case RREF_GNRML: _accel_z = -val * GRAVITY_MSS; _rref_accel_mask |= 0x4; imu_updated = true; break;

		// NED velocity from X-Plane local OpenGL frame: X=east, Y=up, Z=south
		case RREF_VX:  _vel_e  =  val; _rref_vel_mask |= 0x1; break;
		case RREF_VY:  _vel_d  = -val; _rref_vel_mask |= 0x2; break;
		case RREF_VZ:  _vel_n  = -val; _rref_vel_mask |= 0x4; break;

		// Position via RREF (avoids DATA@ group-20 slot-offset ambiguity)
		case RREF_LAT: _lat_deg = val; _rref_pos_mask |= 0x1; break;
		case RREF_LON: _lon_deg = val; _rref_pos_mask |= 0x2; break;
		case RREF_ALT: _alt_m   = val; _rref_pos_mask |= 0x4; break; // meters MSL already

		// Attitude via RREF — same approach as the px4xplane plugin (reads
		// theta/phi/psi as direct datarefs). Decouples us from X-Plane's
		// per-user "Data Output" group-17 configuration.
		case RREF_THETA: _pitch_rad = math::radians(val); _rref_att_mask |= 0x1; break;
		case RREF_PHI:   _roll_rad  = math::radians(val); _rref_att_mask |= 0x2; break;
		case RREF_PSI:   _yaw_rad   = math::radians(val); _rref_att_mask |= 0x4; break;

		default: break;
		}
	}

	if (_rref_att_mask == 0x7) {
		_has_att = true;
		publish_mag(t);
	}

	if (imu_updated && _rref_gyro_mask == 0x7 && _rref_accel_mask == 0x7) {
		// 1st-order IIR low-pass on IMU samples. Smooths out X-Plane physics
		// jitter that would otherwise destabilize the EKF.
		if (!_sensor_filt_initialized) {
			_gyro_filt_x  = _gyro_x;  _gyro_filt_y  = _gyro_y;  _gyro_filt_z  = _gyro_z;
			_accel_filt_x = _accel_x; _accel_filt_y = _accel_y; _accel_filt_z = _accel_z;
			_sensor_filt_initialized = true;
		} else {
			const float a = SENSOR_LPF_ALPHA;
			_gyro_filt_x  = a * _gyro_x  + (1.0f - a) * _gyro_filt_x;
			_gyro_filt_y  = a * _gyro_y  + (1.0f - a) * _gyro_filt_y;
			_gyro_filt_z  = a * _gyro_z  + (1.0f - a) * _gyro_filt_z;
			_accel_filt_x = a * _accel_x + (1.0f - a) * _accel_filt_x;
			_accel_filt_y = a * _accel_y + (1.0f - a) * _accel_filt_y;
			_accel_filt_z = a * _accel_z + (1.0f - a) * _accel_filt_z;
		}

		// Per-axis gyro bias estimation. Calibration runs while the vehicle
		// is stationary (groundspeed < threshold) — matches the px4xplane
		// plugin's AccelCalibration gating instead of using a variance gate.
		// X-Plane landing-gear physics produces large natural gyro jitter
		// (σ > 10°/s on some models) which any sensible variance gate would
		// reject; gating on groundspeed instead lets calibration converge.
		const bool vel_ok = (_rref_vel_mask == 0x7);
		const float gs = vel_ok ? sqrtf(_vel_n*_vel_n + _vel_e*_vel_e + _vel_d*_vel_d) : 1e9f;
		const bool stationary = vel_ok && (gs < ACCEL_CAL_STATIONARY_VEL);

		auto accumulate_axis = [&](float sample,
		                           float &sum, int &count,
		                           float &bias_out, bool &locked) {
			if (locked) { return; }
			if (!stationary) {
				// Vehicle moved — restart this axis' window.
				sum = 0; count = 0;
				return;
			}
			sum += sample;
			count++;
			if (count >= GYRO_BIAS_SAMPLES) {
				bias_out = sum / count;
				locked = true;
			}
		};
		// Calibrate on FILTERED samples so the gate sees the smoothed signal.
		accumulate_axis(_gyro_filt_x, _gyro_bias_sum_x,
		                _gyro_bias_count_x, _gyro_bias_x, _gyro_bias_locked_x);
		accumulate_axis(_gyro_filt_y, _gyro_bias_sum_y,
		                _gyro_bias_count_y, _gyro_bias_y, _gyro_bias_locked_y);
		accumulate_axis(_gyro_filt_z, _gyro_bias_sum_z,
		                _gyro_bias_count_z, _gyro_bias_z, _gyro_bias_locked_z);

		if (!_gyro_bias_reported && _gyro_bias_locked_x
		    && _gyro_bias_locked_y && _gyro_bias_locked_z) {
			PX4_INFO("Gyro bias locked (%d samples/axis, %u windows rejected): "
				 "X=%.4f Y=%.4f Z=%.4f rad/s",
				 GYRO_BIAS_SAMPLES, (unsigned)_gyro_bias_rejects,
				 (double)_gyro_bias_x, (double)_gyro_bias_y, (double)_gyro_bias_z);
			_gyro_bias_reported = true;
		}

		// Accelerometer magnitude calibration (same approach as the px4xplane
		// plugin's AccelCalibration). Only runs while vehicle is stationary
		// (ground-speed below threshold) and accel + velocity data are both
		// valid. If vehicle moves before calibration completes, restart.
		if (!_accel_calibrated && vel_ok) {
			if (stationary) {
				_accel_cal_stationary_count++;
				if (_accel_cal_stationary_count >= ACCEL_CAL_WAIT_SAMPLES) {
					// Use filtered accel for calibration so the magnitude
					// estimate isn't dominated by X-Plane jitter.
					const float mag = sqrtf(_accel_filt_x*_accel_filt_x
					                      + _accel_filt_y*_accel_filt_y
					                      + _accel_filt_z*_accel_filt_z);
					_accel_cal_sum_mag += mag;
					_accel_cal_count++;
					if (_accel_cal_count >= ACCEL_CAL_SAMPLES) {
						const float measured = _accel_cal_sum_mag / _accel_cal_count;
						if (measured > 0.1f) {
							_accel_scale_factor = GRAVITY_MSS / measured;
							_accel_calibrated = true;
							PX4_INFO("Accel calibrated: measured |g|=%.4f m/s², "
								 "scale=%.4f (%.2f%% correction)",
								 (double)measured, (double)_accel_scale_factor,
								 (double)((_accel_scale_factor - 1.0f) * 100.0f));
						}
					}
				}
			} else {
				// Aircraft moved before calibration completed — restart.
				_accel_cal_stationary_count = 0;
				_accel_cal_count = 0;
				_accel_cal_sum_mag = 0.f;
			}
		}

		perf_count(_perf_interval);
		publish_imu(t);
	}

	if (_rref_pos_mask == 0x7) {
		_has_pos = true;
		if (t - _t_last_baro >= 40_ms) {
			publish_baro(t);
			_t_last_baro = t;
		}
	}

	if (_rref_vel_mask == 0x7) { _has_vel = true; }

	if (_has_pos && _has_vel && (t - _t_last_gps >= 200_ms)) {
		publish_gps(t);
		_t_last_gps = t;
	}
}

void SimulatorXPlane::handle_data_at(const uint8_t *pkt, size_t len)
{
	_data_pkt_count++;
	size_t off = 5;
	while (off + 36 <= len) {
		uint32_t gid;
		float    d[8];
		memcpy(&gid, pkt + off,     4);
		memcpy(d,    pkt + off + 4, 32);
		off += 36;
		apply_data_group(gid, d);
	}
}

void SimulatorXPlane::apply_data_group(uint32_t gid, const float *d)
{
	const hrt_abstime t = hrt_absolute_time();

	switch (gid) {
	case XP_GRP_GLOAD:
		// Fallback if RREF not yet available. d[5]=g_axil, d[6]=g_side, d[7]=g_nrml
		if (_rref_accel_mask != 0x7) {
			_accel_x = -d[5] * GRAVITY_MSS;
			_accel_y =  d[6] * GRAVITY_MSS;
			_accel_z = -d[7] * GRAVITY_MSS;
		}
		break;

	case XP_GRP_ANG_VEL:
		// Fallback if RREF Prad/Qrad/Rrad not yet fully received.
		// XP11: d[1]=P, d[0]=Q, d[2]=R already in rad/s (different slot order)
		// XP12: d[0]=P, d[1]=Q, d[2]=R in deg/s (needs radians conversion)
		if (_rref_gyro_mask != 0x7) {
			if (is_xplane12()) {
				_gyro_x = math::radians(d[0]);
				_gyro_y = math::radians(d[1]);
				_gyro_z = math::radians(d[2]);
			} else {
				// XP11 — rad/s, but P/Q slots swapped
				_gyro_x = d[1];
				_gyro_y = d[0];
				_gyro_z = d[2];
			}
			publish_imu(t);
		}
		break;

	case XP_GRP_ATTITUDE:
		// Attitude now comes from RREF theta/phi/psi (more reliable than
		// DATA@ slot-order which varies between X-Plane versions and
		// per-user network settings). Fallback only if RREF hasn't filled
		// the mask yet — and even then, only consume d[0]=pitch, d[1]=roll,
		// d[2]=heading_true, the X-Plane 11/12 documented layout.
		if (_rref_att_mask != 0x7) {
			_pitch_rad = math::radians(d[0]);
			_roll_rad  = math::radians(d[1]);
			_yaw_rad   = math::radians(d[2]);
			_has_att   = true;
			publish_mag(t);
		}
		break;

	default:
		break;
	}

	if (_has_pos && (t - _t_last_baro) >= 40_ms) {
		publish_baro(t);
		_t_last_baro = t;
	}

	if (_has_pos && _has_vel && (t - _t_last_gps) >= 200_ms) {
		publish_gps(t);
		_t_last_gps = t;
	}
}

// ── Sensor publishing ─────────────────────────────────────────────────────────

void SimulatorXPlane::publish_imu(hrt_abstime t)
{
	// Noise σ values matched to px4xplane plugin's MEMS sensor model
	// (BMI088/ICM-20689 gyro + SIH-reference accel vibration). Below these
	// levels EKF2 over-trusts the sensor and reads small X-Plane physics
	// offsets as catastrophic bias — which blocks arming and causes jumpy
	// initialization. The plugin learned this the hard way; we inherit the
	// lesson. Mag noise lowered to match plugin's value too (was 10× too high).
	const float gyro_n  = 0.01f;     // rad/s RMS (~0.57°/s, BMI088 datasheet)
	const float accel_n = 0.1f;      // m/s² RMS (SIH reference)

	// Pre-arm IMU gate — same trick as satria-firmware's SIM_XPlane.
	// Before arming, EKF2 / hover_thrust_estimator / mc_pos_control all need
	// to settle their bias estimates. X-Plane's raw IMU stream has 0.2-1.0
	// m/s² and °/s jitter (much higher than real MEMS), which makes the
	// bias estimators chase ghosts during boot — sometimes for tens of
	// seconds. Freezing the IMU at the stationary baseline (gyro=0,
	// accel=(0,0,-g)) while disarmed lets every estimator converge
	// instantly, mimicking a vehicle on a perfectly stable test rig. The
	// moment we arm, the bridge's filtered/calibrated stream flows live
	// for use by the EKF + rate controller.
	_vehicle_status_sub.update(&_vehicle_status);
	const bool armed = (_vehicle_status.arming_state ==
	                    vehicle_status_s::ARMING_STATE_ARMED);

	float accel_x = _accel_filt_x * _accel_scale_factor;
	float accel_y = _accel_filt_y * _accel_scale_factor;
	float accel_z = _accel_filt_z * _accel_scale_factor;
	float gyro_x  = _gyro_filt_x - _gyro_bias_x;
	float gyro_y  = _gyro_filt_y - _gyro_bias_y;
	float gyro_z  = _gyro_filt_z - _gyro_bias_z;

	if (!armed) {
		accel_x = 0.0f;
		accel_y = 0.0f;
		accel_z = -CONSTANTS_ONE_G;   // stationary specific force (FRD body)
		gyro_x  = 0.0f;
		gyro_y  = 0.0f;
		gyro_z  = 0.0f;
	}

	// Publish FILTERED accel with magnitude scaling. The IIR low-pass kills
	// X-Plane physics jitter before noise injection; scaling normalises |a|
	// to ≈1 g (no-op until calibration completes).
	_px4_accel.set_temperature(20.0f);
	_px4_accel.update(t,
		accel_x + xplane_wgn() * accel_n,
		accel_y + xplane_wgn() * accel_n,
		accel_z + xplane_wgn() * accel_n);

	// Publish FILTERED gyro with bias subtracted (no-op pre-calibration).
	_px4_gyro.set_temperature(20.0f);
	_px4_gyro.update(t,
		gyro_x + xplane_wgn() * gyro_n,
		gyro_y + xplane_wgn() * gyro_n,
		gyro_z + xplane_wgn() * gyro_n);
}

void SimulatorXPlane::update_mag_earth()
{
	// Only refresh when we have a real position and either it's the first time
	// or the vehicle has moved a non-trivial distance (~1 km).
	if (_rref_pos_mask != 0x7) { return; }
	const float dlat = _lat_deg - _mag_earth_lat;
	const float dlon = _lon_deg - _mag_earth_lon;
	if (_mag_earth_valid && (dlat*dlat + dlon*dlon < 1e-4f)) { return; }   // ≈11 km/deg

	const float dec_rad = math::radians(get_mag_declination_degrees(_lat_deg, _lon_deg));
	const float inc_rad = math::radians(get_mag_inclination_degrees(_lat_deg, _lon_deg));
	const float field   = get_mag_strength_gauss(_lat_deg, _lon_deg);

	// NED earth-frame vector: horizontal along declination, vertical via inclination.
	const float h = field * cosf(inc_rad);
	_mag_ned_n = h * cosf(dec_rad);
	_mag_ned_e = h * sinf(dec_rad);
	_mag_ned_d = field * sinf(inc_rad);

	_mag_earth_lat   = _lat_deg;
	_mag_earth_lon   = _lon_deg;
	_mag_earth_valid = true;
}

void SimulatorXPlane::mag_body(float &bx, float &by, float &bz) const
{
	const float p = _pitch_rad, r = _roll_rad, y = _yaw_rad;
	const float cp = cosf(p), sp = sinf(p);
	const float cr = cosf(r), sr = sinf(r);
	const float cy = cosf(y), sy = sinf(y);
	const float mn = _mag_ned_n, me = _mag_ned_e, md = _mag_ned_d;
	bx = (cp*cy)*mn           + (cp*sy)*me           + (-sp)*md;
	by = (sr*sp*cy - cr*sy)*mn + (sr*sp*sy + cr*cy)*me + (sr*cp)*md;
	bz = (cr*sp*cy + sr*sy)*mn + (cr*sp*sy - sr*cy)*me + (cr*cp)*md;
}

void SimulatorXPlane::publish_mag(hrt_abstime t)
{
	update_mag_earth();
	if (!_mag_earth_valid) { return; }   // wait for first GPS fix before publishing

	float bx, by, bz;
	mag_body(bx, by, bz);
	_px4_mag.set_temperature(20.0f);
	// Mag noise σ = 0.0002 Gauss (200 nT) — matches px4xplane plugin's
	// MEMS magnetometer model. Previous 0.002 was 10× too high and made
	// EKF2 under-trust mag heading.
	const float mag_n = 0.0002f;
	_px4_mag.update(t,
		bx + xplane_wgn() * mag_n,
		by + xplane_wgn() * mag_n,
		bz + xplane_wgn() * mag_n);
}

void SimulatorXPlane::publish_baro(hrt_abstime t)
{
	sensor_baro_s baro{};
	baro.timestamp_sample = t;
	// 1 Pa RMS noise — matches sensor_baro_sim and prevents DataValidator from
	// flagging the stream STALE when altitude is constant on the ground.
	baro.pressure         = 101325.0f * expf(-_alt_m / 8434.5f) + xplane_wgn() * 1.0f;
	baro.temperature      = 20.0f + xplane_wgn() * 0.01f;
	baro.device_id        = 6620172;
	baro.timestamp        = hrt_absolute_time();
	_baro_pub.publish(baro);
}

void SimulatorXPlane::publish_gps(hrt_abstime t)
{
	if (!_gps_pub) {
		_gps_pub = new uORB::PublicationMulti<sensor_gps_s>{ORB_ID(sensor_gps)};
	}

	sensor_gps_s gps{};
	gps.time_utc_usec         = t;
	gps.latitude_deg          = _lat_deg;
	gps.longitude_deg         = _lon_deg;
	gps.altitude_msl_m        = _alt_m;
	gps.altitude_ellipsoid_m  = _alt_m;
	gps.fix_type              = 3;
	gps.eph                   = 0.5f;
	gps.epv                   = 0.8f;
	gps.hdop                  = 1.0f;
	gps.vdop                  = 1.0f;
	gps.vel_m_s               = sqrtf(_vel_n * _vel_n + _vel_e * _vel_e);
	gps.vel_n_m_s             = _vel_n;
	gps.vel_e_m_s             = _vel_e;
	gps.vel_d_m_s             = _vel_d;
	gps.cog_rad               = atan2f(_vel_e, _vel_n);
	gps.vel_ned_valid          = true;
	gps.satellites_used        = 10;
	gps.s_variance_m_s        = 0.25f;
	gps.c_variance_rad        = 0.1f;
	gps.heading               = NAN;
	gps.heading_offset        = NAN;

	device::Device::DeviceId devid{};
	devid.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	devid.devid_s.bus      = 0;
	devid.devid_s.address  = 0;
	devid.devid_s.devtype  = DRV_GPS_DEVTYPE_SIM;
	gps.device_id          = devid.devid;
	gps.timestamp          = hrt_absolute_time();
	_gps_pub->publish(gps);
}

// ── Sender thread ─────────────────────────────────────────────────────────────

void *SimulatorXPlane::sending_trampoline(void * /*unused*/)
{
	g_instance->send();
	return nullptr;
}

void SimulatorXPlane::send()
{
#ifndef __PX4_DARWIN
	pthread_setname_np(pthread_self(), "sim_xplane_send");
#endif

	_actuator_sub = orb_subscribe_multi(ORB_ID(actuator_outputs_sim), 0);

	px4_pollfd_struct_t fds[1]{};
	fds[0].fd     = _actuator_sub;
	fds[0].events = POLLIN;

	while (true) {
		int pret = px4_poll(fds, 1, 100);
		if (pret <= 0) { continue; }

		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(actuator_outputs_sim), _actuator_sub, &_actuator_outputs);
			send_actuator_drefs();
		}
	}

	orb_unsubscribe(_actuator_sub);
}

// ── Main receive loop ─────────────────────────────────────────────────────────

void SimulatorXPlane::run()
{
#ifndef __PX4_DARWIN
	pthread_setname_np(pthread_self(), "sim_xplane_recv");
#endif

	// ── Open UDP socket ──────────────────────────────────────────────────────
	_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
	if (_fd < 0) { PX4_ERR("socket: %s", strerror(errno)); return; }

	int opt = 1;
	setsockopt(_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

	struct sockaddr_in bind_addr{};
	bind_addr.sin_family      = AF_INET;
	bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	bind_addr.sin_port        = htons(_bind_port);

	if (::bind(_fd, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
		PX4_ERR("bind port %u: %s", _bind_port, strerror(errno));
		::close(_fd); _fd = -1; return;
	}

	memset(&_xplane_addr, 0, sizeof(_xplane_addr));
	_xplane_addr.sin_family = AF_INET;
	_xplane_addr.sin_port   = htons(_xplane_port);
	inet_pton(AF_INET, _xplane_ip, &_xplane_addr.sin_addr);

	PX4_INFO("UDP port %u → X-Plane %s:%u  |  %d DREFs loaded",
		 _bind_port, _xplane_ip, _xplane_port, _n_drefs);

	// ── Bootstrap X-Plane ────────────────────────────────────────────────────
	send_dsel();
	send_rref_subscribe();
	send_fixed_drefs();
	_t_last_dsel  = hrt_absolute_time();
	_t_last_rref  = hrt_absolute_time();
	_t_last_fixed = hrt_absolute_time();

	// Wait for first recognised packet from X-Plane.
	// Re-sends subscription requests every 5 s and prints a status line every
	// 2 s so the user can see what is (or isn't) arriving.
	PX4_INFO("Waiting for X-Plane data on UDP port %u …", _bind_port);
	PX4_INFO("  Make sure X-Plane Network settings send data to %s:%u", _xplane_ip, _bind_port);
	{
		uint8_t buf[4096];
		hrt_abstime t_wait_status = hrt_absolute_time();

		while (true) {
			fd_set rfds;
			FD_ZERO(&rfds);
			FD_SET(_fd, &rfds);
			struct timeval tv{1, 0};   // 1 s timeout
			int ret = ::select(_fd + 1, &rfds, nullptr, nullptr, &tv);

			hrt_abstime now = hrt_absolute_time();

			if (ret > 0 && FD_ISSET(_fd, &rfds)) {
				struct sockaddr_in src{};
				socklen_t src_len = sizeof(src);
				ssize_t n = ::recvfrom(_fd, buf, sizeof(buf), 0,
						       (struct sockaddr *)&src, &src_len);

				if (n < 5) { continue; }

				char src_ip[INET_ADDRSTRLEN] = "?";
				inet_ntop(AF_INET, &src.sin_addr, src_ip, sizeof(src_ip));
				uint16_t src_port = ntohs(src.sin_port);

				if (memcmp(buf, "DATA", 4) == 0) {
					PX4_INFO("X-Plane connected: DATA@ from %s:%u  (%zd bytes)",
						 src_ip, src_port, n);
					break;

				} else if (memcmp(buf, "RREF", 4) == 0) {
					PX4_INFO("X-Plane connected: RREF  from %s:%u  (%zd bytes)",
						 src_ip, src_port, n);
					break;

				} else {
					char hdr[6] = {};
					memcpy(hdr, buf, (n < 5) ? (size_t)n : 5);
					PX4_WARN("Unknown packet from %s:%u  hdr='%.5s'  len=%zd",
						 src_ip, src_port, hdr, n);
				}

			} else {
				// Timeout — print status every 2 s
				if (now - t_wait_status >= 2_s) {
					PX4_INFO("Still waiting for X-Plane … (no data on port %u)", _bind_port);
					t_wait_status = now;
				}
			}

			// Re-subscribe periodically in case X-Plane restarted
			if (now - _t_last_dsel >= 5_s) { send_dsel();           _t_last_dsel = now; }
			if (now - _t_last_rref >= 5_s) { send_rref_subscribe(); _t_last_rref = now; }
		}
	}

	_t_last_rate  = hrt_absolute_time();
	_t_last_fixed = hrt_absolute_time();
	send_fixed_drefs();

	// ── Start sender thread ──────────────────────────────────────────────────
	pthread_t        sender;
	pthread_attr_t   attr;
	pthread_attr_init(&attr);
	pthread_attr_setstacksize(&attr, PX4_STACK_ADJUSTED(8000));
	pthread_create(&sender, &attr, SimulatorXPlane::sending_trampoline, nullptr);
	pthread_attr_destroy(&attr);

	// ── Main receive loop ────────────────────────────────────────────────────
	uint8_t buf[4096];

	while (true) {
		fd_set rfds;
		FD_ZERO(&rfds);
		FD_SET(_fd, &rfds);
		struct timeval tv{0, 10000};
		int ret = ::select(_fd + 1, &rfds, nullptr, nullptr, &tv);

		if (ret > 0 && FD_ISSET(_fd, &rfds)) {
			ssize_t n = ::recvfrom(_fd, buf, sizeof(buf), 0, nullptr, nullptr);
			if (n >= 5) {
				// Advance PX4 lockstep clock so hrt_absolute_time() is non-zero
				struct timespec ts;
				clock_gettime(CLOCK_MONOTONIC, &ts);
				px4_clock_settime(CLOCK_MONOTONIC, &ts);

				if      (memcmp(buf, "RREF", 4) == 0) { handle_rref(buf, (size_t)n); }
				else if (memcmp(buf, "DATA", 4) == 0) { handle_data_at(buf, (size_t)n); }
			}
		}

		hrt_abstime now = hrt_absolute_time();
		if (now - _t_last_dsel   >= 5_s) { send_dsel();             _t_last_dsel   = now; }
		if (now - _t_last_rref   >= 5_s) { send_rref_subscribe();   _t_last_rref   = now; }
		if (now - _t_last_fixed  >= 1_s) { send_fixed_drefs();      _t_last_fixed  = now; }

		if (now - _t_last_rate >= 5_s) {
			const float dt = (float)(now - _t_last_rate) * 1e-6f;
			// Status line modeled after ArduPilot's SIM_XPlane debug print:
			// rates + attitude + position so a single line tells you whether
			// the bridge is healthy and what state X-Plane is reporting.
			printf("XP%u.%u DATA@:%5.1f Hz  RREF:%5.1f Hz  "
			       "R/P/Y:%+6.1f/%+6.1f/%+6.1f deg  "
			       "vel N/E/D:%+6.2f/%+6.2f/%+6.2f m/s  "
			       "pos:%.6f/%.6f  alt:%.1f m\n",
			       _xplane_version / 10000, (_xplane_version % 10000) / 100,
			       (double)(_data_pkt_count / dt),
			       (double)(_rref_pkt_count / dt),
			       (double)math::degrees(_roll_rad),
			       (double)math::degrees(_pitch_rad),
			       (double)math::degrees(_yaw_rad),
			       (double)_vel_n, (double)_vel_e, (double)_vel_d,
			       (double)_lat_deg, (double)_lon_deg, (double)_alt_m);
			_rref_pkt_count = 0;
			_data_pkt_count = 0;
			_t_last_rate    = now;
		}
	}

	pthread_join(sender, nullptr);
}

// ── Module entry points ───────────────────────────────────────────────────────

int SimulatorXPlane::start(int argc, char *argv[])
{
	const char *xplane_ip  = "127.0.0.1";
	uint16_t    xplane_port = 49000;
	uint16_t    bind_port   = 49005;
	const char *model_name  = "xplane_quad";

	for (int i = 3; i < argc - 1; i++) {
		if      (strcmp(argv[i], "-i") == 0) { xplane_ip   = argv[++i]; }
		else if (strcmp(argv[i], "-p") == 0) { xplane_port = (uint16_t)atoi(argv[++i]); }
		else if (strcmp(argv[i], "-b") == 0) { bind_port   = (uint16_t)atoi(argv[++i]); }
		else if (strcmp(argv[i], "-m") == 0) { model_name  = argv[++i]; }
	}

	g_instance = new SimulatorXPlane(xplane_ip, xplane_port, bind_port, model_name);
	if (!g_instance) { PX4_ERR("alloc failed"); return 1; }
	g_instance->run();
	return 0;
}

__BEGIN_DECLS
extern int simulator_xplane_main(int argc, char *argv[]);
__END_DECLS

int simulator_xplane_main(int argc, char *argv[])
{
	if (argc < 2 || strcmp(argv[1], "start") != 0) {
		PX4_INFO("Usage: simulator_xplane start [-i <ip>] [-p <xplane_port>] [-b <bind_port>] [-m <model>]");
		PX4_INFO("  -i  X-Plane IP     (default: 127.0.0.1)");
		PX4_INFO("  -p  X-Plane port   (default: 49000)");
		PX4_INFO("  -b  Local UDP port (default: 49005)");
		PX4_INFO("  -m  Model name     (default: xplane_quad)");
		PX4_INFO("      Map file: etc/init.d-posix/models/<model>.json");
		return 1;
	}

	if (g_task >= 0) { PX4_WARN("already running"); return 0; }

	g_task = px4_task_spawn_cmd("simulator_xplane",
				    SCHED_DEFAULT, SCHED_PRIORITY_MAX,
				    PX4_STACK_ADJUSTED(8000),
				    SimulatorXPlane::start, argv);
	return (g_task >= 0) ? 0 : 1;
}
