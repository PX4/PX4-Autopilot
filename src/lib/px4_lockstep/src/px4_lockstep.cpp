#include <px4_lockstep/px4_lockstep.h>

#include <cmath>
#include <limits>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <type_traits>
#include <utility>

#include <new>
#include <atomic>
#include <pthread.h>

#include <px4_platform_common/init.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/uORBTopics.hpp>

#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/geofence_status.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>

#include <dataman/dataman.h>
#include <lib/parameters/param.h>

// Module headers (make sure px4_lockstep/CMakeLists adds these include dirs)
#include "Commander.hpp"
#include "navigator.h"
#include "navigation.h"
#include "MulticopterPositionControl.hpp"
#include "mc_att_control.hpp"
#include "MulticopterRateControl.hpp"
#include "FlightModeManager.hpp"
#include "ControlAllocator.hpp"

// For VEHICLE_CMD_DO_SET_MODE payloads
#include "px4_custom_mode.h"

// -----------------------------------------------------------------------------
// ABI compatibility helpers
// -----------------------------------------------------------------------------

extern "C" PX4_LOCKSTEP_EXPORT uint32_t px4_lockstep_abi_version(void)
{
	return PX4_LOCKSTEP_ABI_VERSION;
}

extern "C" PX4_LOCKSTEP_EXPORT void px4_lockstep_sizes(uint32_t *in_sz,
				    uint32_t *out_sz,
				    uint32_t *cfg_sz)
{
	if (in_sz) {
		*in_sz = 0u;
	}
	if (out_sz) {
		*out_sz = 0u;
	}
	if (cfg_sz) {
		*cfg_sz = static_cast<uint32_t>(sizeof(px4_lockstep_config_t));
	}
}

namespace {

constexpr float kPi = 3.14159265358979323846f;
static std::atomic<int> g_lockstep_active{0};

static inline float deg2rad(float deg) { return deg * (kPi / 180.0f); }

static inline bool env_flag_enabled(const char *name)
{
	const char *val = getenv(name);
	if (!val || val[0] == '\0') {
		return false;
	}
	return !(val[0] == '0' && val[1] == '\0');
}

static inline void ensure_lockstep_thread_name()
{
#ifndef __PX4_QURT
	static thread_local bool name_checked = false;
	if (name_checked) {
		return;
	}
	name_checked = true;

	constexpr size_t kNameLen = 32;
	char name_buf[kNameLen] {};
	int ret = pthread_getname_np(pthread_self(), name_buf, kNameLen);
	if (ret == 0 && name_buf[0] != '\0') {
		return;
	}

#ifdef __PX4_DARWIN
	(void)pthread_setname_np("px4_lockstep");
#else
	(void)pthread_setname_np(pthread_self(), "px4_lockstep");
#endif
#endif
}

struct StepRateLimiter {
	uint64_t last_run_us{0};
	uint64_t period_us{0}; // 0 => always

	void set_hz(int32_t hz)
	{
		if (hz > 0) {
			period_us = static_cast<uint64_t>(1000000ULL / static_cast<uint64_t>(hz));
		} else {
			period_us = 0;
		}
	}

	bool should_run(uint64_t now_us)
	{
		if (period_us == 0) {
			return true;
		}
		if (last_run_us == 0) {
			last_run_us = now_us;
			return true;
		}
		if ((now_us - last_run_us) >= period_us) {
			last_run_us = now_us;
			return true;
		}
		return false;
	}
};

struct LockstepRuntime {
	px4_lockstep_config_t cfg{};

	// Publications owned by the lockstep harness.
	uORB::Publication<home_position_s>   pub_home{ORB_ID(home_position)};
	uORB::Publication<mission_s>         pub_mission{ORB_ID(mission)};
	uORB::Publication<geofence_status_s> pub_geofence_status{ORB_ID(geofence_status)};
	uORB::Publication<vehicle_status_s> pub_vehicle_status{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_control_mode_s> pub_vehicle_control_mode{ORB_ID(vehicle_control_mode)};
	uORB::Publication<actuator_armed_s> pub_actuator_armed{ORB_ID(actuator_armed)};
	uORB::Publication<mission_result_s> pub_mission_result{ORB_ID(mission_result)};

	// Debug/inspection subscriptions
	uORB::Subscription sub_vehicle_status_dbg{ORB_ID(vehicle_status)};
	uORB::Subscription sub_vehicle_control_mode_dbg{ORB_ID(vehicle_control_mode)};
	uORB::Subscription sub_pos_sp_triplet{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription sub_traj_sp{ORB_ID(trajectory_setpoint)};
	uORB::Subscription sub_vehicle_constraints_dbg{ORB_ID(vehicle_constraints)};
	uORB::Subscription sub_home_position{ORB_ID(home_position)};
	uORB::Subscription sub_global_position{ORB_ID(vehicle_global_position)};
	uORB::Subscription sub_geofence_status_dbg{ORB_ID(geofence_status)};
	uORB::Subscription sub_mission_result_dbg{ORB_ID(mission_result)};
	uORB::Subscription sub_att_sp{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription sub_rates_sp{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription sub_mission_result{ORB_ID(mission_result)};
	uORB::Subscription sub_land_detected{ORB_ID(vehicle_land_detected)};
	uORB::Subscription sub_vehicle_local_position{ORB_ID(vehicle_local_position)};

	// PX4 modules we step
	Commander *commander{nullptr};
	Navigator *navigator{nullptr};
	MulticopterPositionControl *mc_pos{nullptr};
	MulticopterAttitudeControl *mc_att{nullptr};
	MulticopterRateControl *mc_rate{nullptr};
	FlightModeManager *flight_mode_mgr{nullptr};
	ControlAllocator *control_alloc{nullptr};

	// Rate limiters
	StepRateLimiter cmd_rate;
	StepRateLimiter nav_rate;
	StepRateLimiter pos_rate;
	StepRateLimiter att_rate;
	StepRateLimiter rate_rate;
	StepRateLimiter alloc_rate;

	// Home position handling
	bool home_set{false};
	uint32_t home_update_count{0};
	double home_lat{std::numeric_limits<double>::quiet_NaN()};
	double home_lon{std::numeric_limits<double>::quiet_NaN()};
	float home_alt{NAN};
	uint64_t last_time_us{0};
	uint64_t last_debug_us{0};
	bool debug_enabled{false};

	// Commander-lite command inputs and state (used when commander is disabled).
	px4_lockstep_cmd_t cmd{};
	bool cmd_mission_started{false};
	bool cmd_mission_completed{false};
	// ---------------------------------------------------------------------
	// External (Julia) uORB pub/sub handles
	// ---------------------------------------------------------------------
	struct ExtPublisher {
		const struct orb_metadata *meta{nullptr};
		orb_advert_t handle{nullptr};
		// If >= 0, the publisher must advertise on this uORB instance.
		// If -1, uORB assigns an instance automatically on advertise.
		int requested_instance{-1};
		int instance{-1};
		int priority{0};
		unsigned queue_size{1};
		std::vector<uint8_t> buffer{};
		bool pending{false};
	};

	struct ExtSubscriber {
		const struct orb_metadata *meta{nullptr};
		int handle{-1};
	};

	std::vector<ExtPublisher> ext_pubs{};
	std::vector<ExtSubscriber> ext_subs{};

	LockstepRuntime()
	{
		cfg = {};
		cfg.dataman_use_ram = 1;
		cfg.enable_commander = 0;
		cfg.commander_rate_hz = 100;
		cfg.navigator_rate_hz = 20;
		cfg.mc_pos_control_rate_hz = 100;
		cfg.mc_att_control_rate_hz = 250;
		cfg.mc_rate_control_rate_hz = 250;
		cfg.enable_control_allocator = 1;
		cfg.control_allocator_rate_hz = 250;

	}

	~LockstepRuntime()
	{
		delete commander;
		delete navigator;
		delete mc_pos;
		delete mc_att;
		delete mc_rate;
		delete flight_mode_mgr;
		delete control_alloc;

		for (auto &s : ext_subs) {
			if (s.handle >= 0) {
				(void)orb_unsubscribe(s.handle);
				s.handle = -1;
			}
		}
		for (auto &p : ext_pubs) {
			if (p.handle != nullptr) {
				(void)orb_unadvertise(p.handle);
				p.handle = nullptr;
			}
		}
	}
};

// --- Mission file loader (QGC WPL 110) ---

struct WplLine {
	int seq{0};
	int current{0};
	int frame{0};
	int command{0};
	float param1{0.f};
	float param2{0.f};
	float param3{0.f};
	float param4{0.f};
	double x{0.0}; // lat
	double y{0.0}; // lon
	float z{0.f};  // alt
	int autocontinue{1};
};

static bool parse_wpl_line(const std::string &line, WplLine &out)
{
	std::stringstream ss(line);
	ss >> out.seq;
	ss >> out.current;
	ss >> out.frame;
	ss >> out.command;
	ss >> out.param1;
	ss >> out.param2;
	ss >> out.param3;
	ss >> out.param4;
	ss >> out.x;
	ss >> out.y;
	ss >> out.z;
	ss >> out.autocontinue;
	return !ss.fail();
}

static void wpl_to_mission_item(const WplLine &wpl, mission_item_s &mi)
{
	mi = {};
	mi.nav_cmd = static_cast<uint16_t>(wpl.command);
	mi.lat = wpl.x;
	mi.lon = wpl.y;
	mi.altitude = wpl.z;
	mi.autocontinue = (wpl.autocontinue != 0);

	// Frame handling (best-effort): MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
	mi.altitude_is_relative = (wpl.frame == 3);

	// Common MAVLink semantics:
	// param4 is yaw in degrees for many NAV_* commands
	mi.yaw = deg2rad(wpl.param4);

	// Waypoint params
	// MAV_CMD_NAV_WAYPOINT (16): param1 = hold, param2 = acceptance radius
	// MAV_CMD_NAV_TAKEOFF (22): param1 = pitch
	if (wpl.command == 16) {
		mi.time_inside = wpl.param1;
		mi.acceptance_radius = wpl.param2;

	} else if (wpl.command == 22) {
		// acceptance_radius sometimes used by PX4 for takeoff
		mi.acceptance_radius = wpl.param2;
	}

	// Reasonable defaults
	if (!PX4_ISFINITE(mi.acceptance_radius) || mi.acceptance_radius <= 0.f) {
		mi.acceptance_radius = 1.0f;
	}
}

} // namespace

namespace {

static const struct orb_metadata *orb_meta_from_name(const char *topic_name)
{
	if (topic_name == nullptr) {
		return nullptr;
	}
	const struct orb_metadata *const *topics = orb_get_topics();
	const size_t n = orb_topics_count();
	for (size_t i = 0; i < n; i++) {
		const struct orb_metadata *m = topics[i];
		if (m && m->o_name && (std::strcmp(m->o_name, topic_name) == 0)) {
			return m;
		}
	}
	return nullptr;
}

// Some PX4 versions include additional metadata like "size without padding".
// Detect it at compile time without hard-depending on a specific PX4 uORB.h.
template <typename T>
static inline auto get_o_size_no_padding_impl(const T *m, int)
	-> decltype(m->o_size_no_padding, uint32_t{})
{
	return static_cast<uint32_t>(m->o_size_no_padding);
}

template <typename T>
static inline uint32_t get_o_size_no_padding_impl(const T *, ...)
{
	return 0u;
}

static inline uint32_t get_o_size_no_padding(const struct orb_metadata *m)
{
	return get_o_size_no_padding_impl(m, 0);
}

template <typename T>
static inline auto get_o_fields_impl(const T *m, int) -> decltype(m->o_fields)
{
	return m->o_fields;
}

template <typename T>
static inline const char *get_o_fields_impl(const T *, ...)
{
	return nullptr;
}

static inline const char *get_o_fields(const struct orb_metadata *m)
{
	return get_o_fields_impl(m, 0);
}

template <typename T>
static inline auto get_message_hash_impl(const T *m, int)
	-> decltype(m->message_hash, uint32_t{})
{
	return static_cast<uint32_t>(m->message_hash);
}

template <typename T>
static inline uint32_t get_message_hash_impl(const T *, ...)
{
	return 0u;
}

static inline uint32_t get_message_hash(const struct orb_metadata *m)
{
	return get_message_hash_impl(m, 0);
}

template <typename T>
static inline auto get_queue_size_impl(const T *m, int) -> decltype(m->o_queue, uint8_t{})
{
	return static_cast<uint8_t>(m->o_queue);
}

template <typename T>
static inline uint8_t get_queue_size_impl(const T *, ...)
{
	return 0u;
}

static inline uint8_t get_queue_size(const struct orb_metadata *m)
{
	return get_queue_size_impl(m, 0);
}

} // namespace

namespace {

struct PendingParam {
	std::string name;
	bool is_int;
	int32_t i32;
	float f32;
};

std::vector<PendingParam> g_preinit_params;

int apply_param_i32(const char *name, int32_t value)
{
	if (!name) {
		return -1;
	}
	param_t h = param_find(name);
	if (h == PARAM_INVALID) {
		PX4_WARN("param not found: %s", name);
		return -2;
	}
	if (param_type(h) != PARAM_TYPE_INT32) {
		return -3;
	}
	return (param_set(h, &value) == 0) ? 0 : -4;
}

int apply_param_f32(const char *name, float value)
{
	if (!name) {
		return -1;
	}
	param_t h = param_find(name);
	if (h == PARAM_INVALID) {
		PX4_WARN("param not found: %s", name);
		return -2;
	}
	if (param_type(h) != PARAM_TYPE_FLOAT) {
		return -3;
	}
	return (param_set(h, &value) == 0) ? 0 : -4;
}

bool apply_preinit_params()
{
	bool ok = true;
	for (const auto &p : g_preinit_params) {
		int ret = 0;
		if (p.is_int) {
			ret = apply_param_i32(p.name.c_str(), p.i32);
		} else {
			ret = apply_param_f32(p.name.c_str(), p.f32);
		}
		if (ret != 0) {
			ok = false;
		}
	}
	g_preinit_params.clear();
	return ok;
}

} // namespace

extern "C" {

// -----------------------------------------------------------------------------
// PX4 parameter set/get helpers
// -----------------------------------------------------------------------------

int px4_lockstep_param_set_i32(px4_lockstep_handle_t handle, const char *name, int32_t value)
{
	if (!handle || !name) {
		return -1;
	}
	return apply_param_i32(name, value);
}

int px4_lockstep_param_set_f32(px4_lockstep_handle_t handle, const char *name, float value)
{
	if (!handle || !name) {
		return -1;
	}
	return apply_param_f32(name, value);
}

int px4_lockstep_param_get_i32(px4_lockstep_handle_t handle, const char *name, int32_t *out_value)
{
	if (!handle || !name || !out_value) {
		return -1;
	}
	param_t h = param_find(name);
	if (h == PARAM_INVALID) {
		return -2;
	}
	if (param_type(h) != PARAM_TYPE_INT32) {
		return -3;
	}
	return (param_get(h, out_value) == 0) ? 0 : -4;
}

int px4_lockstep_param_get_f32(px4_lockstep_handle_t handle, const char *name, float *out_value)
{
	if (!handle || !name || !out_value) {
		return -1;
	}
	param_t h = param_find(name);
	if (h == PARAM_INVALID) {
		return -2;
	}
	if (param_type(h) != PARAM_TYPE_FLOAT) {
		return -3;
	}
	return (param_get(h, out_value) == 0) ? 0 : -4;
}

int px4_lockstep_param_notify(px4_lockstep_handle_t handle)
{
	if (!handle) {
		return -1;
	}
	param_notify_changes();
	return 0;
}

int px4_lockstep_param_preinit_set_i32(const char *name, int32_t value)
{
	if (!name) {
		return -1;
	}
	PendingParam p{};
	p.name = name;
	p.is_int = true;
	p.i32 = value;
	g_preinit_params.push_back(p);
	return 0;
}

int px4_lockstep_param_preinit_set_f32(const char *name, float value)
{
	if (!name) {
		return -1;
	}
	PendingParam p{};
	p.name = name;
	p.is_int = false;
	p.f32 = value;
	g_preinit_params.push_back(p);
	return 0;
}

int px4_lockstep_control_alloc_update_params(px4_lockstep_handle_t handle)
{
	if (!handle) {
		return -1;
	}
	LockstepRuntime *rt = reinterpret_cast<LockstepRuntime *>(handle);
	if (!rt->control_alloc) {
		return -2;
	}
	rt->control_alloc->lockstep_update_params();
	return 0;
}

px4_lockstep_handle_t px4_lockstep_create(const px4_lockstep_config_t *cfg)
{
	static bool platform_initialized = false;
	int expected = 0;
	if (!g_lockstep_active.compare_exchange_strong(expected, 1)) {
		PX4_ERR("lockstep: only one runtime handle is allowed per process");
		return nullptr;
	}
	if (!platform_initialized) {
		if (px4_platform_init() != PX4_OK) {
			PX4_ERR("px4_platform_init failed");
		}
		platform_initialized = true;
	}

	LockstepRuntime *rt = new (std::nothrow) LockstepRuntime();
	if (!rt) {
		g_lockstep_active.store(0);
		return nullptr;
	}

	if (cfg) {
		rt->cfg = *cfg;
	}

	if (rt->cfg.enable_commander != 0) {
		PX4_ERR("lockstep: commander-in-loop is not supported; set enable_commander=0");
		delete rt;
		g_lockstep_active.store(0);
		return nullptr;
	}

	rt->debug_enabled = env_flag_enabled("PX4_LOCKSTEP_DEBUG");

	rt->cmd_rate.set_hz(rt->cfg.commander_rate_hz);
	rt->nav_rate.set_hz(rt->cfg.navigator_rate_hz);
	rt->pos_rate.set_hz(rt->cfg.mc_pos_control_rate_hz);
	rt->att_rate.set_hz(rt->cfg.mc_att_control_rate_hz);
	rt->rate_rate.set_hz(rt->cfg.mc_rate_control_rate_hz);
	rt->alloc_rate.set_hz(rt->cfg.control_allocator_rate_hz);

	// Enable lockstep HRT time on POSIX (patched in platforms/posix).
	// Safe to call even if the lockstep functions are stubs on other platforms.
	hrt_lockstep_enable(true);

	// Dataman: initialize RAM backend and enable synchronous mode
	if (rt->cfg.dataman_use_ram) {
		// Patched API: initializes RAM backend without worker thread.
		if (dm_lockstep_init(true) != 0) {
			PX4_ERR("dm_lockstep_init failed");
		}
	}
	dm_lockstep_set_sync(true);

	// Parameters: we keep things minimal, but control allocation needs some CA_* parameters.
	// In normal PX4 these come from the airframe config file + Actuators setup.
	// In lockstep the simulator is expected to configure CA_* parameters deterministically
	// from the aircraft spec (via the exported px4_lockstep_param_set_* API).
	(void)param_init();

	auto set_param_i32 = [](const char *name, int32_t v) {
		(void)apply_param_i32(name, v);
	};

	// Commander-related defaults for lockstep simulation.
	//
	// - COM_LOW_BAT_ACT=3: RTL at critical battery, land at emergency.
	// - COM_RC_IN_MODE=1: allow running without RC input (common in pure SITL).
	//
	// These are only applied in the lockstep harness build and can be overridden
	// by normal PX4 param mechanisms if desired.
	if (rt->cfg.enable_commander) {
		set_param_i32("COM_LOW_BAT_ACT", 3);
		set_param_i32("COM_RC_IN_MODE", 1);
	}

	if (rt->cfg.enable_control_allocator) {
		// Enable dynamic control allocation.
		set_param_i32("SYS_CTRL_ALLOC", 1);
	}

	// Apply any pre-init parameters queued by the host (e.g. CA_* geometry).
	if (!apply_preinit_params()) {
		PX4_ERR("lockstep: preinit param application failed");
		delete rt;
		g_lockstep_active.store(0);
		return nullptr;
	}

	// Create PX4 modules (do NOT start their tasks).
	if (rt->cfg.enable_commander) {
		rt->commander = new (std::nothrow) Commander();
	}
	rt->navigator = new (std::nothrow) Navigator();
	rt->flight_mode_mgr = new (std::nothrow) FlightModeManager();
	rt->mc_pos = new (std::nothrow) MulticopterPositionControl();
	rt->mc_att = new (std::nothrow) MulticopterAttitudeControl();
	rt->mc_rate = new (std::nothrow) MulticopterRateControl();
	if (rt->cfg.enable_control_allocator) {
		rt->control_alloc = new (std::nothrow) ControlAllocator();
	}

	if ((rt->cfg.enable_commander && !rt->commander)
		|| !rt->navigator
		|| !rt->flight_mode_mgr
		|| !rt->mc_pos
		|| !rt->mc_att
		|| !rt->mc_rate
		|| (rt->cfg.enable_control_allocator && !rt->control_alloc)) {
		PX4_ERR("lockstep: failed to allocate modules");
		delete rt;
		g_lockstep_active.store(0);
		return nullptr;
	}

	// Put modules into lockstep mode (patched API in each module).
	if (rt->commander) {
		rt->commander->enable_lockstep(true);
	}
	rt->navigator->enable_lockstep(true);
	rt->flight_mode_mgr->enable_lockstep(true);
	rt->mc_pos->enable_lockstep(true);
	rt->mc_att->enable_lockstep(true);
	rt->mc_rate->enable_lockstep(true);
	if (rt->control_alloc) {
		rt->control_alloc->enable_lockstep(true);
	}

	// Run module-specific lockstep init (no callbacks, no scheduling).
	if (rt->commander) {
		(void)rt->commander->init_lockstep();
	}
	// Navigator initialization happens in run_once via its internal guard.
	(void)rt->flight_mode_mgr->init_lockstep();
	(void)rt->mc_pos->init_lockstep();
	(void)rt->mc_att->init_lockstep();
	(void)rt->mc_rate->init_lockstep();
	if (rt->control_alloc) {
		(void)rt->control_alloc->init_lockstep();
	}

	PX4_INFO("px4_lockstep created");
	return reinterpret_cast<px4_lockstep_handle_t>(rt);
}

void px4_lockstep_destroy(px4_lockstep_handle_t handle)
{
	LockstepRuntime *rt = reinterpret_cast<LockstepRuntime *>(handle);
	if (!rt) {
		return;
	}

	delete rt;
	g_lockstep_active.store(0);
}

int px4_lockstep_set_cmd(px4_lockstep_handle_t handle, const px4_lockstep_cmd_t *cmd)
{
	if (!handle || !cmd) {
		return -1;
	}
	LockstepRuntime *rt = reinterpret_cast<LockstepRuntime *>(handle);
	rt->cmd = *cmd;
	return 0;
}

int px4_lockstep_load_mission_qgc_wpl(px4_lockstep_handle_t handle, const char *mission_path)
{
	LockstepRuntime *rt = reinterpret_cast<LockstepRuntime *>(handle);
	if (!rt || !mission_path) {
		return -1;
	}

	std::ifstream f(mission_path);
	if (!f.is_open()) {
		PX4_ERR("failed to open mission file: %s", mission_path);
		return -2;
	}

	std::string line;
	if (!std::getline(f, line)) {
		PX4_ERR("empty mission file");
		return -3;
	}

	// Header: "QGC WPL 110"
	if (line.find("QGC") == std::string::npos) {
		PX4_WARN("mission header not QGC WPL, trying to parse anyway");
	}

	std::vector<mission_item_s> items;
	items.reserve(128);

	while (std::getline(f, line)) {
		if (line.empty()) {
			continue;
		}
		WplLine wpl;
		if (!parse_wpl_line(line, wpl)) {
			PX4_WARN("skip malformed WPL line: %s", line.c_str());
			continue;
		}
		mission_item_s mi{};
		wpl_to_mission_item(wpl, mi);
		items.push_back(mi);
	}

	if (items.empty()) {
		PX4_ERR("mission contains no items");
		return -4;
	}

	// Preload into Dataman using the same keys Mission uses.
	mission_s mission_state{};
	mission_state.timestamp = hrt_absolute_time();
	if (mission_state.timestamp == 0) {
		mission_state.timestamp = 1;
	}
	mission_state.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	mission_state.fence_dataman_id = DM_KEY_FENCE_POINTS_0;
	mission_state.safepoint_dataman_id = DM_KEY_SAFE_POINTS_0;
	mission_state.count = static_cast<uint16_t>(items.size());
	mission_state.current_seq = 0;
	mission_state.land_start_index = -1;
	mission_state.land_index = -1;
	mission_state.mission_id = mission_state.timestamp & 0xffffffffu;
	mission_state.geofence_id = 0u;
	mission_state.safe_points_id = 0u;

	// Store mission state
	if (!dm_lockstep_write(DM_KEY_MISSION_STATE, 0, &mission_state, sizeof(mission_state))) {
		PX4_ERR("dm_lockstep_write mission_state failed");
		return -5;
	}

	// Store mission items
	for (unsigned i = 0; i < items.size(); i++) {
		mission_item_s mi = items[i];
		if (!dm_lockstep_write(static_cast<dm_item_t>(mission_state.mission_dataman_id), i, &mi, sizeof(mi))) {
			PX4_ERR("dm_lockstep_write mission item %u failed", i);
			return -6;
		}
	}

	rt->pub_mission.publish(mission_state);

	PX4_INFO("mission loaded: %u items", (unsigned)items.size());
	return 0;
}

static void maybe_publish_home_position_fallback(LockstepRuntime &rt, uint64_t time_us)
{
	if (rt.cfg.enable_commander) {
		return;
	}

	// First, consume any externally injected home position updates.
	home_position_s home{};
	if (rt.sub_home_position.update(&home)) {
		if (home.valid_hpos && home.valid_alt) {
			rt.home_set = true;
			rt.home_lat = home.lat;
			rt.home_lon = home.lon;
			rt.home_alt = home.alt;
		}
	}

	// If no valid home position has been injected yet, derive one from global position.
	bool published_fallback = false;
	if (!rt.home_set) {
		vehicle_global_position_s gpos{};
		if (rt.sub_global_position.update(&gpos)) {
			if (gpos.lat_lon_valid && gpos.alt_valid) {
				rt.home_set = true;
				rt.home_lat = gpos.lat;
				rt.home_lon = gpos.lon;
				rt.home_alt = gpos.alt;
				published_fallback = true;
		}
	}
	}

	if (published_fallback) {
		home_position_s home_msg{};
		home_msg.timestamp = time_us;
		home_msg.lat = rt.home_lat;
		home_msg.lon = rt.home_lon;
		home_msg.alt = rt.home_alt;
		home_msg.valid_hpos = true;
		home_msg.valid_alt = true;
		home_msg.valid_lpos = true;
		home_msg.update_count = ++rt.home_update_count;
		rt.pub_home.publish(home_msg);
	}
}

static void maybe_force_geofence_ready(LockstepRuntime &rt, uint64_t time_us)
{
	if (rt.cfg.enable_commander) {
		return;
	}

	geofence_status_s geofence{};
	if (rt.sub_geofence_status_dbg.copy(&geofence)) {
		if (geofence.status == geofence_status_s::GF_STATUS_READY) {
			return;
		}
	}

	geofence.timestamp = time_us;
	geofence.geofence_id = 0u;
	geofence.status = geofence_status_s::GF_STATUS_READY;
	rt.pub_geofence_status.publish(geofence);
}

// -----------------------------------------------------------------------------
// Generic uORB helper (topic lookup + queued publish)
// -----------------------------------------------------------------------------

static int publish_queued_uorb(LockstepRuntime &rt)
{
	for (auto &p : rt.ext_pubs) {
		if (!p.pending || (p.meta == nullptr)) {
			continue;
		}
		if (p.buffer.size() != static_cast<size_t>(p.meta->o_size)) {
			PX4_ERR("queued buffer size mismatch for %s", p.meta->o_name);
			p.pending = false;
			return -1;
		}
		if (p.handle == nullptr) {
			p.handle = orb_advertise_multi(
				p.meta,
				p.buffer.data(),
				&p.instance);
			if (p.handle == nullptr) {
				PX4_ERR("orb_advertise failed for %s", p.meta->o_name);
				p.pending = false;
				return -2;
			}

			if (p.requested_instance >= 0 && p.instance != p.requested_instance) {
				PX4_ERR("orb_advertise_multi assigned instance %d but requested %d for %s",
					p.instance,
					p.requested_instance,
					p.meta->o_name);
				p.pending = false;
				return -4;
			}
		} else {
			if (orb_publish(p.meta, p.handle, p.buffer.data()) != 0) {
				PX4_ERR("orb_publish failed for %s", p.meta->o_name);
				p.pending = false;
				return -3;
			}
		}
		p.pending = false;
	}
	return 0;
}

static void debug_state(LockstepRuntime &rt, uint64_t now_us);

[[maybe_unused]] static void commander_lite_step(LockstepRuntime &rt, uint64_t time_us)
{
	if (rt.cfg.enable_commander != 0) {
		return;
	}

	const bool cmd_armed = (rt.cmd.armed != 0);
	bool req_mission = (rt.cmd.request_mission != 0);
	const bool req_rtl = (rt.cmd.request_rtl != 0);

	mission_result_s mres{};
	const bool has_mission_result = rt.sub_mission_result.copy(&mres);
	const bool mission_valid = has_mission_result && mres.valid;
	const bool mission_unknown = !has_mission_result;
	const bool mission_finished = has_mission_result && mres.finished;
	const uint16_t mission_seq = has_mission_result ? mres.seq_current : 0;
	const uint16_t mission_count = has_mission_result ? mres.seq_total : 0;

	vehicle_land_detected_s land{};
	const bool has_land = rt.sub_land_detected.copy(&land);
	const bool landed = has_land ? land.landed : false;

	home_position_s home{};
	const bool has_home = rt.sub_home_position.copy(&home);
	const bool home_ready = has_home && home.valid_hpos && home.valid_lpos && home.valid_alt && (home.update_count > 0);

	vehicle_local_position_s lpos{};
	const bool has_lpos = rt.sub_vehicle_local_position.copy(&lpos);
	const bool est_ready = has_lpos && lpos.xy_valid && lpos.z_valid && lpos.v_xy_valid && lpos.v_z_valid;

	const bool can_start_mission = cmd_armed && req_mission && home_ready && est_ready &&
				       (mission_valid || mission_unknown);
	const bool can_mark_mission_started = cmd_armed && req_mission && home_ready && est_ready && mission_valid;

	if (!cmd_armed) {
		rt.cmd_mission_started = false;
		rt.cmd_mission_completed = false;
	} else {
		if (!req_mission) {
			rt.cmd_mission_started = false;
			rt.cmd_mission_completed = false;
		} else if (can_mark_mission_started) {
			rt.cmd_mission_started = true;
		}
		if (rt.cmd_mission_started && mission_finished && landed
		    && mission_count > 0 && mission_seq >= static_cast<uint16_t>(mission_count - 1)) {
			rt.cmd_mission_completed = true;
		}
	}

	if (rt.cmd_mission_completed) {
		req_mission = false;
	}

	uint8_t nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
	if (cmd_armed) {
		if (req_rtl) {
			nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
		} else if (rt.cmd_mission_completed) {
			nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
		} else if (can_start_mission) {
			nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
		}
	}

	const bool auto_mode = (nav_state != vehicle_status_s::NAVIGATION_STATE_MANUAL);

	vehicle_status_s status{};
	status.timestamp = time_us;
	status.arming_state = cmd_armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
	status.nav_state = nav_state;
	status.nav_state_user_intention = nav_state;
	status.nav_state_timestamp = time_us;
	status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	status.is_vtol = false;
	status.is_vtol_tailsitter = false;
	status.in_transition_mode = false;
	status.in_transition_to_fw = false;
	rt.pub_vehicle_status.publish(status);

	vehicle_control_mode_s vcm{};
	vcm.timestamp = time_us;
	vcm.flag_armed = cmd_armed;
	vcm.flag_multicopter_position_control_enabled = true;
	vcm.flag_control_manual_enabled = !auto_mode;
	vcm.flag_control_auto_enabled = auto_mode;
	vcm.flag_control_offboard_enabled = false;
	vcm.flag_control_position_enabled = true;
	vcm.flag_control_velocity_enabled = true;
	vcm.flag_control_altitude_enabled = true;
	vcm.flag_control_climb_rate_enabled = true;
	vcm.flag_control_acceleration_enabled = false;
	vcm.flag_control_attitude_enabled = true;
	vcm.flag_control_rates_enabled = true;
	vcm.flag_control_allocation_enabled = (rt.cfg.enable_control_allocator != 0);
	vcm.flag_control_termination_enabled = false;
	vcm.source_id = 0;
	rt.pub_vehicle_control_mode.publish(vcm);

	actuator_armed_s armed{};
	armed.timestamp = time_us;
	armed.armed = cmd_armed;
	armed.prearmed = cmd_armed;
	armed.ready_to_arm = true;
	armed.lockdown = false;
	armed.kill = false;
	armed.termination = false;
	armed.in_esc_calibration_mode = false;
	rt.pub_actuator_armed.publish(armed);
}

[[maybe_unused]] static void commander_lite_filter_mission_result(LockstepRuntime &rt, uint64_t time_us)
{
	if (rt.cfg.enable_commander != 0) {
		return;
	}

	mission_result_s mres{};
	if (!rt.sub_mission_result.copy(&mres)) {
		return;
	}

	if (!rt.cmd_mission_started && mres.finished) {
		mres.finished = false;
		mres.timestamp = time_us;
		rt.pub_mission_result.publish(mres);
	}
}

static int step_lockstep_common(LockstepRuntime &rt, uint64_t time_us)
{
	// Basic monotonic guarantee
	if (rt.last_time_us != 0 && time_us < rt.last_time_us) {
		PX4_WARN("lockstep: time went backwards (%llu -> %llu)",
			(unsigned long long)rt.last_time_us,
			(unsigned long long)time_us);
	}
	rt.last_time_us = time_us;

	// Drive PX4 timebase
	hrt_lockstep_set_absolute_time(time_us);

	const int uorb_ret = publish_queued_uorb(rt);
	if (uorb_ret != 0) {
		return uorb_ret;
	}

	maybe_publish_home_position_fallback(rt, time_us);

	// Deterministic stepping order
	const uint64_t now = time_us;

	// Keep legacy commander-disabled behavior for the Julia lockstep harness:
	// when Commander is absent, external uORB injections provide arming/mode state.
	if (rt.commander && rt.cmd_rate.should_run(now)) {
		rt.commander->run_once();
	}

	if (rt.nav_rate.should_run(now)) {
		rt.navigator->run_once();
	}

	if (rt.flight_mode_mgr) {
		rt.flight_mode_mgr->run_once();
	}

	if (rt.pos_rate.should_run(now)) {
		rt.mc_pos->run_once();
	}

	if (rt.att_rate.should_run(now)) {
		rt.mc_att->run_once();
	}

	if (rt.rate_rate.should_run(now)) {
		rt.mc_rate->run_once();
	}

	if (rt.control_alloc && rt.alloc_rate.should_run(now)) {
		rt.control_alloc->run_once();
	}

	maybe_force_geofence_ready(rt, time_us);

	debug_state(rt, time_us);
	return 0;
}

static void debug_state(LockstepRuntime &rt, uint64_t now_us)
{
	if (!rt.debug_enabled) {
		return;
	}
	if (rt.last_debug_us != 0 && (now_us - rt.last_debug_us) < 1000000ULL) {
		return;
	}
	rt.last_debug_us = now_us;

	vehicle_status_s vstatus{};
	const bool has_status = rt.sub_vehicle_status_dbg.update(&vstatus);

	vehicle_control_mode_s vcm{};
	const bool has_vcm = rt.sub_vehicle_control_mode_dbg.update(&vcm);

	position_setpoint_triplet_s triplet{};
	const bool has_triplet = rt.sub_pos_sp_triplet.update(&triplet);

	trajectory_setpoint_s traj{};
	const bool has_traj = rt.sub_traj_sp.copy(&traj);
	vehicle_constraints_s constraints{};
	const bool has_constraints = rt.sub_vehicle_constraints_dbg.update(&constraints);

	home_position_s home{};
	const bool has_home = rt.sub_home_position.copy(&home);

	vehicle_global_position_s gpos{};
	const bool has_gpos = rt.sub_global_position.copy(&gpos);

	geofence_status_s geofence{};
	const bool has_geofence = rt.sub_geofence_status_dbg.copy(&geofence);

	mission_result_s mres{};
	const bool has_mission_result = rt.sub_mission_result_dbg.copy(&mres);

	vehicle_attitude_setpoint_s att_sp{};
	const bool has_att_sp = rt.sub_att_sp.copy(&att_sp);

	vehicle_rates_setpoint_s rates_sp{};
	const bool has_rates_sp = rt.sub_rates_sp.copy(&rates_sp);

	PX4_INFO("lockstep dbg t=%llu status=%d auto=%d armed=%d", (unsigned long long)now_us,
		int(has_status), int(has_vcm ? vcm.flag_control_auto_enabled : false),
		int(has_status ? vstatus.arming_state == vehicle_status_s::ARMING_STATE_ARMED : false));

	if (has_triplet) {
		PX4_INFO("lockstep triplet valid=%d type=%u alt=%.2f lat=%.6f lon=%.6f", (int)triplet.current.valid,
			(unsigned)triplet.current.type, (double)triplet.current.alt, triplet.current.lat, triplet.current.lon);
	} else {
		PX4_INFO("lockstep triplet none");
	}

	if (has_traj) {
		PX4_INFO("lockstep traj z=%.2f vz=%.2f az=%.2f yaw=%.2f", (double)traj.position[2],
			(double)traj.velocity[2], (double)traj.acceleration[2], (double)traj.yaw);
	} else {
		PX4_INFO("lockstep traj none");
	}

	if (has_constraints) {
		PX4_INFO("lockstep constraints takeoff=%d speed_up=%.2f", (int)constraints.want_takeoff,
			(double)constraints.speed_up);
	} else {
		PX4_INFO("lockstep constraints none");
	}

	if (has_home) {
		PX4_INFO("lockstep home valid_hpos=%d valid_alt=%d lat=%.6f lon=%.6f alt=%.2f upd=%u",
			(int)home.valid_hpos, (int)home.valid_alt, home.lat, home.lon, (double)home.alt,
			(unsigned)home.update_count);
	} else {
		PX4_INFO("lockstep home none");
	}

	if (has_gpos) {
		PX4_INFO("lockstep gpos valid_hpos=%d valid_alt=%d lat=%.6f lon=%.6f alt=%.2f",
			(int)gpos.lat_lon_valid, (int)gpos.alt_valid, gpos.lat, gpos.lon, (double)gpos.alt);
	} else {
		PX4_INFO("lockstep gpos none");
	}

	if (has_geofence) {
		PX4_INFO("lockstep geofence status=%u id=%u", (unsigned)geofence.status,
			(unsigned)geofence.geofence_id);
	} else {
		PX4_INFO("lockstep geofence none");
	}

	if (has_mission_result) {
		PX4_INFO("lockstep mission valid=%d count=%u seq=%u home_cnt=%u",
			(int)mres.valid, (unsigned)mres.seq_total, (unsigned)mres.seq_current,
			(unsigned)mres.home_position_counter);
	} else {
		PX4_INFO("lockstep mission none");
	}

	const double nan = std::numeric_limits<double>::quiet_NaN();
	const double thrust = has_att_sp ? static_cast<double>(att_sp.thrust_body[2]) : nan;
	const double rate_x = has_rates_sp ? static_cast<double>(rates_sp.roll) : nan;
	const double rate_y = has_rates_sp ? static_cast<double>(rates_sp.pitch) : nan;
	const double rate_z = has_rates_sp ? static_cast<double>(rates_sp.yaw) : nan;
	PX4_INFO("lockstep outputs thrust=%.3f rates=(%.3f,%.3f,%.3f)",
		thrust,
		rate_x,
		rate_y,
		rate_z);
}

PX4_LOCKSTEP_EXPORT int px4_lockstep_step_uorb(px4_lockstep_handle_t handle,
					 uint64_t time_us)
{
	LockstepRuntime *rt = reinterpret_cast<LockstepRuntime *>(handle);
	if (!rt) {
		return -1;
	}

	ensure_lockstep_thread_name();
	return step_lockstep_common(*rt, time_us);
}

PX4_LOCKSTEP_EXPORT int px4_lockstep_orb_topic_metadata(px4_lockstep_handle_t handle,
                                                       const char *topic_name,
                                                       const char **out_fields,
                                                       uint32_t *out_size,
                                                       uint32_t *out_size_no_padding,
                                                       uint32_t *out_message_hash,
                                                       uint8_t *out_queue_size)
{
	LockstepRuntime *rt = reinterpret_cast<LockstepRuntime *>(handle);
	(void)rt;
	if (!rt || (topic_name == nullptr) || (out_fields == nullptr) || (out_size == nullptr)
	    || (out_size_no_padding == nullptr)) {
		return -1;
	}

	const struct orb_metadata *meta = orb_meta_from_name(topic_name);
	if (meta == nullptr) {
		return -2;
	}

	if (out_message_hash) {
		*out_message_hash = get_message_hash(meta);
	}
	if (out_queue_size) {
		*out_queue_size = get_queue_size(meta);
	}

	const char *fields = get_o_fields(meta);
	if (fields == nullptr) {
		*out_fields = nullptr;
		*out_size = static_cast<uint32_t>(meta->o_size);
		*out_size_no_padding = get_o_size_no_padding(meta);
		return -3;
	}

	*out_fields = fields;
	*out_size = static_cast<uint32_t>(meta->o_size);
	*out_size_no_padding = get_o_size_no_padding(meta);
	return 0;
}

PX4_LOCKSTEP_EXPORT int px4_lockstep_orb_create_publisher(px4_lockstep_handle_t handle,
					 const char *topic_name,
					 int32_t priority,
					 uint32_t queue_size,
					 px4_lockstep_uorb_pub_t *out_pub_id,
					 int32_t *out_instance,
					 uint32_t *out_msg_size)
{
	return px4_lockstep_orb_create_publisher_ex(
		handle,
		topic_name,
		priority,
		queue_size,
		-1, /* requested_instance */
		out_pub_id,
		out_instance,
		out_msg_size);
}

PX4_LOCKSTEP_EXPORT int px4_lockstep_orb_create_publisher_ex(px4_lockstep_handle_t handle,
					    const char *topic_name,
					    int32_t priority,
					    uint32_t queue_size,
					    int32_t requested_instance,
					    px4_lockstep_uorb_pub_t *out_pub_id,
					    int32_t *out_instance,
					    uint32_t *out_msg_size)
{
	LockstepRuntime *rt = reinterpret_cast<LockstepRuntime *>(handle);
	if (!rt || (topic_name == nullptr) || (out_pub_id == nullptr)) {
		return -1;
	}
	if (priority > 0) {
		PX4_ERR("uORB priority not supported for %s", topic_name);
		return -5;
	}

	const struct orb_metadata *meta = orb_meta_from_name(topic_name);
	if (meta == nullptr) {
		PX4_ERR("unknown uORB topic: %s", topic_name);
		return -2;
	}
	if (queue_size > 0 && meta->o_queue != queue_size) {
		PX4_ERR("uORB queue size mismatch for %s (requested %u, topic %u)",
			topic_name,
			(unsigned)queue_size,
			(unsigned)meta->o_queue);
		return -6;
	}

	LockstepRuntime::ExtPublisher pub{};
	pub.meta = meta;
	pub.priority = (priority > 0) ? priority : 0;
	pub.queue_size = (queue_size > 0) ? queue_size : 1u;
	pub.requested_instance = requested_instance;
	pub.instance = (requested_instance >= 0) ? requested_instance : -1;
	pub.handle = nullptr;
	pub.pending = false;
	pub.buffer.resize(meta->o_size);

	rt->ext_pubs.push_back(std::move(pub));
	const int32_t id = static_cast<int32_t>(rt->ext_pubs.size() - 1);
	*out_pub_id = id;
	if (out_instance) {
		*out_instance = rt->ext_pubs[id].instance;
	}
	if (out_msg_size) {
		*out_msg_size = rt->ext_pubs[id].meta->o_size;
	}
	return 0;
}

PX4_LOCKSTEP_EXPORT int px4_lockstep_orb_queue_publish(px4_lockstep_handle_t handle,
				      px4_lockstep_uorb_pub_t pub_id,
				      const void *msg,
				      uint32_t msg_size)
{
	LockstepRuntime *rt = reinterpret_cast<LockstepRuntime *>(handle);
	if (!rt || (msg == nullptr)) {
		return -1;
	}
	if (pub_id < 0 || static_cast<size_t>(pub_id) >= rt->ext_pubs.size()) {
		return -2;
	}

	auto &pub = rt->ext_pubs[pub_id];
	if (pub.meta == nullptr) {
		return -3;
	}
	if (msg_size != pub.meta->o_size) {
		PX4_ERR("uORB publish size mismatch for %s (got %u expected %u)",
			pub.meta->o_name,
			(unsigned)msg_size,
			(unsigned)pub.meta->o_size);
		return -4;
	}

	if (pub.buffer.size() != msg_size) {
		pub.buffer.resize(msg_size);
	}
	std::memcpy(pub.buffer.data(), msg, msg_size);
	pub.pending = true;
	return 0;
}

PX4_LOCKSTEP_EXPORT int px4_lockstep_orb_publisher_instance(px4_lockstep_handle_t handle,
					   px4_lockstep_uorb_pub_t pub_id,
					   int32_t *out_instance)
{
	LockstepRuntime *rt = reinterpret_cast<LockstepRuntime *>(handle);
	if (!rt || (out_instance == nullptr)) {
		return -1;
	}
	if (pub_id < 0 || static_cast<size_t>(pub_id) >= rt->ext_pubs.size()) {
		return -2;
	}
	*out_instance = rt->ext_pubs[pub_id].instance;
	return 0;
}

PX4_LOCKSTEP_EXPORT int px4_lockstep_orb_create_subscriber(px4_lockstep_handle_t handle,
					  const char *topic_name,
					  uint32_t instance,
					  px4_lockstep_uorb_sub_t *out_sub_id,
					  uint32_t *out_msg_size)
{
	LockstepRuntime *rt = reinterpret_cast<LockstepRuntime *>(handle);
	if (!rt || (topic_name == nullptr) || (out_sub_id == nullptr)) {
		return -1;
	}

	const struct orb_metadata *meta = orb_meta_from_name(topic_name);
	if (meta == nullptr) {
		PX4_ERR("unknown uORB topic: %s", topic_name);
		return -2;
	}

	int h = orb_subscribe_multi(meta, instance);
	if (h < 0) {
		PX4_ERR("orb_subscribe_multi failed for %s[%u] (errno=%d)",
			topic_name,
			(unsigned)instance,
			errno);
		return -3;
	}

	LockstepRuntime::ExtSubscriber sub{};
	sub.meta = meta;
	sub.handle = h;
	rt->ext_subs.push_back(sub);
	const int32_t id = static_cast<int32_t>(rt->ext_subs.size() - 1);
	*out_sub_id = id;
	if (out_msg_size) {
		*out_msg_size = meta->o_size;
	}
	return 0;
}

PX4_LOCKSTEP_EXPORT int px4_lockstep_orb_check(px4_lockstep_handle_t handle,
				      px4_lockstep_uorb_sub_t sub_id,
				      int32_t *out_updated)
{
	LockstepRuntime *rt = reinterpret_cast<LockstepRuntime *>(handle);
	if (!rt || (out_updated == nullptr)) {
		return -1;
	}
	if (sub_id < 0 || static_cast<size_t>(sub_id) >= rt->ext_subs.size()) {
		return -2;
	}

	auto &sub = rt->ext_subs[sub_id];
	if (sub.handle < 0) {
		return -3;
	}

	bool updated = false;
	const int ret = orb_check(sub.handle, &updated);
	*out_updated = updated ? 1 : 0;
	return ret;
}

PX4_LOCKSTEP_EXPORT int px4_lockstep_orb_copy(px4_lockstep_handle_t handle,
				     px4_lockstep_uorb_sub_t sub_id,
				     void *out_msg,
				     uint32_t msg_size)
{
	LockstepRuntime *rt = reinterpret_cast<LockstepRuntime *>(handle);
	if (!rt || (out_msg == nullptr)) {
		return -1;
	}
	if (sub_id < 0 || static_cast<size_t>(sub_id) >= rt->ext_subs.size()) {
		return -2;
	}

	auto &sub = rt->ext_subs[sub_id];
	if (!sub.meta || sub.handle < 0) {
		return -3;
	}
	if (msg_size != sub.meta->o_size) {
		PX4_ERR("uORB copy size mismatch for %s (got %u expected %u)",
			sub.meta->o_name,
			(unsigned)msg_size,
			(unsigned)sub.meta->o_size);
		return -4;
	}

	return orb_copy(sub.meta, sub.handle, out_msg);
}

PX4_LOCKSTEP_EXPORT int px4_lockstep_orb_unsubscribe(px4_lockstep_handle_t handle,
				     px4_lockstep_uorb_sub_t sub_id)
{
	LockstepRuntime *rt = reinterpret_cast<LockstepRuntime *>(handle);
	if (!rt) {
		return -1;
	}
	if (sub_id < 0 || static_cast<size_t>(sub_id) >= rt->ext_subs.size()) {
		return -2;
	}

	auto &sub = rt->ext_subs[sub_id];
	if (sub.handle >= 0) {
		const int ret = orb_unsubscribe(sub.handle);
		sub.handle = -1;
		return ret;
	}
	return 0;
}

} // extern "C"
