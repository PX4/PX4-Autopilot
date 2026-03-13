#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
#  define PX4_LOCKSTEP_EXPORT __declspec(dllexport)
#else
#  define PX4_LOCKSTEP_EXPORT __attribute__((visibility("default")))
#endif

// Opaque handle managed by the library.
typedef void *px4_lockstep_handle_t;

// Minimal runtime config.
typedef struct px4_lockstep_config_t {
	// If non-zero, initialize Dataman with RAM backend.
	int32_t dataman_use_ram;

	// If non-zero, step Commander in lockstep.
	// Commander-in-loop is not supported right now; creation will fail if enabled.
	// When disabled, the lockstep harness publishes a minimal replacement set of topics.
	int32_t enable_commander;          // 0/1
	int32_t commander_rate_hz;         // <=0 => step every tick

	// Module stepping rates in Hz. If <= 0, the module is stepped every call.
	int32_t navigator_rate_hz;
	int32_t mc_pos_control_rate_hz;
	int32_t mc_att_control_rate_hz;
	int32_t mc_rate_control_rate_hz;

	// Enable and step the PX4 control allocator (mixing).
	//
	// NOTE: The lockstep harness does **not** configure actuator geometry.
	// Callers are expected to set CA_* parameters (rotor count/positions/KM, etc.)
	// before stepping the lockstep runtime.
	int32_t enable_control_allocator;      // 0/1
	int32_t control_allocator_rate_hz;     // <=0 => step every tick
} px4_lockstep_config_t;

// Minimal commander-lite command inputs.
//
// These are interpreted inside the lockstep harness when Commander-in-loop
// is disabled. The harness publishes vehicle_status/control_mode/actuator_armed
// based on these inputs and mission_result.
typedef struct px4_lockstep_cmd_t {
	uint8_t armed;            // 0/1
	uint8_t request_mission;  // 0/1
	uint8_t request_rtl;      // 0/1
} px4_lockstep_cmd_t;

// -----------------------------------------------------------------------------
// PX4 parameter set/get helpers
// -----------------------------------------------------------------------------
//
// These are intentionally small wrappers over the PX4 parameter system so the
// Julia-side simulator can configure PX4 deterministically from an aircraft spec.
//
// Returns 0 on success, <0 on error:
//  -1: invalid arguments
//  -2: unknown parameter
//  -3: type mismatch
//  -4: set/get failed

PX4_LOCKSTEP_EXPORT int px4_lockstep_param_set_i32(px4_lockstep_handle_t handle,
                                                  const char *name,
                                                  int32_t value);
PX4_LOCKSTEP_EXPORT int px4_lockstep_param_set_f32(px4_lockstep_handle_t handle,
                                                  const char *name,
                                                  float value);
PX4_LOCKSTEP_EXPORT int px4_lockstep_param_get_i32(px4_lockstep_handle_t handle,
                                                  const char *name,
                                                  int32_t *out_value);
PX4_LOCKSTEP_EXPORT int px4_lockstep_param_get_f32(px4_lockstep_handle_t handle,
                                                  const char *name,
                                                  float *out_value);
PX4_LOCKSTEP_EXPORT int px4_lockstep_param_notify(px4_lockstep_handle_t handle);

// -----------------------------------------------------------------------------
// Pre-init parameter staging
// -----------------------------------------------------------------------------
//
// These functions enqueue parameter changes that will be applied inside
// px4_lockstep_create() *before* PX4 modules are initialized. This is useful
// for deterministic startup configuration (e.g., CA_* geometry).
//
// Returns 0 on success, <0 on error:
//  -1: invalid arguments
//  -4: set failed
PX4_LOCKSTEP_EXPORT int px4_lockstep_param_preinit_set_i32(const char *name,
                                                          int32_t value);
PX4_LOCKSTEP_EXPORT int px4_lockstep_param_preinit_set_f32(const char *name,
                                                          float value);

// -----------------------------------------------------------------------------
// Control allocator param refresh (lockstep helper)
// -----------------------------------------------------------------------------
//
// Force the control allocator to re-read its parameters immediately.
// This avoids a global param_notify broadcast when only CA_* geometry changed.
//
// Returns 0 on success, <0 on error:
//  -1: invalid arguments
//  -2: control allocator not enabled
PX4_LOCKSTEP_EXPORT int px4_lockstep_control_alloc_update_params(px4_lockstep_handle_t handle);

// ABI compatibility
//
// The Julia wrapper assumes the exact memory layout of the config struct.
// These helper functions allow a runtime handshake so mismatches fail fast.
//
// Note: input/output structs were removed; in_sz/out_sz are set to 0.
#define PX4_LOCKSTEP_ABI_VERSION 3u

PX4_LOCKSTEP_EXPORT uint32_t px4_lockstep_abi_version(void);
PX4_LOCKSTEP_EXPORT void px4_lockstep_sizes(uint32_t *in_sz,
				    uint32_t *out_sz,
				    uint32_t *cfg_sz);

// Create/destroy.
PX4_LOCKSTEP_EXPORT px4_lockstep_handle_t px4_lockstep_create(const px4_lockstep_config_t *cfg);
PX4_LOCKSTEP_EXPORT void px4_lockstep_destroy(px4_lockstep_handle_t handle);

// Update the commander-lite command inputs (arm/mission/RTL requests).
// Returns 0 on success, <0 on invalid arguments.
PX4_LOCKSTEP_EXPORT int px4_lockstep_set_cmd(px4_lockstep_handle_t handle,
					     const px4_lockstep_cmd_t *cmd);

// Load a mission (QGC WPL 110) and preload it into Dataman.
// Returns 0 on success.
PX4_LOCKSTEP_EXPORT int px4_lockstep_load_mission_qgc_wpl(px4_lockstep_handle_t handle, const char *mission_path);

// One lockstep tick (uORB-only input/output).
//
// This entrypoint advances the PX4 timebase and runs modules. Inputs/outputs
// are supplied via the generic uORB publish/subscribe API.
//
// Returns 0 on success, <0 on error:
//  -1: queued buffer size mismatch
//  -2: uORB advertise failed
//  -3: uORB publish failed
//  -4: requested publisher instance mismatch
PX4_LOCKSTEP_EXPORT int px4_lockstep_step_uorb(px4_lockstep_handle_t handle,
					 uint64_t time_us);

// -----------------------------------------------------------------------------
// uORB metadata queries
// -----------------------------------------------------------------------------
//
// Expose uORB compile-time topic metadata to the Julia side so message layout
// compatibility can be validated against the *loaded* PX4 binary at runtime.
//
// out_fields points to a static string owned by PX4/uORB metadata. Callers must
// copy it if they need to retain it. If the PX4 build does not expose field
// metadata, out_fields is set to NULL and the function returns -3.
// out_size returns the expected sizeof(<topic>_s) for the topic.
// out_size_no_padding is 0 if the PX4 version does not expose a
// "size without padding" field in the uORB metadata.
// out_message_hash returns PX4's 32-bit message hash (0 if unavailable).
// out_queue_size returns the topic queue size (0 if unavailable).
//
// Returns 0 on success, <0 on error:
//  -1: invalid arguments
//  -2: unknown topic
//  -3: topic has no field description string
PX4_LOCKSTEP_EXPORT int px4_lockstep_orb_topic_metadata(px4_lockstep_handle_t handle,
                                                       const char *topic_name,
                                                       const char **out_fields,
                                                       uint32_t *out_size,
                                                       uint32_t *out_size_no_padding,
                                                       uint32_t *out_message_hash,
                                                       uint8_t *out_queue_size);

// -----------------------------------------------------------------------------
// Generic uORB publish/subscribe interface (experimental)
// -----------------------------------------------------------------------------
//
// Motivation:
// - Enable multi-rate sensor injection (e.g. IMU @ 1kHz, GPS @ 5Hz) from Julia.
//
// Design notes:
// - Publishers are created once (init time). Messages are queued from Julia using
//   px4_lockstep_orb_queue_publish() and published inside px4_lockstep_step_uorb()
//   after the lockstep time has been updated.
// - Message memory layout must exactly match PX4's generated uORB C structs.
//   The intended workflow is to auto-generate Julia structs from PX4's .msg
//   definitions (or from the generated uORB topic headers).

typedef int32_t px4_lockstep_uorb_pub_t;
typedef int32_t px4_lockstep_uorb_sub_t;

// Create a uORB publisher (advertisement is deferred until first queued publish).
//
// priority: currently not supported by the C API; must be 0.
// queue_size: currently cannot override the compiled uORB queue size. If >0 it must
// match the topic's queue size; pass 0 to skip the check.
//
// out_instance is -1 until the first publish (uORB assigns instances on advertise).
// out_msg_size returns the expected sizeof(topic_struct).
PX4_LOCKSTEP_EXPORT int px4_lockstep_orb_create_publisher(px4_lockstep_handle_t handle,
                                                         const char *topic_name,
                                                         int32_t priority,
                                                         uint32_t queue_size,
                                                         px4_lockstep_uorb_pub_t *out_pub_id,
                                                         int32_t *out_instance,
                                                         uint32_t *out_msg_size);

// Create a uORB publisher with a deterministic requested instance.
//
// requested_instance: -1 for auto (default uORB behavior), otherwise the desired instance.
//
// Note: advertisement is still deferred until the first queued publish (the first
// px4_lockstep_step_uorb() that processes a pending publish). The requested instance is
// enforced at advertise time: if uORB assigns a different instance, the lockstep step
// call fails with -4.
PX4_LOCKSTEP_EXPORT int px4_lockstep_orb_create_publisher_ex(px4_lockstep_handle_t handle,
                                                            const char *topic_name,
                                                            int32_t priority,
                                                            uint32_t queue_size,
                                                            int32_t requested_instance,
                                                            px4_lockstep_uorb_pub_t *out_pub_id,
                                                            int32_t *out_instance,
                                                            uint32_t *out_msg_size);

// Queue a publish for the next px4_lockstep_step_uorb().
// The message bytes are copied internally.
PX4_LOCKSTEP_EXPORT int px4_lockstep_orb_queue_publish(px4_lockstep_handle_t handle,
                                                      px4_lockstep_uorb_pub_t pub_id,
                                                      const void *msg,
                                                      uint32_t msg_size);

// Get the uORB instance id associated with a publisher (assigned on advertise).
PX4_LOCKSTEP_EXPORT int px4_lockstep_orb_publisher_instance(px4_lockstep_handle_t handle,
                                                           px4_lockstep_uorb_pub_t pub_id,
                                                           int32_t *out_instance);

// Create a uORB subscriber.
// out_msg_size returns the expected sizeof(topic_struct).
PX4_LOCKSTEP_EXPORT int px4_lockstep_orb_create_subscriber(px4_lockstep_handle_t handle,
                                                          const char *topic_name,
                                                          uint32_t instance,
                                                          px4_lockstep_uorb_sub_t *out_sub_id,
                                                          uint32_t *out_msg_size);

// Check whether a topic has been updated since the last copy.
PX4_LOCKSTEP_EXPORT int px4_lockstep_orb_check(px4_lockstep_handle_t handle,
                                              px4_lockstep_uorb_sub_t sub_id,
                                              int32_t *out_updated);

// Copy the latest topic data into out_msg.
PX4_LOCKSTEP_EXPORT int px4_lockstep_orb_copy(px4_lockstep_handle_t handle,
                                             px4_lockstep_uorb_sub_t sub_id,
                                             void *out_msg,
                                             uint32_t msg_size);

// Unsubscribe and free the subscription handle.
PX4_LOCKSTEP_EXPORT int px4_lockstep_orb_unsubscribe(px4_lockstep_handle_t handle,
                                                    px4_lockstep_uorb_sub_t sub_id);

#ifdef __cplusplus
}
#endif
