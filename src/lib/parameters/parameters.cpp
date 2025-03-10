/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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

/**
 * @file parameters.cpp
 *
 * Global parameter store.
 *
 * Note that it might make sense to convert this into a driver.  That would
 * offer some interesting options regarding state for e.g. ORB advertisements
 * and background parameter saving.
 */

#define PARAM_IMPLEMENTATION
#include "param.h"
#include "param_translation.h"
#include <parameters/px4_parameters.hpp>
#include <lib/tinybson/tinybson.h>

#include <crc32.h>
#include <float.h>
#include <math.h>

#include <containers/Bitset.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic_bitset.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/sem.h>
#include <px4_platform_common/shutdown.h>
#include <px4_platform_common/micro_hal.h>

using namespace time_literals;

#include "uORB/uORB.h"
#include "uORB/topics/parameter_update.h"
#include <uORB/Subscription.hpp>

#include "ExhaustiveLayer.h"
#include "ConstLayer.h"
#include "DynamicSparseLayer.h"
#include "StaticSparseLayer.h"

#include "atomic_transaction.h"

/* Include functions common to user and kernel sides */
#include "parameters_common.cpp"

#if defined(__PX4_NUTTX) && !defined(CONFIG_BUILD_FLAT)
#include <px4_platform/board_ctrl.h>
#include "parameters_ioctl.h"
#endif

#if defined(FLASH_BASED_PARAMS)
#include "flashparams/flashparams.h"
#else
inline static int flash_param_save(param_filter_func filter) { return -1; }
inline static int flash_param_load() { return -1; }
inline static int flash_param_import() { return -1; }
#endif

static char *param_default_file = nullptr;
static char *param_backup_file = nullptr;

#include "autosave.h"
static ParamAutosave *autosave_instance {nullptr};

static px4::AtomicBitset<param_info_count> params_active;  // params found
static px4::AtomicBitset<param_info_count> params_unsaved;

static ConstLayer firmware_defaults;
static DynamicSparseLayer runtime_defaults{&firmware_defaults};
DynamicSparseLayer user_config{&runtime_defaults};

/** parameter update topic handle */
#if not defined(CONFIG_PARAM_REMOTE)
static orb_advert_t param_topic = nullptr;
static unsigned int param_instance = 0;
#endif

static perf_counter_t param_export_perf;
static perf_counter_t param_find_perf;
static perf_counter_t param_get_perf;
static perf_counter_t param_set_perf;

static pthread_mutex_t file_mutex  =
	PTHREAD_MUTEX_INITIALIZER; ///< this protects against concurrent param saves (file or flash access).

// Support for remote parameter node
#if defined(CONFIG_PARAM_PRIMARY)
# include "parameters_primary.h"
#endif // CONFIG_PARAM_PRIMARY
#if defined(CONFIG_PARAM_REMOTE)
# include "parameters_remote.h"
#endif // CONFIG_PARAM_REMOTE

void
param_init()
{
	param_export_perf = perf_alloc(PC_ELAPSED, "param: export");
	param_find_perf = perf_alloc(PC_COUNT, "param: find");
	param_get_perf = perf_alloc(PC_COUNT, "param: get");
	param_set_perf = perf_alloc(PC_ELAPSED, "param: set");

#if defined(__PX4_NUTTX) && !defined(CONFIG_BUILD_FLAT)
	px4_register_boardct_ioctl(_PARAMIOCBASE, param_ioctl);
#endif

#if defined(CONFIG_PARAM_PRIMARY)
	param_primary_init();
#endif // CONFIG_PARAM_PRIMARY

#if defined(CONFIG_PARAM_REMOTE)
	param_remote_init();
#endif // CONFIG_PARAM_REMOTE

#if not defined(CONFIG_PARAM_REMOTE)
	autosave_instance = new ParamAutosave();
#endif
}


void
param_notify_changes()
{
// Don't send if this is a remote node. Only the primary
// sends out update notices
#if not defined(CONFIG_PARAM_REMOTE)
	parameter_update_s pup {};
	pup.instance = param_instance++;
	pup.get_count = perf_event_count(param_get_perf);
	pup.set_count = perf_event_count(param_set_perf);
	pup.find_count = perf_event_count(param_find_perf);
	pup.export_count = perf_event_count(param_export_perf);
	pup.active = params_active.count();
	pup.changed = user_config.size();
	pup.custom_default = runtime_defaults.size();
	pup.timestamp = hrt_absolute_time();

	if (param_topic == nullptr) {
		param_topic = orb_advertise(ORB_ID(parameter_update), &pup);

	} else {
		orb_publish(ORB_ID(parameter_update), param_topic, &pup);
	}

#endif
}

static param_t param_find_internal(const char *name, bool notification)
{
	perf_count(param_find_perf);

	param_t middle;
	param_t front = 0;
	param_t last = param_info_count;

	/* perform a binary search of the known parameters */

	while (front <= last) {
		middle = front + (last - front) / 2;
		int ret = strcmp(name, param_name(middle));

		if (ret == 0) {
			if (notification) {
				param_set_used(middle);
			}

			return middle;

		} else if (middle == front) {
			/* An end point has been hit, but there has been no match */
			break;

		} else if (ret < 0) {
			last = middle;

		} else {
			front = middle;
		}
	}

	/* not found */
	return PARAM_INVALID;
}

param_t param_find(const char *name)
{
	return param_find_internal(name, true);
}

param_t param_find_no_notification(const char *name)
{
	return param_find_internal(name, false);
}

unsigned param_count_used()
{
	return params_active.count();
}

param_t param_for_used_index(unsigned index)
{
	// walk all params and count used params
	if (index < param_info_count) {
		unsigned used_count = 0;

		for (int i = 0; i < params_active.size(); i++) {
			if (params_active[i]) {
				// we found the right used count,
				//  return the param value
				if (index == used_count) {
					return static_cast<param_t>(i);
				}

				used_count++;
			}
		}
	}

	return PARAM_INVALID;
}

int param_get_used_index(param_t param)
{
	/* this tests for out of bounds and does a constant time lookup */
	if (!param_used(param)) {
		return -1;
	}

	/* walk all params and count, now knowing that it has a valid index */
	int used_count = 0;

	for (int i = 0; i < params_active.size(); i++) {
		if (params_active[i]) {

			if (param == i) {
				return used_count;
			}

			used_count++;
		}
	}

	return -1;
}

bool
param_value_unsaved(param_t param)
{
	return handle_in_range(param) ? params_unsaved[param] : false;
}

int
param_get(param_t param, void *val)
{
	perf_count(param_get_perf);

	if (!handle_in_range(param)) {
		PX4_ERR("get: param %" PRId16 " invalid", param);
		return PX4_ERROR;
	}

	if (!params_active[param]) {
		PX4_DEBUG("get: param %" PRId16 " (%s) not active", param, param_name(param));
	}

	int result = PX4_ERROR;

	if (val) {

		auto retrieve_value = user_config.get(param);

		switch (param_type(param)) {
		case PARAM_TYPE_INT32:
			memcpy(val, &retrieve_value.i, sizeof(retrieve_value.i));
			return PX4_OK;

		case PARAM_TYPE_FLOAT:
			memcpy(val, &retrieve_value.f, sizeof(retrieve_value.f));
			return PX4_OK;
		}
	}

	return result;
}

int
param_get_default_value_internal(param_t param, void *default_val)
{
	if (!handle_in_range(param)) {
		PX4_ERR("get default value: param %d invalid", param);
		return PX4_ERROR;
	}

	if (default_val) {
		switch (param_type(param)) {
		case PARAM_TYPE_INT32: {
				int32_t val = runtime_defaults.get(param).i;
				memcpy(default_val, &val, sizeof(val));
				return PX4_OK;
			}

		case PARAM_TYPE_FLOAT: {
				float val = runtime_defaults.get(param).f;
				memcpy(default_val, &val, sizeof(val));
				return PX4_OK;
			}
		}
	}

	return PX4_ERROR;
}

int
param_get_default_value(param_t param, void *default_val)
{
	if (!handle_in_range(param)) {
		return PX4_ERROR;
	}

	int ret = param_get_default_value_internal(param, default_val);
	return ret;
}

bool param_value_is_default(param_t param)
{
	if (!handle_in_range(param)) {
		return true;
	}

	if (!user_config.contains(param)) {
		// if user config does not contain it, consider it default.
		return true;

	} else {
		// compare with default value
		const param_value_u user_config_value = user_config.get(param);
		const param_value_u runtime_default_value = runtime_defaults.get(param);

		switch (param_type(param)) {
		case PARAM_TYPE_INT32: {
				return user_config_value.i == runtime_default_value.i;
			}

		case PARAM_TYPE_FLOAT: {
				return user_config_value.f - runtime_default_value.f < FLT_EPSILON;
			}
		}
	}

	return true;
}

static void
param_autosave()
{
	if (autosave_instance) {
		autosave_instance->request();
	}
}

void
param_control_autosave(bool enable)
{
	if (autosave_instance) {
		autosave_instance->enable(enable);
	}
}

static int
param_set_internal(param_t param, const void *val, bool mark_saved, bool notify_changes, bool update_remote = true)
{
	if (!handle_in_range(param)) {
		PX4_ERR("set invalid param %d", param);
		return PX4_ERROR;
	}

	if (val == nullptr) {
		PX4_ERR("set invalid value");
		return PX4_ERROR;
	}

	int result = -1;
	bool param_changed = false;
	perf_begin(param_set_perf);

	const param_value_u user_config_value = user_config.get(param);
	param_value_u new_value{};

	switch (param_type(param)) {
	case PARAM_TYPE_INT32:
		memcpy(&new_value.i, val, sizeof(new_value.i));
		param_changed = user_config_value.i != new_value.i;
		break;

	case PARAM_TYPE_FLOAT:
		memcpy(&new_value.f, val, sizeof(new_value.f));
		param_changed = fabsf(user_config_value.f - new_value.f) > FLT_EPSILON;
		break;

	default: {
			PX4_ERR("param_set invalid param type for %s", param_name(param));
			break;
		}
	}

	if (user_config.store(param, new_value)) {
		params_unsaved.set(param, !mark_saved);
		result = PX4_OK;

	} else {
		PX4_ERR("param_set failed to store param %s", param_name(param));
		result = PX4_ERROR;
	}

	if ((result == PX4_OK) && param_changed && !mark_saved) { // this is false when importing parameters
		param_autosave();
	}

	// If this is the parameter server, make sure that the remote is updated
#if defined(CONFIG_PARAM_PRIMARY)

	if (param_changed && update_remote) {
		param_primary_set_value(param, val);
	}

#endif

	// If this is the parameter remote, make sure that the primary is updated
#if defined(CONFIG_PARAM_REMOTE)

	if (param_changed && update_remote) {
		param_remote_set_value(param, val);
	}

#endif

	perf_end(param_set_perf);

	/*
	 * If we set something, now that we have unlocked, go ahead and advertise that
	 * a thing has been set.
	 */
	if ((result == PX4_OK) && param_changed && notify_changes) {
		param_notify_changes();
	}

	return result;
}

#if defined(FLASH_BASED_PARAMS)
int param_set_external(param_t param, const void *val, bool mark_saved, bool notify_changes)
{
	return param_set_internal(param, val, mark_saved, notify_changes);
}

void param_get_external(param_t param, void *val)
{
	param_get(param, val);
}
#endif

int param_set(param_t param, const void *val)
{
	return param_set_internal(param, val, false, true);
}

int param_set_no_notification(param_t param, const void *val)
{
	return param_set_internal(param, val, false, false);
}

int param_set_no_remote_update(param_t param, const void *val, bool notify)
{
	return param_set_internal(param, val, false, notify, false);
}

bool param_used(param_t param)
{
	if (handle_in_range(param)) {
		return params_active[param];
	}

	return false;
}

void param_set_used(param_t param)
{
	if (handle_in_range(param)) {
#if defined(CONFIG_PARAM_REMOTE)

		if (!param_used(param)) {
			param_remote_set_used(param);
		}

#endif

		params_active.set(param, true);
	}
}

int param_set_default_value(param_t param, const void *val)
{
	if (!handle_in_range(param)) {
		PX4_ERR("set default value invalid param %d", param);
		return PX4_ERROR;
	}

	if (val == nullptr) {
		PX4_ERR("set default value invalid value");
		return PX4_ERROR;
	}

	int result = PX4_ERROR;


	// check if param being set to default value
	bool setting_to_static_default = false;
	const param_value_u firmware_default_value = firmware_defaults.get(param);

	switch (param_type(param)) {
	case PARAM_TYPE_INT32: {
			int32_t new_value;
			memcpy(&new_value, val, sizeof(new_value));
			setting_to_static_default = firmware_default_value.i == new_value;
			break;
		}

	case PARAM_TYPE_FLOAT: {
			float new_value;
			memcpy(&new_value, val, sizeof(new_value));
			setting_to_static_default = fabsf(firmware_default_value.f - new_value) <= FLT_EPSILON;
			break;
		}
	}

	if (setting_to_static_default) {
		runtime_defaults.reset(param);

		result = PX4_OK;

	} else {
		param_value_u new_value{};

		switch (param_type(param)) {
		case PARAM_TYPE_INT32: {
				memcpy(&new_value.i, val, sizeof(new_value.i));
				break;
			}

		case PARAM_TYPE_FLOAT: {
				memcpy(&new_value.f, val, sizeof(new_value.f));
				break;
			}

		default:
			break;
		}

		if (runtime_defaults.store(param, new_value)) {
			user_config.refresh(param);
			result = PX4_OK;

		} else {
			result = PX4_ERROR;
		}
	}


	if ((result == PX4_OK) && param_used(param)) {
		// send notification if param is already in use
		param_notify_changes();
	}

	return result;
}

static int param_reset_internal(param_t param, bool notify = true, bool autosave = true)
{
#if defined(CONFIG_PARAM_REMOTE)
	// Remote doesn't support reset
	return false;
#endif

	bool param_found = user_config.contains(param);

	if (handle_in_range(param)) {
		user_config.reset(param);
	}

	if (autosave) {
		param_autosave();
	}

	if (param_found && notify) {
		param_notify_changes();
	}

#if defined(CONFIG_PARAM_PRIMARY)
	param_primary_reset(param);
#endif

	return param_found;
}

int param_reset(param_t param) { return param_reset_internal(param, true); }
int param_reset_no_notification(param_t param) { return param_reset_internal(param, false); }

static void
param_reset_all_internal(bool auto_save)
{
#if defined(CONFIG_PARAM_REMOTE)
	// Remote doesn't support reset
	return;
#endif

	for (param_t param = 0; handle_in_range(param); param++) {
		param_reset_internal(param, false, false);
	}

	if (auto_save) {
		param_autosave();
	}

#if defined(CONFIG_PARAM_PRIMARY)
	param_primary_reset_all();
#endif

	param_notify_changes();
}

void
param_reset_all()
{
	param_reset_all_internal(true);
}

void
param_reset_excludes(const char *excludes[], int num_excludes)
{
	for (param_t param = 0; handle_in_range(param); param++) {
		const char *name = param_name(param);
		bool exclude = false;

		for (int index = 0; index < num_excludes; index ++) {
			int len = strlen(excludes[index]);

			if ((excludes[index][len - 1] == '*'
			     && strncmp(name, excludes[index], len - 1) == 0)
			    || strcmp(name, excludes[index]) == 0) {

				exclude = true;
				break;
			}
		}

		if (!exclude) {
			param_reset(param);
		}
	}
}

void
param_reset_specific(const char *resets[], int num_resets)
{
	for (param_t param = 0; handle_in_range(param); param++) {
		const char *name = param_name(param);
		bool reset = false;

		for (int index = 0; index < num_resets; index++) {
			int len = strlen(resets[index]);

			if ((resets[index][len - 1] == '*'
			     && strncmp(name, resets[index], len - 1) == 0)
			    || strcmp(name, resets[index]) == 0) {

				reset = true;
				break;
			}
		}

		if (reset) {
			param_reset(param);
		}
	}
}

int
param_set_default_file(const char *filename)
{
	if ((param_backup_file && strcmp(filename, param_backup_file) == 0)) {
		PX4_ERR("default file can't be the same as the backup file %s", filename);
		return PX4_ERROR;
	}

#ifdef FLASH_BASED_PARAMS
	// the default for flash-based params is always the FLASH
	(void)filename;
#else

	if (param_default_file != nullptr) {
		// we assume this is not in use by some other thread
		free(param_default_file);
		param_default_file = nullptr;
	}

	if (filename) {
		param_default_file = strdup(filename);
	}

#endif /* FLASH_BASED_PARAMS */

	return 0;
}

const char *param_get_default_file()
{
	return param_default_file;
}

int param_set_backup_file(const char *filename)
{
	if (param_default_file && strcmp(filename, param_default_file) == 0) {
		PX4_ERR("backup file can't be the same as the default file %s", filename);
		return PX4_ERROR;
	}

	if (param_backup_file != nullptr) {
		// we assume this is not in use by some other thread
		free(param_backup_file);
		param_backup_file = nullptr;
	}

	if (filename) {
		param_backup_file = strdup(filename);

	} else {
		param_backup_file = nullptr; // backup disabled
	}

	return 0;
}

const char *param_get_backup_file()
{
	return param_backup_file;
}

static int param_export_internal(int fd, param_filter_func filter);
static int param_verify(int fd);

int param_save_default(bool blocking)
{
	PX4_DEBUG("param_save_default");

	// take the file lock
	if (blocking) {
		pthread_mutex_lock(&file_mutex);

	} else {
		if (pthread_mutex_trylock(&file_mutex) != 0) {
			PX4_DEBUG("param_save_default: file lock failed (already locked)");
			return -EWOULDBLOCK;
		}
	}

	int shutdown_lock_ret = px4_shutdown_lock();

	if (shutdown_lock_ret != 0) {
		PX4_ERR("px4_shutdown_lock() failed (%i)", shutdown_lock_ret);
	}

	int res = PX4_ERROR;
	const char *filename = param_get_default_file();

	if (filename) {
		static constexpr int MAX_ATTEMPTS = 3;

		for (int attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
			// write parameters to file
			int fd = ::open(filename, O_WRONLY | O_CREAT | O_TRUNC, PX4_O_MODE_666);

			if (fd > -1) {
				perf_begin(param_export_perf);
				res = param_export_internal(fd, nullptr);
				perf_end(param_export_perf);
				::close(fd);

				if (res == PX4_OK) {
					// reopen file to verify
					int fd_verify = ::open(filename, O_RDONLY, PX4_O_MODE_666);
					res = param_verify(fd_verify) || lseek(fd_verify, 0, SEEK_SET) || param_verify(fd_verify);
					::close(fd_verify);
				}
			}

			if (res == PX4_OK) {
				break;

			} else {
				PX4_ERR("parameter export to %s failed (%d) attempt %d", filename, res, attempt);
				px4_usleep(10000); // wait at least 10 milliseconds before trying again
			}
		}

	} else {
		perf_begin(param_export_perf);
		res = flash_param_save(nullptr);
		perf_end(param_export_perf);
	}

	if (res != PX4_OK) {
		PX4_ERR("param export failed (%d)", res);

	} else {
		params_unsaved.reset();

		// backup file
		if (param_backup_file) {
			int fd_backup_file = ::open(param_backup_file, O_WRONLY | O_CREAT | O_TRUNC, PX4_O_MODE_666);

			if (fd_backup_file > -1) {
				int backup_export_ret = param_export_internal(fd_backup_file, nullptr);
				::close(fd_backup_file);

				if (backup_export_ret != 0) {
					PX4_ERR("backup parameter export to %s failed (%d)", param_backup_file, backup_export_ret);

				} else {
					// verify export
					int fd_verify = ::open(param_backup_file, O_RDONLY, PX4_O_MODE_666);
					param_verify(fd_verify);
					::close(fd_verify);
				}
			}
		}
	}

	pthread_mutex_unlock(&file_mutex);

	if (shutdown_lock_ret == 0) {
		px4_shutdown_unlock();
	}

	return res;
}

/**
 * @return 0 on success, 1 if all params have not yet been stored, -1 if device open failed, -2 if writing parameters failed
 */
int
param_load_default()
{
	int res = 0;
	const char *filename = param_get_default_file();

	if (!filename) {
		return flash_param_load();
	}

	int fd_load = ::open(filename, O_RDONLY);

	if (fd_load < 0) {
		/* no parameter file is OK, otherwise this is an error */
		if (errno != ENOENT) {
			PX4_ERR("open '%s' for reading failed", filename);
			return -1;
		}

		return 1;
	}

	int result = param_load(fd_load);
	::close(fd_load);

	if (result != 0) {
		PX4_ERR("error reading parameters from '%s'", filename);
		return -2;
	}

	return res;
}

static int param_verify_callback(bson_decoder_t decoder, bson_node_t node)
{
	if (node->type == BSON_EOO) {
		return 0;
	}

	// find the parameter this node represents
	param_t param = param_find_no_notification(node->name);

	if (param == PARAM_INVALID) {
		PX4_ERR("verify: invalid parameter '%s'", node->name);
		return -1;
	}

	// handle verifying the parameter from the node
	switch (node->type) {
	case BSON_INT32: {
			if (param_type(param) != PARAM_TYPE_INT32) {
				PX4_ERR("verify: invalid param type %d for '%s' (BSON_INT32)", param_type(param), node->name);
				return -1;
			}

			int32_t value;

			if (param_get(param, &value) == 0) {
				if (value == node->i32) {
					return 1; // valid

				} else {
					PX4_ERR("verify: '%s' invalid BSON value %" PRIi32 "(expected %" PRIi32 ")", node->name, node->i32, value);
				}
			}
		}
		break;

	case BSON_DOUBLE: {
			if (param_type(param) != PARAM_TYPE_FLOAT) {
				PX4_ERR("verify: invalid param type %d for '%s' (BSON_DOUBLE)", param_type(param), node->name);
				return -1;
			}

			float value;

			if (param_get(param, &value) == 0) {
				if (fabsf(value - (float)node->d) <= FLT_EPSILON) {
					return 1; // valid

				} else {
					PX4_ERR("verify: '%s' invalid BSON value %.3f (expected %.3f)", node->name, node->d, (double)value);
				}
			}
		}
		break;

	default:
		PX4_ERR("verify: '%s' invalid node type %d", node->name, node->type);
	}

	return -1;
}

static int param_verify(int fd)
{
	PX4_DEBUG("param_verify");

	if (fd < 0) {
		return -1;
	}

	if (lseek(fd, 0, SEEK_SET) != 0) {
		PX4_ERR("verify: seek failed");
		return -1;
	}

	bson_decoder_s decoder{};

	if (bson_decoder_init_file(&decoder, fd, param_verify_callback) == 0) {
		int result = -1;

		do {
			result = bson_decoder_next(&decoder);

		} while (result > 0);

		if (result == 0) {
			if (decoder.total_document_size != decoder.total_decoded_size) {
				PX4_ERR("BSON document size (%" PRId32 ") doesn't match bytes decoded (%" PRId32 ")", decoder.total_document_size,
					decoder.total_decoded_size);

			} else {
				return 0;
			}

		} else if (result == -ENODATA) {
			PX4_ERR("verify: no BSON data");

		} else {
			PX4_ERR("verify: failed (%d)", result);
		}
	}

	return -1;
}

int
param_export(const char *filename, param_filter_func filter)
{
	PX4_DEBUG("param_export");

	int shutdown_lock_ret = px4_shutdown_lock();

	if (shutdown_lock_ret != 0) {
		PX4_ERR("px4_shutdown_lock() failed (%i)", shutdown_lock_ret);
	}

	// take the file lock
	if (pthread_mutex_trylock(&file_mutex) != 0) {
		PX4_ERR("param_export: file lock failed (already locked)");
		return PX4_ERROR;
	}

	int fd = ::open(filename, O_RDWR | O_CREAT, PX4_O_MODE_666);
	int result = PX4_ERROR;

	perf_begin(param_export_perf);

	if (fd > -1) {
		result = param_export_internal(fd, filter);

	} else {
		result = flash_param_save(filter);
	}

	perf_end(param_export_perf);

	pthread_mutex_unlock(&file_mutex);

	if (shutdown_lock_ret == 0) {
		px4_shutdown_unlock();
	}

	return result;
}

// internal parameter export, caller is responsible for locking
static int param_export_internal(int fd, param_filter_func filter)
{
	PX4_DEBUG("param_export_internal");
	const auto changed_params = user_config.containedAsBitset();

	int result = -1;
	bson_encoder_s encoder{};
	uint8_t bson_buffer[256];

	if (lseek(fd, 0, SEEK_SET) != 0) {
		PX4_ERR("export seek failed %d", errno);
		return -1;
	}

	if (bson_encoder_init_buf_file(&encoder, fd, &bson_buffer, sizeof(bson_buffer)) != 0) {
		goto out;
	}

	for (param_t param = 0; handle_in_range(param); param++) {
		if (!changed_params[param] || (filter && !filter(param))) {
			continue;
		}

		const param_value_u runtime_default_value = runtime_defaults.get(param);
		const param_value_u user_config_value = user_config.get(param);

		// don't export default values
		switch (param_type(param)) {
		case PARAM_TYPE_INT32:
			if (user_config_value.i == runtime_default_value.i) {
				PX4_DEBUG("skipping %s %" PRIi32 " export", param_name(param), runtime_default_value.i);
				continue;
			}

			break;

		case PARAM_TYPE_FLOAT:
			if (fabsf(user_config_value.f - runtime_default_value.f) <= FLT_EPSILON) {
				PX4_DEBUG("skipping %s %.3f export", param_name(param), (double)runtime_default_value.f);
				continue;
			}

			break;
		}

		const char *name = param_name(param);
		const size_t size = param_size(param);

		/* append the appropriate BSON type object */
		switch (param_type(param)) {
		case PARAM_TYPE_INT32: {
				const int32_t i = user_config_value.i;
				PX4_DEBUG("exporting: %s (%d) size: %lu val: %" PRIi32, name, param, (long unsigned int)size, i);

				if (bson_encoder_append_int32(&encoder, name, i) != 0) {
					PX4_ERR("BSON append failed for '%s'", name);
					goto out;
				}
			}
			break;

		case PARAM_TYPE_FLOAT: {
				const double f = (double)user_config_value.f;
				PX4_DEBUG("exporting: %s (%d) size: %lu val: %.3f", name, param, (long unsigned int)size, (double)f);

				if (bson_encoder_append_double(&encoder, name, f) != 0) {
					PX4_ERR("BSON append failed for '%s'", name);
					goto out;
				}
			}
			break;

		default:
			PX4_ERR("%s unrecognized parameter type %d, skipping export", name, param_type(param));
		}
	}

	result = 0;

out:

	if (result == 0) {
		if (bson_encoder_fini(&encoder) != PX4_OK) {
			PX4_ERR("BSON encoder finalize failed");
			result = -1;
		}
	}

	return result;
}

static int
param_import_callback(bson_decoder_t decoder, bson_node_t node)
{
	/*
	 * EOO means the end of the parameter object. (Currently not supporting
	 * nested BSON objects).
	 */
	if (node->type == BSON_EOO) {
		PX4_DEBUG("end of parameters");
		return 0;
	}

	// if we do param_set() directly in the translation, set PARAM_SKIP_IMPORT as return value and return here
	if (param_modify_on_import(node) == param_modify_on_import_ret::PARAM_SKIP_IMPORT) {
		return 1;
	}

	// Find the parameter this node represents.  If we don't know it, ignore the node.
	param_t param = param_find_no_notification(node->name);

	if (param == PARAM_INVALID) {
		PX4_WARN("ignoring unrecognised parameter '%s'", node->name);
		return 1;
	}

	// Handle setting the parameter from the node
	switch (node->type) {
	case BSON_INT32: {
			if (param_type(param) == PARAM_TYPE_INT32) {
				int32_t i = node->i32;
				param_set_internal(param, &i, true, true);
				PX4_DEBUG("Imported %s with value %" PRIi32, param_name(param), i);

			} else {
				PX4_WARN("unexpected type for %s", node->name);
			}
		}
		break;

	case BSON_DOUBLE: {
			if (param_type(param) == PARAM_TYPE_FLOAT) {
				float f = node->d;
				param_set_internal(param, &f, true, true);
				PX4_DEBUG("Imported %s with value %f", param_name(param), (double)f);

			} else {
				PX4_WARN("unexpected type for %s", node->name);
			}
		}
		break;

	default:
		PX4_ERR("import: unrecognised node type for '%s'", node->name);
	}

	// don't return zero, that means EOF
	return 1;
}

static int
param_import_internal(int fd)
{
	static constexpr int MAX_ATTEMPTS = 3;

	for (int attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
		bson_decoder_s decoder{};

		if (bson_decoder_init_file(&decoder, fd, param_import_callback) == 0) {
			int result = -1;

			do {
				result = bson_decoder_next(&decoder);

			} while (result > 0);

			if (result == 0) {
				if (decoder.total_document_size == decoder.total_decoded_size) {
					PX4_INFO("BSON document size %" PRId32 " bytes, decoded %" PRId32 " bytes (INT32:%" PRIu16 ", FLOAT:%" PRIu16 ")",
						 decoder.total_document_size, decoder.total_decoded_size,
						 decoder.count_node_int32, decoder.count_node_double);

					return 0;

				} else {
					PX4_ERR("BSON document size (%" PRId32 ") doesn't match bytes decoded (%" PRId32 ")",
						decoder.total_document_size, decoder.total_decoded_size);
				}

			} else if (result == -ENODATA) {
				// silently retry as a precaution unless this is our last attempt
				if (attempt == MAX_ATTEMPTS) {
					PX4_DEBUG("BSON: no data");
					return 0;
				}

			} else {
				PX4_ERR("param import failed (%d) attempt %d", result, attempt);
			}

		} else {
			PX4_ERR("param import bson decoder init failed (attempt %d)", attempt);
		}

		if (attempt != MAX_ATTEMPTS) {
			if (lseek(fd, 0, SEEK_SET) != 0) {
				PX4_ERR("import lseek failed (%d)", errno);
			}

			px4_usleep(10000); // wait at least 10 milliseconds before trying again
		}
	}

	return -1;
}

int
param_import(int fd)
{
	if (fd < 0) {
		return flash_param_import();
	}

	return param_import_internal(fd);
}

int
param_load(int fd)
{
	if (fd < 0) {
		return flash_param_load();
	}

	param_reset_all_internal(false);
	return param_import_internal(fd);
}

void
param_foreach(void (*func)(void *arg, param_t param), void *arg, bool only_changed, bool only_used)
{
	param_t	param;

	for (param = 0; handle_in_range(param); param++) {

		/* if requested, skip unchanged values */
		if (only_changed && (!user_config.contains(param))) {
			continue;
		}

		if (only_used && !param_used(param)) {
			continue;
		}

		func(arg, param);
	}
}

uint32_t param_hash_check()
{
	uint32_t param_hash = 0;

	/* compute the CRC32 over all string param names and 4 byte values */
	for (param_t param = 0; handle_in_range(param); param++) {
		if (!param_used(param) || param_is_volatile(param)) {
			continue;
		}

		const char *name = param_name(param);
		auto value = user_config.get(param).i;
		const void *val = (void *)&value;
		param_hash = crc32part((const uint8_t *)name, strlen(name), param_hash);
		param_hash = crc32part((const uint8_t *)val, param_size(param), param_hash);
	}

	return param_hash;
}

void param_print_status()
{
	PX4_INFO("summary: %d/%d (used/total)", param_count_used(), param_count());

#ifndef FLASH_BASED_PARAMS
	const char *filename = param_get_default_file();

	if (filename != nullptr) {
		PX4_INFO("file: %s", param_get_default_file());
	}

	if (param_backup_file) {
		PX4_INFO("backup file: %s", param_backup_file);
	}

#endif /* FLASH_BASED_PARAMS */

	PX4_INFO("storage array: %d/%d elements (%zu bytes total)",
		 user_config.size(), firmware_defaults.size(), (size_t)user_config.byteSize());


	PX4_INFO("storage array (custom defaults): %d/%d elements (%zu bytes total)",
		 runtime_defaults.size(), firmware_defaults.size(), (size_t)runtime_defaults.byteSize());

	if (autosave_instance) {
		PX4_INFO("auto save: %s", autosave_instance->enabled() ? "on" : "off");

		hrt_abstime last_autosave = autosave_instance->lastAutosave();

		if (last_autosave > 0) {
			PX4_INFO("last auto save: %.3f seconds ago", hrt_elapsed_time(&last_autosave) * 1e-6);
		}
	}

	perf_print_counter(param_export_perf);
	perf_print_counter(param_find_perf);
	perf_print_counter(param_get_perf);
	perf_print_counter(param_set_perf);

#if defined(CONFIG_PARAM_PRIMARY)
	struct param_primary_counters counts;
	param_primary_get_counters(&counts);
	PX4_INFO("set value requests received: %" PRIu32 ", set value responses sent: %" PRIu32,
		 counts.set_value_request_received, counts.set_value_response_sent);
	PX4_INFO("set value requests sent: %" PRIu32 ", set value responses received: %" PRIu32,
		 counts.set_value_request_sent, counts.set_value_response_received);
	PX4_INFO("resets sent: %" PRIu32 ", set used requests received: %" PRIu32,
		 counts.reset_sent, counts.set_used_received);
#endif

#if defined(CONFIG_PARAM_REMOTE)
	struct param_remote_counters counts;
	param_remote_get_counters(&counts);
	PX4_INFO("set value requests received: %" PRIu32 ", set value responses sent: %" PRIu32,
		 counts.set_value_request_received, counts.set_value_response_sent);
	PX4_INFO("set value requests sent: %" PRIu32 ", set value responses received: %" PRIu32,
		 counts.set_value_request_sent, counts.set_value_response_received);
	PX4_INFO("resets received: %" PRIu32 ", set used requests sent: %" PRIu32,
		 counts.reset_received, counts.set_used_sent);
#endif

}
