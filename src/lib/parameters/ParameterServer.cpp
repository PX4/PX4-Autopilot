/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#define PARAM_IMPLEMENTATION

#include "ParameterServer.hpp"

#include "param_translation.h"

#include <crc32.h>
#include <float.h>
#include <math.h>

// #if defined(FLASH_BASED_PARAMS)
// #include "flashparams/flashparams.h"
// #else
// inline static int flash_param_save(param_filter_func filter) { return -1; }
// inline static int flash_param_load() { return -1; }
// inline static int flash_param_import() { return -1; }
// #endif

ParameterServer::ParameterServer() :
	ScheduledWorkItem("parameter_server", px4::wq_configurations::hp_default)
{
	px4_sem_init(&_param_sem, 0, 1);
	px4_sem_init(&_param_sem_save, 0, 1);
	px4_sem_init(&_reader_lock_holders_lock, 0, 1);

	_param_request_sub.registerCallback();
}

ParameterServer::~ParameterServer()
{
	px4_sem_destroy(&_param_sem);
	px4_sem_destroy(&_param_sem_save);
	px4_sem_destroy(&_reader_lock_holders_lock);

	perf_free(_export_perf);
	perf_free(_find_count_perf);
	perf_free(_get_count_perf);
	perf_free(_set_perf);
}

void ParameterServer::lockReader()
{
	do {} while (px4_sem_wait(&_reader_lock_holders_lock) != 0);

	++_reader_lock_holders;

	if (_reader_lock_holders == 1) {
		// the first reader takes the lock, the next ones are allowed to just continue
		do {} while (px4_sem_wait(&_param_sem) != 0);
	}

	px4_sem_post(&_reader_lock_holders_lock);
}

void ParameterServer::lockWriter()
{
	do {} while (px4_sem_wait(&_param_sem) != 0);
}

void ParameterServer::unlockReader()
{
	do {} while (px4_sem_wait(&_reader_lock_holders_lock) != 0);

	--_reader_lock_holders;

	if (_reader_lock_holders == 0) {
		// the last reader releases the lock
		px4_sem_post(&_param_sem);
	}

	px4_sem_post(&_reader_lock_holders_lock);
}

void ParameterServer::unlockWriter()
{
	px4_sem_post(&_param_sem);
}

int ParameterServer::compareValues(const void *a, const void *b)
{
	struct param_wbuf_s *pa = (struct param_wbuf_s *)a;
	struct param_wbuf_s *pb = (struct param_wbuf_s *)b;

	if (pa->param < pb->param) {
		return -1;
	}

	if (pa->param > pb->param) {
		return 1;
	}

	return 0;
}

ParameterServer::param_wbuf_s *ParameterServer::findChanged(param_t param)
{
	if (_params_changed[param] && (_param_values != nullptr)) {
		param_wbuf_s key{};
		key.param = param;
		return (param_wbuf_s *)utarray_find(_param_values, &key, ParameterServer::compareValues);
	}

	return nullptr;
}

const void *ParameterServer::getParameterValuePointer(param_t param)
{
	if (handle_in_range(param)) {
		/* work out whether we're fetching the default or a written value */
		struct param_wbuf_s *s = findChanged(param);

		if (s != nullptr) {
			return &s->val;

		} else {
			if (_params_custom_default[param] && _param_custom_default_values) {
				// get default from custom default storage
				param_wbuf_s key{};
				key.param = param;
				param_wbuf_s *pbuf = (param_wbuf_s *)utarray_find(_param_custom_default_values, &key, ParameterServer::compareValues);

				if (pbuf != nullptr) {
					return &pbuf->val;
				}
			}

			// otherwise return static default value
			switch (getParameterType(param)) {
			case PARAM_TYPE_INT32:
				return &px4::parameters[param].val.i;

			case PARAM_TYPE_FLOAT:
				return &px4::parameters[param].val.f;
			}
		}
	}

	return nullptr;
}

int ParameterServer::getParameterDefaultValueInternal(param_t param, void *default_val)
{
	if (!handle_in_range(param)) {
		PX4_ERR("get default value: param %d invalid", param);
		return PX4_ERROR;
	}

	if (default_val) {
		if (_params_custom_default[param] && _param_custom_default_values) {
			// get default from custom default storage
			param_wbuf_s key{};
			key.param = param;
			param_wbuf_s *pbuf = (param_wbuf_s *)utarray_find(_param_custom_default_values, &key, ParameterServer::compareValues);

			if (pbuf != nullptr) {
				memcpy(default_val, &pbuf->val, getParameterSize(param));
				return PX4_OK;
			}
		}

		// otherwise return static default value
		switch (getParameterType(param)) {
		case PARAM_TYPE_INT32:
			memcpy(default_val, &px4::parameters[param].val.i, getParameterSize(param));
			return PX4_OK;

		case PARAM_TYPE_FLOAT:
			memcpy(default_val, &px4::parameters[param].val.f, getParameterSize(param));
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

param_t ParameterServer::findParameter(const char *name, bool notification)
{
	perf_count(_find_count_perf);

	param_t middle;
	param_t front = 0;
	param_t last = param_info_count;

	// perform a binary search of the known parameters
	while (front <= last) {
		middle = front + (last - front) / 2;
		int ret = strcmp(name, getParameterName(middle));

		if (ret == 0) {
			if (notification) {
				setParameterUsed(middle);
			}

			return middle;

		} else if (middle == front) {
			// An end point has been hit, but there has been no match
			break;

		} else if (ret < 0) {
			last = middle;

		} else {
			front = middle;
		}
	}

	// not found
	return PARAM_INVALID;
}

bool ParameterServer::isParameterUsed(param_t param) const
{
	if (handle_in_range(param)) {
		return _params_active[param];
	}

	return false;
}

param_t ParameterServer::forIndex(unsigned index) const
{
	if (index < param_info_count) {
		return (param_t)index;
	}

	return PARAM_INVALID;
}

param_t ParameterServer::forUsedIndex(unsigned index) const
{
	// walk all params and count used params
	if (index < param_info_count) {
		unsigned used_count = 0;

		for (int i = 0; i < _params_active.size(); i++) {
			if (_params_active[i]) {
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

int ParameterServer::getParameterIndex(param_t param) const
{
	if (handle_in_range(param)) {
		return (unsigned)param;
	}

	return -1;
}

int ParameterServer::getParameterUsedIndex(param_t param) const
{
	// this tests for out of bounds and does a constant time lookup
	if (!isParameterUsed(param)) {
		return -1;
	}

	// walk all params and count, now knowing that it has a valid index
	int used_count = 0;

	for (int i = 0; i < _params_active.size(); i++) {
		if (_params_active[i]) {

			if (param == i) {
				return used_count;
			}

			used_count++;
		}
	}

	return -1;
}

const char *ParameterServer::getParameterName(param_t param) const
{
	if (handle_in_range(param)) {
		return px4::parameters[param].name;
	}

	return nullptr;
}

bool ParameterServer::isParameterVolatile(param_t param) const
{
	if (handle_in_range(param)) {
		for (const auto &p : px4::parameters_volatile) {
			if (static_cast<px4::params>(param) == p) {
				return true;
			}
		}
	}

	return false;
}

bool ParameterServer::isParameterValueDefault(param_t param)
{
	if (!handle_in_range(param)) {
		return true;
	}

	if (!_params_changed[param] && !_params_custom_default[param]) {
		// no value saved and no custom default
		return true;

	} else {
		// the _param_values dynamic array might carry things that have been set
		// back to default, so we don't rely on the _params_changed bitset here
		switch (getParameterType(param)) {
		case PARAM_TYPE_INT32: {
				lockReader();
				int32_t default_value = 0;

				if (getParameterDefaultValueInternal(param, &default_value) == PX4_OK) {
					const void *v = getParameterValuePointer(param);

					if (v) {
						bool unchanged = (*static_cast<const int32_t *>(v) == default_value);
						unlockReader();
						return unchanged;
					}
				}

				unlockReader();
			}
			break;

		case PARAM_TYPE_FLOAT: {
				lockReader();
				float default_value = 0;

				if (getParameterDefaultValueInternal(param, &default_value) == PX4_OK) {
					const void *v = getParameterValuePointer(param);

					if (v) {
						bool unchanged = (fabsf(*static_cast<const float *>(v) - default_value) <= FLT_EPSILON);
						unlockReader();
						return unchanged;
					}
				}

				unlockReader();
			}
			break;
		}
	}

	return true;
}

bool ParameterServer::isParameterValueUnsaved(param_t param)
{
	return handle_in_range(param) ? _params_unsaved[param] : false;
}

param_type_t ParameterServer::getParameterType(param_t param) const
{
	return handle_in_range(param) ? px4::parameters_type[param] : PARAM_TYPE_UNKNOWN;
}

size_t ParameterServer::getParameterSize(param_t param) const
{
	if (handle_in_range(param)) {
		switch (getParameterType(param)) {
		case PARAM_TYPE_INT32:
		case PARAM_TYPE_FLOAT:
			return 4;

		default:
			return 0;
		}
	}

	return 0;
}

int ParameterServer::getParameterValue(param_t param, void *val)
{
	perf_count(_get_count_perf);

	if (!handle_in_range(param)) {
		PX4_ERR("get: param %" PRId16 " invalid", param);
		return PX4_ERROR;
	}

	if (!_params_active[param]) {
		PX4_DEBUG("get: param %" PRId16 " (%s) not active", param, getParameterName(param));
	}

	int result = PX4_ERROR;

	if (val) {
		if (!_params_changed[param] && !_params_custom_default[param]) {
			// if parameter is unchanged (static default value) copy immediately and avoid locking
			switch (getParameterType(param)) {
			case PARAM_TYPE_INT32:
				memcpy(val, &px4::parameters[param].val.i, sizeof(px4::parameters[param].val.i));
				return PX4_OK;

			case PARAM_TYPE_FLOAT:
				memcpy(val, &px4::parameters[param].val.f, sizeof(px4::parameters[param].val.f));
				return PX4_OK;
			}
		}

		lockReader();
		const void *v = getParameterValuePointer(param);

		if (v) {
			memcpy(val, v, getParameterSize(param));
			result = PX4_OK;
		}

		unlockReader();
	}

	return result;
}

int ParameterServer::getParameterDefaultValue(param_t param, void *default_val)
{
	if (!handle_in_range(param)) {
		return PX4_ERROR;
	}

	int ret = 0;

	if (!_params_custom_default[param]) {
		// return static default value
		switch (getParameterType(param)) {
		case PARAM_TYPE_INT32:
			memcpy(default_val, &px4::parameters[param].val.i, sizeof(px4::parameters[param].val.i));
			return PX4_OK;

		case PARAM_TYPE_FLOAT:
			memcpy(default_val, &px4::parameters[param].val.f, sizeof(px4::parameters[param].val.f));
			return PX4_OK;
		}

	} else {
		lockReader();
		ret = getParameterDefaultValueInternal(param, default_val);
		unlockReader();
	}

	return ret;
}

int ParameterServer::getParameterSystemDefaultValue(param_t param, void *default_val)
{
	if (!handle_in_range(param)) {
		return PX4_ERROR;
	}

	int ret = PX4_OK;

	switch (getParameterType(param)) {
	case PARAM_TYPE_INT32:
		memcpy(default_val, &px4::parameters[param].val.i, getParameterSize(param));
		break;

	case PARAM_TYPE_FLOAT:
		memcpy(default_val, &px4::parameters[param].val.f, getParameterSize(param));
		break;

	default:
		ret = PX4_ERROR;
		break;
	}

	return ret;
}

int ParameterServer::setParameter(param_t param, const void *val, bool mark_saved, bool notify_changes)
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

	lockWriter();
	perf_begin(_set_perf);

	// create the parameter store if it doesn't exist
	if (_param_values == nullptr) {
		utarray_new(_param_values, &param_icd);

		// mark all parameters unchanged (default)
		_params_changed.reset();
		_params_unsaved.reset();
	}

	if (_param_values == nullptr) {
		PX4_ERR("failed to allocate modified values array");
		goto out;

	} else {
		param_wbuf_s *s = findChanged(param);

		if (s == nullptr) {
			/* construct a new parameter */
			param_wbuf_s buf{};
			buf.param = param;

			param_changed = true;

			/* add it to the array and sort */
			utarray_push_back(_param_values, &buf);
			utarray_sort(_param_values, ParameterServer::compareValues);
			_params_changed.set(param, true);

			/* find it after sorting */
			s = findChanged(param);
		}

		if (s == nullptr) {
			PX4_ERR("error _param_values storage slot invalid");

		} else {
			/* update the changed value */
			switch (getParameterType(param)) {
			case PARAM_TYPE_INT32:
				if (s->val.i != *(int32_t *)val) {
					s->val.i = *(int32_t *)val;
					param_changed = true;
				}

				_params_changed.set(param, true);
				_params_unsaved.set(param, !mark_saved);
				result = PX4_OK;
				break;

			case PARAM_TYPE_FLOAT:
				if (fabsf(s->val.f - * (float *)val) > FLT_EPSILON) {
					s->val.f = *(float *)val;
					param_changed = true;
				}

				_params_changed.set(param, true);
				_params_unsaved.set(param, !mark_saved);
				result = PX4_OK;
				break;

			default:
				PX4_ERR("param_set invalid param type for %s", getParameterName(param));
				break;
			}
		}

		if ((result == PX4_OK) && param_changed && !mark_saved) { // this is false when importing parameters
			autoSave();
		}
	}

out:
	perf_end(_set_perf);
	unlockWriter();

	/*
	 * If we set something, now that we have unlocked, go ahead and advertise that
	 * a thing has been set.
	 */
	if ((result == PX4_OK) && param_changed && notify_changes) {
		notifyChanges();
	}

	return result;
}

int ParameterServer::setParameterDefaultValue(param_t param, const void *val)
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

	lockWriter();

	if (_param_custom_default_values == nullptr) {
		utarray_new(_param_custom_default_values, &param_icd);

		// mark all parameters unchanged (default)
		_params_custom_default.reset();

		if (_param_custom_default_values == nullptr) {
			PX4_ERR("failed to allocate custom default values array");
			unlockWriter();
			return PX4_ERROR;
		}
	}

	// check if param being set to default value
	bool setting_to_static_default = false;

	switch (getParameterType(param)) {
	case PARAM_TYPE_INT32:
		setting_to_static_default = (px4::parameters[param].val.i == *(int32_t *)val);
		break;

	case PARAM_TYPE_FLOAT:
		setting_to_static_default = (fabsf(px4::parameters[param].val.f - * (float *)val) <= FLT_EPSILON);
		break;
	}

	// find if custom default value is already set
	param_wbuf_s *s = nullptr;

	{
		param_wbuf_s key{};
		key.param = param;
		s = (param_wbuf_s *)utarray_find(_param_custom_default_values, &key, ParameterServer::compareValues);
	}

	if (setting_to_static_default) {
		if (s != nullptr) {
			// param in memory and set to non-default value, clear
			int pos = utarray_eltidx(_param_custom_default_values, s);
			utarray_erase(_param_custom_default_values, pos, 1);
		}

		// do nothing if param not already set and being set to default
		_params_custom_default.set(param, false);
		result = PX4_OK;

	} else {
		if (s == nullptr) {
			// construct a new parameter default value
			param_wbuf_s buf{};
			buf.param = param;

			// add it to the array and sort
			utarray_push_back(_param_custom_default_values, &buf);
			utarray_sort(_param_custom_default_values, ParameterServer::compareValues);

			// find it after sorting
			s = (param_wbuf_s *)utarray_find(_param_custom_default_values, &buf, ParameterServer::compareValues);
		}

		if (s != nullptr) {
			// update the default value
			switch (getParameterType(param)) {
			case PARAM_TYPE_INT32:
				s->val.i = *(int32_t *)val;
				_params_custom_default.set(param, true);
				result = PX4_OK;
				break;

			case PARAM_TYPE_FLOAT:
				s->val.f = *(float *)val;
				_params_custom_default.set(param, true);
				result = PX4_OK;
				break;

			default:
				break;
			}
		}
	}

	unlockWriter();

	if ((result == PX4_OK) && isParameterUsed(param)) {
		// send notification if param is already in use
		notifyChanges();
	}

	return result;
}

void ParameterServer::setParameterUsed(param_t param)
{
	if (handle_in_range(param)) {
		_params_active.set(param, true);
	}
}

void ParameterServer::notifyChanges()
{
	_notify_scheduled.store(true);
	ScheduleDelayed(10_ms);
	// TODO: delay back to back notifications
}

int ParameterServer::resetParameter(param_t param, bool notify)
{
	param_wbuf_s *s = nullptr;
	bool param_found = false;

	lockWriter();

	if (handle_in_range(param)) {
		/* look for a saved value */
		s = findChanged(param);

		/* if we found one, erase it */
		if (s != nullptr) {
			int pos = utarray_eltidx(_param_values, s);
			utarray_erase(_param_values, pos, 1);
		}

		_params_changed.set(param, false);
		_params_unsaved.set(param, true);

		param_found = true;
	}

	autoSave();

	unlockWriter();

	if (s != nullptr && notify) {
		notifyChanges();
	}

	return (!param_found);
}

void ParameterServer::resetAllParameters(bool auto_save)
{
	lockWriter();

	if (_param_values != nullptr) {
		utarray_free(_param_values);

		_params_changed.reset();
	}

	/* mark as reset / deleted */
	_param_values = nullptr;

	if (auto_save) {
		autoSave();
	}

	unlockWriter();

	notifyChanges();
}

void ParameterServer::resetExcludes(const char *excludes[], int num_excludes)
{
	for (param_t param = 0; handle_in_range(param); param++) {
		const char *name = getParameterName(param);
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
			resetParameter(param);
		}
	}
}

void ParameterServer::resetSpecificParameter(const char *resets[], int num_resets)
{
	for (param_t param = 0; handle_in_range(param); param++) {
		const char *name = getParameterName(param);
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
			resetParameter(param);
		}
	}
}

int ParameterServer::exportToFile(const char *filename, param_filter_func filter)
{
	PX4_DEBUG("param_export");

	int shutdown_lock_ret = px4_shutdown_lock();

	if (shutdown_lock_ret != 0) {
		PX4_ERR("px4_shutdown_lock() failed (%i)", shutdown_lock_ret);
	}

	// take the file lock
	do {} while (px4_sem_wait(&_param_sem_save) != 0);

	lockReader();

	int fd = ::open(filename, O_RDWR | O_CREAT, PX4_O_MODE_666);
	int result = PX4_ERROR;

	perf_begin(_export_perf);

	if (fd > -1) {
		result = exportInternal(fd, filter);

	} else {
		// TODO: flash params
		// result = flash_param_save(filter);
	}

	perf_end(_export_perf);

	unlockReader();
	px4_sem_post(&_param_sem_save);

	if (shutdown_lock_ret == 0) {
		px4_shutdown_unlock();
	}

	return result;
}

int ParameterServer::bsonImportCallback(bson_decoder_t decoder, bson_node_t node)
{
	/*
	 * EOO means the end of the parameter object. (Currently not supporting
	 * nested BSON objects).
	 */
	if (node->type == BSON_EOO) {
		PX4_DEBUG("end of parameters");
		return 0;
	}

	param_modify_on_import(node);

	// Find the parameter this node represents.  If we don't know it, ignore the node.
	param_t param = findParameter(node->name, false);

	if (param == PARAM_INVALID) {
		PX4_WARN("ignoring unrecognised parameter '%s'", node->name);
		return 1;
	}

	// Handle setting the parameter from the node
	switch (node->type) {
	case BSON_INT32: {
			if (getParameterType(param) == PARAM_TYPE_INT32) {
				int32_t i = node->i32;
				setParameter(param, &i, true, true);
				PX4_DEBUG("Imported %s with value %" PRIi32, param_name(param), i);

			} else {
				PX4_WARN("unexpected type for %s", node->name);
			}
		}
		break;

	case BSON_DOUBLE: {
			if (getParameterType(param) == PARAM_TYPE_FLOAT) {
				float f = node->d;
				setParameter(param, &f, true, true);
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

int ParameterServer::importCallbackTrampoline(bson_decoder_t decoder, void *priv, bson_node_t node)
{
	if (priv) {
		return static_cast<ParameterServer *>(priv)->bsonImportCallback(decoder, node);
	}

	return -1;
}

int ParameterServer::importFromFileDescriptorInternal(int fd)
{
	static constexpr int MAX_ATTEMPTS = 3;

	for (int attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
		bson_decoder_s decoder{};

		if (bson_decoder_init_file(&decoder, fd, importCallbackTrampoline, this) == 0) {
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

int ParameterServer::importFromFileDescriptor(int fd)
{
	// TODO: flash params
	// if (fd < 0) {
	// 	return flash_param_import();
	// }

	return importFromFileDescriptorInternal(fd);
}

int ParameterServer::loadFromFileDescriptor(int fd)
{
	// TODO: flash params
	// if (fd < 0) {
	// 	return flash_param_load();
	// }

	resetAllParameters(false);

	return importFromFileDescriptorInternal(fd);
}

int ParameterServer::verifyBsonExportCallback(bson_decoder_t decoder, bson_node_t node)
{
	if (node->type == BSON_EOO) {
		return 0;
	}

	// find the parameter this node represents
	param_t param = findParameter(node->name, false);

	if (param == PARAM_INVALID) {
		PX4_ERR("verify: invalid parameter '%s'", node->name);
		return -1;
	}

	// handle verifying the parameter from the node
	switch (node->type) {
	case BSON_INT32: {
			if (getParameterType(param) != PARAM_TYPE_INT32) {
				PX4_ERR("verify: invalid param type %d for '%s' (BSON_INT32)", getParameterType(param), node->name);
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
			if (getParameterType(param) != PARAM_TYPE_FLOAT) {
				PX4_ERR("verify: invalid param type %d for '%s' (BSON_DOUBLE)", getParameterType(param), node->name);
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

int ParameterServer::verifyBsonExportTrampoline(bson_decoder_t decoder, void *priv, bson_node_t node)
{
	if (priv) {
		return static_cast<ParameterServer *>(priv)->verifyBsonExportCallback(decoder, node);
	}

	return -1;
}

int ParameterServer::verifyBsonExport(int fd)
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

	if (bson_decoder_init_file(&decoder, fd, verifyBsonExportTrampoline, this) == 0) {
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

int ParameterServer::bsonDumpCallback(bson_decoder_t decoder, bson_node_t node)
{
	switch (node->type) {
	case BSON_EOO:
		PX4_INFO_RAW("BSON_EOO\n");
		return 0;

	case BSON_DOUBLE:
		PX4_INFO_RAW("BSON_DOUBLE: %s = %.6f\n", node->name, node->d);
		return 1;

	case BSON_BOOL:
		PX4_INFO_RAW("BSON_BOOL:   %s = %d\n", node->name, node->b);
		return 1;

	case BSON_INT32:
		PX4_INFO_RAW("BSON_INT32:  %s = %" PRIi32 "\n", node->name, node->i32);
		return 1;

	case BSON_INT64:
		PX4_INFO_RAW("BSON_INT64:  %s = %" PRIi64 "\n", node->name, node->i64);
		return 1;

	default:
		PX4_INFO_RAW("ERROR %s unhandled bson type %d\n", node->name, node->type);
		return 1; // just skip this entry
	}

	return -1;
}

int ParameterServer::bsonDumpCallbackTrampoline(bson_decoder_t decoder, void *priv, bson_node_t node)
{
	if (priv) {
		return static_cast<ParameterServer *>(priv)->bsonDumpCallback(decoder, node);
	}

	return -1;
}

int ParameterServer::bsonDump(int fd)
{
	bson_decoder_s decoder{};

	if (bson_decoder_init_file(&decoder, fd, bsonDumpCallbackTrampoline, this) == 0) {
		PX4_INFO_RAW("BSON document size %" PRId32 "\n", decoder.total_document_size);

		int result = -1;

		do {
			result = bson_decoder_next(&decoder);

		} while (result > 0);

		if (result == 0) {
			PX4_INFO_RAW("BSON decoded %" PRId32 " bytes (double:%" PRIu16 ", string:%" PRIu16 ", bin:%" PRIu16 ", bool:%" PRIu16
				     ", int32:%" PRIu16 ", int64:%" PRIu16 ")\n",
				     decoder.total_decoded_size,
				     decoder.count_node_double, decoder.count_node_string, decoder.count_node_bindata, decoder.count_node_bool,
				     decoder.count_node_int32, decoder.count_node_int64);

			return 0;

		} else if (result == -ENODATA) {
			PX4_WARN("BSON: no data");
			return 0;

		} else {
			PX4_ERR("param dump failed (%d)", result);
		}
	}

	return -1;
}

void ParameterServer::forEachParameter(void (*func)(void *arg, param_t param), void *arg, bool only_changed,
				       bool only_used)
{
	param_t	param;

	for (param = 0; handle_in_range(param); param++) {

		/* if requested, skip unchanged values */
		if (only_changed && (findChanged(param) == nullptr)) {
			continue;
		}

		if (only_used && !isParameterUsed(param)) {
			continue;
		}

		func(arg, param);
	}
}

int ParameterServer::setDefaultFile(const char *filename)
{
	if ((_param_backup_file && strcmp(filename, _param_backup_file) == 0)) {
		PX4_ERR("default file can't be the same as the backup file %s", filename);
		return PX4_ERROR;
	}

#ifdef FLASH_BASED_PARAMS
	// the default for flash-based params is always the FLASH
	(void)filename;
#else

	if (_param_default_file != nullptr) {
		// we assume this is not in use by some other thread
		free(_param_default_file);
		_param_default_file = nullptr;
	}

	if (filename) {
		_param_default_file = strdup(filename);
	}

#endif /* FLASH_BASED_PARAMS */

	return 0;
}

int ParameterServer::setBackupFile(const char *filename)
{
	if (_param_default_file && strcmp(filename, _param_default_file) == 0) {
		PX4_ERR("backup file can't be the same as the default file %s", filename);
		return PX4_ERROR;
	}

	if (_param_backup_file != nullptr) {
		// we assume this is not in use by some other thread
		free(_param_backup_file);
		_param_backup_file = nullptr;
	}

	if (filename) {
		_param_backup_file = strdup(filename);

	} else {
		_param_backup_file = nullptr; // backup disabled
	}

	return 0;
}

int ParameterServer::saveDefault()
{
	PX4_DEBUG("param_save_default");
	int shutdown_lock_ret = px4_shutdown_lock();

	if (shutdown_lock_ret != 0) {
		PX4_ERR("px4_shutdown_lock() failed (%i)", shutdown_lock_ret);
	}

	// take the file lock
	do {} while (px4_sem_wait(&_param_sem_save) != 0);

	lockReader();

	int res = PX4_ERROR;
	const char *filename = getDefaultFile();

	if (filename) {
		static constexpr int MAX_ATTEMPTS = 3;

		for (int attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
			// write parameters to file
			int fd = ::open(filename, O_WRONLY | O_CREAT | O_TRUNC, PX4_O_MODE_666);

			if (fd > -1) {
				perf_begin(_export_perf);
				res = exportInternal(fd, nullptr);
				perf_end(_export_perf);
				::close(fd);

				if (res == PX4_OK) {
					// reopen file to verify
					int fd_verify = ::open(filename, O_RDONLY, PX4_O_MODE_666);
					res = verifyBsonExport(fd_verify) || lseek(fd_verify, 0, SEEK_SET) || verifyBsonExport(fd_verify);
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
		// TODO: flash params
		// perf_begin(_export_perf);
		// res = flash_param_save(nullptr);
		// perf_end(_export_perf);
	}

	if (res != PX4_OK) {
		PX4_ERR("param export failed (%d)", res);

	} else {
		_params_unsaved.reset();

		// backup file
		if (_param_backup_file) {
			int fd_backup_file = ::open(_param_backup_file, O_WRONLY | O_CREAT | O_TRUNC, PX4_O_MODE_666);

			if (fd_backup_file > -1) {
				int backup_export_ret = exportInternal(fd_backup_file, nullptr);
				::close(fd_backup_file);

				if (backup_export_ret != 0) {
					PX4_ERR("backup parameter export to %s failed (%d)", _param_backup_file, backup_export_ret);

				} else {
					// verify export
					int fd_verify = ::open(_param_backup_file, O_RDONLY, PX4_O_MODE_666);
					verifyBsonExport(fd_verify);
					::close(fd_verify);
				}
			}
		}
	}

	unlockReader();
	px4_sem_post(&_param_sem_save);

	if (shutdown_lock_ret == 0) {
		px4_shutdown_unlock();
	}

	return res;
}

int ParameterServer::loadDefault()
{
	int res = 0;
	const char *filename = getDefaultFile();

	// TODO: flash params
	// if (!filename) {
	// 	return flash_param_load();
	// }

	int fd_load = ::open(filename, O_RDONLY);

	if (fd_load < 0) {
		/* no parameter file is OK, otherwise this is an error */
		if (errno != ENOENT) {
			PX4_ERR("open '%s' for reading failed", filename);
			return -1;
		}

		return 1;
	}

	int result = loadFromFileDescriptor(fd_load);
	::close(fd_load);

	if (result != 0) {
		PX4_ERR("error reading parameters from '%s'", filename);
		return -2;
	}

	return res;
}

uint32_t ParameterServer::hashCheck()
{
	uint32_t param_hash = 0;

	lockReader();

	/* compute the CRC32 over all string param names and 4 byte values */
	for (param_t param = 0; handle_in_range(param); param++) {
		if (!isParameterUsed(param) || isParameterVolatile(param)) {
			continue;
		}

		const char *name = getParameterName(param);
		const void *val = getParameterValuePointer(param);
		param_hash = crc32part((const uint8_t *)name, strlen(name), param_hash);
		param_hash = crc32part((const uint8_t *)val, getParameterSize(param), param_hash);
	}

	unlockReader();

	return param_hash;
}

void ParameterServer::printStatus()
{
	PX4_INFO("summary: %d/%d (used/total)", param_count_used(), param_count());

#ifndef FLASH_BASED_PARAMS
	const char *filename = getDefaultFile();

	if (filename != nullptr) {
		PX4_INFO("file: %s", filename);
	}

	if (_param_backup_file) {
		PX4_INFO("backup file: %s", _param_backup_file);
	}

#endif /* FLASH_BASED_PARAMS */

	if (_param_values != nullptr) {
		PX4_INFO("storage array: %d/%d elements (%zu bytes total)",
			 utarray_len(_param_values), _param_values->n, _param_values->n * sizeof(UT_icd));
	}

	if (_param_custom_default_values != nullptr) {
		PX4_INFO("storage array (custom defaults): %d/%d elements (%zu bytes total)",
			 utarray_len(_param_custom_default_values), _param_custom_default_values->n,
			 _param_custom_default_values->n * sizeof(UT_icd));
	}

	PX4_INFO("auto save: %s", _autosave_disabled ? "off" : "on");

	if (!_autosave_disabled && (_last_autosave_timestamp > 0)) {
		PX4_INFO("last auto save: %.3f seconds ago", hrt_elapsed_time(&_last_autosave_timestamp) * 1e-6);
	}

	perf_print_counter(_export_perf);
	perf_print_counter(_find_count_perf);
	perf_print_counter(_get_count_perf);
	perf_print_counter(_set_perf);
}

void ParameterServer::controlAutosave(bool enable)
{
	lockWriter();

	if (!enable && _autosave_scheduled.load()) {
		_autosave_scheduled.store(false);
	}

	_autosave_disabled = !enable;
	unlockWriter();
}

void ParameterServer::autoSave(bool now)
{
	if (now) {
		_autosave_scheduled.store(true);
		ScheduleNow();
		return;
	}

	if (_autosave_scheduled.load() || _autosave_disabled) {
		return;
	}

	// wait at least 300ms before saving, because:
	// - tasks often call param_set() for multiple params, so this avoids unnecessary save calls
	// - the logger stores changed params. He gets notified on a param change via uORB and then
	//   looks at all unsaved params.
	hrt_abstime delay = 300_ms;

	static constexpr const hrt_abstime rate_limit = 2_s; // rate-limit saving to 2 seconds
	const hrt_abstime last_save_elapsed = hrt_elapsed_time(&_last_autosave_timestamp);

	if (last_save_elapsed < rate_limit && rate_limit > last_save_elapsed + delay) {
		delay = rate_limit - last_save_elapsed;
	}

	_autosave_scheduled.store(true);

	ScheduleDelayed(delay);
}

int ParameterServer::exportInternal(int fd, param_filter_func filter)
{
	PX4_DEBUG("param_export_internal");

	int result = -1;
	param_wbuf_s *s = nullptr;
	bson_encoder_s encoder{};
	uint8_t bson_buffer[256];

	if (lseek(fd, 0, SEEK_SET) != 0) {
		PX4_ERR("export seek failed %d", errno);
		return -1;
	}

	if (bson_encoder_init_buf_file(&encoder, fd, &bson_buffer, sizeof(bson_buffer)) != 0) {
		goto out;
	}

	// no modified parameters, export empty BSON document
	if (_param_values == nullptr) {
		result = 0;
		goto out;
	}

	while ((s = (struct param_wbuf_s *)utarray_next(_param_values, s)) != nullptr) {
		if (filter && !filter(s->param)) {
			continue;
		}

		// don't export default values
		switch (getParameterType(s->param)) {
		case PARAM_TYPE_INT32: {
				int32_t default_value = 0;
				getParameterDefaultValueInternal(s->param, &default_value);

				if (s->val.i == default_value) {
					PX4_DEBUG("skipping %s %" PRIi32 " export", getParameterName(s->param), default_value);
					continue;
				}
			}
			break;

		case PARAM_TYPE_FLOAT: {
				float default_value = 0;
				getParameterDefaultValueInternal(s->param, &default_value);

				if (fabsf(s->val.f - default_value) <= FLT_EPSILON) {
					PX4_DEBUG("skipping %s %.3f export", getParameterName(s->param), (double)default_value);
					continue;
				}
			}
			break;
		}

		const char *name = getParameterName(s->param);
		const size_t size = getParameterSize(s->param);

		/* append the appropriate BSON type object */
		switch (getParameterType(s->param)) {
		case PARAM_TYPE_INT32: {
				const int32_t i = s->val.i;
				PX4_DEBUG("exporting: %s (%d) size: %lu val: %" PRIi32, name, s->param, (long unsigned int)size, i);

				if (bson_encoder_append_int32(&encoder, name, i) != 0) {
					PX4_ERR("BSON append failed for '%s'", name);
					goto out;
				}
			}
			break;

		case PARAM_TYPE_FLOAT: {
				const double f = (double)s->val.f;
				PX4_DEBUG("exporting: %s (%d) size: %lu val: %.3f", name, s->param, (long unsigned int)size, (double)f);

				if (bson_encoder_append_double(&encoder, name, f) != 0) {
					PX4_ERR("BSON append failed for '%s'", name);
					goto out;
				}
			}
			break;

		default:
			PX4_ERR("%s unrecognized parameter type %d, skipping export", name, getParameterType(s->param));
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

void ParameterServer::Run()
{

	// Check for parameter requests (get/set/list)
	if (_param_request_sub.updated()) {
		parameter_request_s request;

		if (_param_request_sub.copy(&request)) {

			if (request.message_type == parameter_request_s::MESSAGE_TYPE_PARAM_REQUEST_READ) {

				parameter_value_s parameter_value{};
				parameter_value.param_count = countUsed();

				param_t param = findParameter(request.name);

				if (param != PARAM_INVALID) {

					parameter_value.param_index = param;

					switch (getParameterType(param)) {
					case PARAM_TYPE_INT32: {
							int32_t v;

							if (getParameterDefaultValue(param, &v) == 0) {
								parameter_value.type = parameter_request_s::TYPE_INT32;
								parameter_value.int64_value = v;
								parameter_value.timestamp = hrt_absolute_time();
								_param_response_pub.publish(parameter_value);
							}
						}
						break;

					case PARAM_TYPE_FLOAT: {
							float v;

							if (getParameterDefaultValue(param, &v) == 0) {
								parameter_value.type = parameter_request_s::TYPE_FLOAT32;
								parameter_value.float64_value = v;
								parameter_value.timestamp = hrt_absolute_time();
								_param_response_pub.publish(parameter_value);
							}
						}
						break;
					}
				}

			} else if (request.message_type == parameter_request_s::MESSAGE_TYPE_PARAM_SET) {

				param_t param = findParameter(request.name);

				if (param != PARAM_INVALID) {
					switch (request.type) {
					case parameter_request_s::TYPE_BOOL:

					case parameter_request_s::TYPE_UINT8:
					case parameter_request_s::TYPE_INT32:
					case parameter_request_s::TYPE_INT64: {
							int32_t i32_value = request.int64_value;
							setParameter(param, &i32_value);
						}
						break;

					case parameter_request_s::TYPE_FLOAT32:
					case parameter_request_s::TYPE_FLOAT64: {
							float float32_value = request.float64_value;
							setParameter(param, &float32_value);
						}
					}
				}

			} else if (request.message_type == parameter_request_s::MESSAGE_TYPE_PARAM_REQUEST_LIST) {

				// TODO:


			} else if (request.node_id == parameter_request_s::NODE_ID_ALL) {
				if (request.message_type == parameter_request_s::MESSAGE_TYPE_PARAM_REQUEST_LIST) {

					// TODO:

				}
			}
		}
	}

	// autosave worker, run last
	if (_autosave_scheduled.load()) {

		bool disabled = false;

		if (!getDefaultFile()) {
			// In case we save to FLASH, defer param writes until disarmed,
			// as writing to FLASH can stall the entire CPU (in rare cases around 300ms on STM32F7)
			_armed_sub.update();

			if (_armed_sub.get().armed) {
				//work_queue(LPWORK, &autosave_work, (worker_t)&autosave_worker, nullptr, USEC2TICK(1_s));
				return;
			}
		}

		lockWriter();
		_last_autosave_timestamp = hrt_absolute_time();
		_autosave_scheduled.store(false);
		disabled = _autosave_disabled;
		unlockWriter();

		if (disabled) {
			return;
		}

		PX4_DEBUG("Autosaving params");
		int ret = saveDefault();

		if (ret != 0) {
			PX4_ERR("param auto save failed (%i)", ret);
		}

	}

	if (_notify_scheduled.load()) {
		lockReader();

		_notify_scheduled.store(false);

		parameter_update_s pup{};
		pup.instance = _param_instance++;
		pup.get_count = perf_event_count(_get_count_perf);
		pup.set_count = perf_event_count(_set_perf);
		pup.find_count = perf_event_count(_find_count_perf);
		pup.export_count = perf_event_count(_export_perf);
		pup.active = _params_active.count();
		pup.changed = _params_changed.count();
		pup.custom_default = _params_custom_default.count();
		pup.timestamp = hrt_absolute_time();
		_parameter_update_pub.publish(pup);

		unlockReader();
	}

}
