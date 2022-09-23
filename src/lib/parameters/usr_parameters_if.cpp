/****************************************************************************
 *
 *   Copyright (c) 2021 Technology Innovation Institute. All rights reserved.
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
 * @file usr_parameters_if.cpp
 *
 * Protected build user space interface to global parameter store.
 */
#define PARAM_IMPLEMENTATION
#include "param.h"
#include "parameters_ioctl.h"
#include <parameters/px4_parameters.hpp>
#include <sys/boardctl.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

#include "parameters_common.cpp"

void
param_notify_changes()
{
	boardctl(PARAMIOCNOTIFY, NULL);
}

param_t param_find(const char *name)
{
	paramiocfind_t data = {name, true, PARAM_INVALID};
	boardctl(PARAMIOCFIND, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

param_t param_find_no_notification(const char *name)
{
	paramiocfind_t data = {name, false, PARAM_INVALID};
	boardctl(PARAMIOCFIND, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

unsigned param_count_used()
{
	paramioccountused_t data = {0};
	boardctl(PARAMIOCCOUNTUSED, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

param_t param_for_used_index(unsigned index)
{
	paramiocforusedindex_t data = {index, 0};
	boardctl(PARAMIOCFORUSEDINDEX, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

int param_get_used_index(param_t param)
{
	paramiocgetusedindex_t data = {param, 0};
	boardctl(PARAMIOCGETUSEDINDEX, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

bool
param_value_unsaved(param_t param)
{
	paramiocunsaved_t data = {param, false};
	boardctl(PARAMIOCUNSAVED, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

int
param_get(param_t param, void *val)
{
	paramiocget_t data = {param, false, val, PX4_ERROR};
	boardctl(PARAMIOCGET, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

int
param_get_default_value(param_t param, void *default_val)
{
	paramiocget_t data = {param, true, default_val, PX4_ERROR};
	boardctl(PARAMIOCGET, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

void
param_control_autosave(bool enable)
{
	paramiocautosave_t data = {enable};
	boardctl(PARAMIOCAUTOSAVE, reinterpret_cast<unsigned long>(&data));
}

int param_set(param_t param, const void *val)
{
	paramiocset_t data = {param, true, val, PX4_ERROR};
	boardctl(PARAMIOCSET, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

int param_set_no_notification(param_t param, const void *val)
{
	paramiocset_t data = {param, false, val, PX4_ERROR};
	boardctl(PARAMIOCSET, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

bool param_used(param_t param)
{
	paramiocused_t data = {param, false};
	boardctl(PARAMIOCUSED, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

void param_set_used(param_t param)
{
	paramiocsetused_t data = {param};
	boardctl(PARAMIOCSETUSED, reinterpret_cast<unsigned long>(&data));
}

int param_set_default_value(param_t param, const void *val)
{
	paramiocsetdefault_t data = {param, val, PX4_ERROR};
	boardctl(PARAMIOCSETDEFAULT, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

int param_reset(param_t param)
{
	paramiocreset_t data = {param, true, PX4_ERROR};
	boardctl(PARAMIOCRESET, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

int param_reset_no_notification(param_t param)
{
	paramiocreset_t data = {param, false, PX4_ERROR};
	boardctl(PARAMIOCRESET, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

void
param_reset_all()
{
	paramiocresetgroup_t data = {PARAM_RESET_ALL, nullptr, 0};
	boardctl(PARAMIOCRESETGROUP, reinterpret_cast<unsigned long>(&data));
}

void
param_reset_excludes(const char *excludes[], int num_excludes)
{
	paramiocresetgroup_t data = {PARAM_RESET_EXCLUDES, excludes, num_excludes};
	boardctl(PARAMIOCRESETGROUP, reinterpret_cast<unsigned long>(&data));
}

void
param_reset_specific(const char *resets[], int num_resets)
{
	paramiocresetgroup_t data = {PARAM_RESET_SPECIFIC, resets, num_resets};
	boardctl(PARAMIOCRESETGROUP, reinterpret_cast<unsigned long>(&data));
}

int param_save_default()
{
	paramiocsavedefault_t data = {PX4_ERROR};
	boardctl(PARAMIOCSAVEDEFAULT, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

int
param_load_default()
{
	paramiocloaddefault_t data = {PX4_ERROR};
	boardctl(PARAMIOCLOADDEFAULT, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

int
param_export(const char *filename, param_filter_func filter)
{
	paramiocexport_t data = {filename, PX4_ERROR};

	if (filter) { PX4_ERR("ERROR: filter not supported in userside blob"); }

	boardctl(PARAMIOCEXPORT, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}

uint32_t param_hash_check()
{
	paramiochash_t data = {0};
	boardctl(PARAMIOCHASH, reinterpret_cast<unsigned long>(&data));
	return data.ret;
}
