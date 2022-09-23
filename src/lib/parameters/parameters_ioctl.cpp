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
 * @file parameters_ioctl.cpp
 *
 * Protected build kernel space interface to global parameter store.
 */

#define PARAM_IMPLEMENTATION

#include <errno.h>

#include "param.h"
#include "parameters_ioctl.h"
#include <px4_platform_common/defines.h>

int	param_ioctl(unsigned int cmd, unsigned long arg)
{
	int ret = OK;

	switch (cmd) {
	case PARAMIOCNOTIFY: {
			param_notify_changes();
		}
		break;

	case PARAMIOCFIND: {
			paramiocfind_t *data = (paramiocfind_t *)arg;

			if (data->notification) {
				data->ret = param_find(data->name);

			} else {
				data->ret = param_find_no_notification(data->name);
			}
		}
		break;

	case PARAMIOCCOUNTUSED: {
			paramioccountused_t *data = (paramioccountused_t *)arg;
			data->ret = param_count_used();
		}
		break;

	case PARAMIOCFORUSEDINDEX: {
			paramiocforusedindex_t *data = (paramiocforusedindex_t *)arg;
			data->ret =  param_for_used_index(data->index);
		}
		break;

	case PARAMIOCGETUSEDINDEX: {
			paramiocgetusedindex_t *data = (paramiocgetusedindex_t *)arg;
			data->ret = param_get_used_index(data->param);
		}
		break;

	case PARAMIOCUNSAVED: {
			paramiocunsaved_t *data = (paramiocunsaved_t *)arg;
			data->ret = param_value_unsaved(data->param);
		}
		break;

	case PARAMIOCGET: {
			paramiocget_t *data = (paramiocget_t *)arg;

			if (data->deflt) {
				data->ret = param_get_default_value(data->param, data->val);

			} else {
				data->ret = param_get(data->param, data->val);
			}
		}
		break;

	case PARAMIOCAUTOSAVE: {
			paramiocautosave_t *data = (paramiocautosave_t *)arg;
			param_control_autosave(data->enable);
		}
		break;

	case PARAMIOCSET: {
			paramiocset_t *data = (paramiocset_t *)arg;

			if (data->notification) {
				data->ret = param_set(data->param, data->val);

			} else {
				data->ret = param_set_no_notification(data->param, data->val);
			}
		}
		break;

	case PARAMIOCUSED: {
			paramiocused_t *data = (paramiocused_t *)arg;
			data->ret = param_used(data->param);
		}
		break;

	case PARAMIOCSETUSED: {
			paramiocsetused_t *data = (paramiocsetused_t *)arg;
			param_set_used(data->param);
		}
		break;

	case PARAMIOCSETDEFAULT: {
			paramiocsetdefault_t *data = (paramiocsetdefault_t *)arg;
			data->ret = param_set_default_value(data->param, data->val);
		}
		break;

	case PARAMIOCRESET: {
			paramiocreset_t *data = (paramiocreset_t *)arg;

			if (data->notification) {
				data->ret = param_reset(data->param);

			} else {
				data->ret = param_reset_no_notification(data->param);
			}
		}
		break;

	case PARAMIOCRESETGROUP: {
			paramiocresetgroup_t *data = (paramiocresetgroup_t *)arg;

			if (data->type == PARAM_RESET_EXCLUDES) {
				param_reset_excludes(data->group, data->num_in_group);

			} else if (data->type == PARAM_RESET_SPECIFIC) {
				param_reset_specific(data->group, data->num_in_group);

			} else {
				param_reset_all();
			}
		}
		break;

	case PARAMIOCSAVEDEFAULT: {
			paramiocsavedefault_t *data = (paramiocsavedefault_t *)arg;
			data->ret = param_save_default();
		}
		break;

	case PARAMIOCLOADDEFAULT: {
			paramiocloaddefault_t *data = (paramiocloaddefault_t *)arg;
			data->ret = param_load_default();
		}
		break;

	case PARAMIOCEXPORT: {
			paramiocexport_t *data = (paramiocexport_t *)arg;
			data->ret = param_export(data->filename, nullptr);
		}
		break;

	case PARAMIOCHASH: {
			paramiochash_t *data = (paramiochash_t *)arg;
			data->ret = param_hash_check();
		}
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}
