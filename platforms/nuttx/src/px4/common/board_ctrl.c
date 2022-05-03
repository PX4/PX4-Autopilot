/****************************************************************************
 *
 *   Copyright (C) 2020 Technology Innovation Institute. All rights reserved.
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
 * @file board_ctrl.c
 *
 * Provide a kernel-userspace boardctl_ioctl interface
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform/board_ctrl.h>
#include "board_config.h"

#include <nuttx/lib/builtin.h>
#include <NuttX/kernel_builtin/kernel_builtin_proto.h>

FAR const struct builtin_s g_kernel_builtins[] = {
#include <NuttX/kernel_builtin/kernel_builtin_list.h>
};

const int g_n_kernel_builtins = sizeof(g_kernel_builtins) / sizeof(struct builtin_s);

static struct {
	ioctl_ptr_t ioctl_func;
} ioctl_ptrs[MAX_IOCTL_PTRS];

/************************************************************************************
 * Name: px4_register_boardct_ioctl
 *
 * Description:
 *   an interface function for kernel services to register an ioct handler for user side
 *
 ************************************************************************************/


int px4_register_boardct_ioctl(unsigned base, ioctl_ptr_t func)
{
	unsigned i = IOCTL_BASE_TO_IDX(base);
	int ret = PX4_ERROR;

	if (i < MAX_IOCTL_PTRS) {
		ioctl_ptrs[i].ioctl_func = func;
		ret = PX4_OK;
	}

	return ret;
}

/************************************************************************************
 * Name: board_ioctl
 *
 * Description:
 *   implements board_ioctl for userspace-kernel interface
 *
 ************************************************************************************/

int board_ioctl(unsigned int cmd, uintptr_t arg)
{
	unsigned i = IOCTL_BASE_TO_IDX(cmd);
	int ret = -EINVAL;

	if (i < MAX_IOCTL_PTRS && ioctl_ptrs[i].ioctl_func) {
		ret = ioctl_ptrs[i].ioctl_func(cmd, arg);
	}

	return ret;
}

/************************************************************************************
 * Name: launch_kernel_builtin
 *
 * Description:
 *   launches a kernel-side build-in module
 *
 ************************************************************************************/

static int launch_kernel_builtin(int argc, char **argv)
{
	int i;
	FAR const struct builtin_s *builtin = NULL;

	for (i = 0; i < g_n_kernel_builtins; i++) {
		if (!strcmp(g_kernel_builtins[i].name, argv[0])) {
			builtin = &g_kernel_builtins[i];
			break;
		}
	}

	if (builtin) {
		/* This is running in the userspace thread, created by nsh, and
		   called via boardctl. Call the main directly */
		return builtin->main(argc, argv);
	}

	return ENOENT;
}

/************************************************************************************
 * Name: platform_ioctl
 *
 * Description:
 *   handle all miscellaneous platform level ioctls
 *
 ************************************************************************************/

static int platform_ioctl(unsigned int cmd, unsigned long arg)
{
	int ret = PX4_OK;

	switch (cmd) {
	case PLATFORMIOCLAUNCH: {
			platformioclaunch_t *data = (platformioclaunch_t *)arg;
			data->ret = launch_kernel_builtin(data->argc, data->argv);
		}
		break;

	case PLATFORMIOCVBUSSTATE: {
			platformiocvbusstate_t *data = (platformiocvbusstate_t *)arg;
			data->ret = px4_arch_gpioread(GPIO_OTGFS_VBUS) ? 0 : 1;
		}
		break;

	case PLATFORMIOCINDICATELOCKOUT: {
			platformioclockoutstate_t *data = (platformioclockoutstate_t *)arg;
			px4_arch_configgpio(data->enabled ? GPIO_nARMED : GPIO_nARMED_INIT);
		}
		break;

	case PLATFORMIOCGETLOCKOUT: {
			platformioclockoutstate_t *data = (platformioclockoutstate_t *)arg;
			data->enabled = px4_arch_gpioread(GPIO_nARMED);
		}
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}


/************************************************************************************
 * Name: kernel_ioctl_initialize
 *
 * Description:
 *   initializes the kernel-side ioctl interface
 *
 ************************************************************************************/

void kernel_ioctl_initialize(void)
{
	for (int i = 0; i < MAX_IOCTL_PTRS; i++) {
		ioctl_ptrs[i].ioctl_func = NULL;
	}

	px4_register_boardct_ioctl(_PLATFORMIOCBASE, platform_ioctl);
}
