/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include <nuttx/config.h>
#include <nuttx/clock.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/rptun/rptun.h>
#include <nuttx/semaphore.h>

#include <drivers/drv_hrt.h>

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "remote_link.h"

static struct rpmsg_endpoint g_ept;
static sem_t g_sem;
static char g_reply[128];
static size_t g_reply_len;
static const char *g_cpuname;
static bool g_registered;

static int ept_cb(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv)
{
	g_reply_len = len < sizeof(g_reply) - 1 ? len : sizeof(g_reply) - 1;
	memcpy(g_reply, data, g_reply_len);
	g_reply[g_reply_len] = '\0';
	nxsem_post(&g_sem);
	return 0;
}

static void ept_unbind(struct rpmsg_endpoint *ept)
{
	rpmsg_destroy_ept(ept);
}

static void device_created(struct rpmsg_device *rdev, void *priv)
{
	if (strcmp(rpmsg_get_cpuname(rdev), g_cpuname) != 0) {
		return;
	}

	rpmsg_create_ept(&g_ept, rdev, "rpmsg-hello", RPMSG_ADDR_ANY, RPMSG_ADDR_ANY, ept_cb, ept_unbind);
}

static void device_destroy(struct rpmsg_device *rdev, void *priv)
{
	if (g_ept.rdev == rdev) {
		rpmsg_destroy_ept(&g_ept);
	}
}

void remote_link_register(const char *cpuname)
{
	if (g_registered) {
		return;
	}

	g_cpuname = cpuname;
	nxsem_init(&g_sem, 0, 0);
	rpmsg_register_callback(NULL, device_created, device_destroy, NULL, NULL);
	g_registered = true;
}

int remote_link_rptun(bool start)
{
	char dev[32];
	snprintf(dev, sizeof(dev), "/dev/rptun/%s", g_cpuname);

	int fd = open(dev, O_RDONLY);

	if (fd < 0) {
		return -errno;
	}

	/* The backend holds the core in reset on stop, so no endpoint teardown
	 * message can reach it: skip them instead of waiting on a dead remote.
	 */
	int ret = start ? ioctl(fd, RPTUNIOC_START, 0) : ioctl(fd, RPTUNIOC_STOP, RPTUN_STOP_NO_NS);
	int err = errno;
	close(fd);
	return ret < 0 ? -err : 0;
}

int remote_link_xfer(const void *payload, size_t len, char *reply, size_t cap, int timeout_ms,
		     uint64_t *rtt_us)
{
	/* dest address arrives with the remote's name-service announce or ack */
	for (int i = 0; i < 200 && !is_rpmsg_ept_ready(&g_ept); i++) {
		usleep(10000);
	}

	if (!is_rpmsg_ept_ready(&g_ept)) {
		return -ENOTCONN;
	}

	while (nxsem_trywait(&g_sem) == 0) {
	}

	hrt_abstime start = hrt_absolute_time();
	int ret = rpmsg_send(&g_ept, payload, len);

	if (ret < 0) {
		return ret;
	}

	if (nxsem_tickwait(&g_sem, MSEC2TICK(timeout_ms)) < 0) {
		return -ETIMEDOUT;
	}

	if (rtt_us) {
		*rtt_us = hrt_elapsed_time(&start);
	}

	if (reply) {
		strncpy(reply, g_reply, cap - 1);
		reply[cap - 1] = '\0';
	}

	return 0;
}
