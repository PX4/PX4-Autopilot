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

/**
 * @file qnx_state_resmgr.cpp
 *
 * QNX resource manager that publishes live vehicle state at /dev/px4/state.
 * Any QNX process can read it with open()/read() (e.g. `cat /dev/px4/state`)
 * without linking PX4, uORB or MAVLink. Each open() gets a fresh snapshot
 * as one line of key=value text.
 *
 * The channel is created with _NTO_CHF_FIXED_PRIORITY, so the server thread
 * keeps its own low priority instead of inheriting a reader's: even a reader
 * at a real-time priority cannot run this code above the control loops.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/tasks.h>
#include <drivers/drv_hrt.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// Each open() gets its own OCB with its own snapshot, so concurrent readers
// never see each other's bytes.
struct state_ocb_t;
#define IOFUNC_OCB_T struct state_ocb_t
#include <sys/dispatch.h>
#include <sys/iofunc.h>
#include <sys/neutrino.h>

struct state_ocb_t {
	iofunc_ocb_t hdr; // must be first
	char text[256];
	size_t len;
};

static constexpr const char *kPath = "/dev/px4/state";

class QnxStateResmgr : public ModuleBase
{
public:
	static Descriptor desc;

	static int task_spawn(int argc, char *argv[]);
	static int run_trampoline(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);

	void run() override;
	int print_status() override;

private:
	static int io_open(resmgr_context_t *ctp, io_open_t *msg, RESMGR_HANDLE_T *handle, void *extra);
	static int io_read(resmgr_context_t *ctp, io_read_t *msg, RESMGR_OCB_T *ocb);
	static state_ocb_t *ocb_calloc(resmgr_context_t *ctp, iofunc_attr_t *attr);
	static void ocb_free(state_ocb_t *ocb);

	// Format the current state into buf. Called from the resmgr thread only.
	size_t format_state(char *buf, size_t size);

	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _battery_sub{ORB_ID(battery_status)};

	unsigned _reads{0};

	resmgr_connect_funcs_t _connect_funcs{};
	resmgr_io_funcs_t _io_funcs{};
	iofunc_funcs_t _ocb_funcs{};
	iofunc_mount_t _mount{};
	iofunc_attr_t _attr{};
};

ModuleBase::Descriptor QnxStateResmgr::desc{task_spawn, custom_command, print_usage};

static QnxStateResmgr *g_self = nullptr;

size_t QnxStateResmgr::format_state(char *buf, size_t size)
{
	vehicle_status_s status{};
	vehicle_local_position_s lpos{};
	battery_status_s battery{};
	const bool have_status = _status_sub.copy(&status);
	const bool have_lpos = _lpos_sub.copy(&lpos);
	const bool have_battery = _battery_sub.copy(&battery);

	const int n = snprintf(buf, size,
			       "t_us=%llu armed=%d nav_state=%d "
			       "x=%.2f y=%.2f alt=%.2f vx=%.2f vy=%.2f vz=%.2f heading_deg=%.1f "
			       "pos_valid=%d battery=%.2f\n",
			       (unsigned long long)hrt_absolute_time(),
			       have_status ? (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) : -1,
			       have_status ? (int)status.nav_state : -1,
			       have_lpos ? (double)lpos.x : (double)NAN,
			       have_lpos ? (double)lpos.y : (double)NAN,
			       have_lpos ? (double) - lpos.z : (double)NAN,
			       have_lpos ? (double)lpos.vx : (double)NAN,
			       have_lpos ? (double)lpos.vy : (double)NAN,
			       have_lpos ? (double)lpos.vz : (double)NAN,
			       have_lpos ? (double)(lpos.heading * 57.29578f) : (double)NAN,
			       have_lpos ? (lpos.xy_valid && lpos.z_valid) : 0,
			       have_battery ? (double)battery.remaining : -1.0);

	return (n > 0) ? math::min((size_t)n, size - 1) : 0;
}

int QnxStateResmgr::io_open(resmgr_context_t *ctp, io_open_t *msg, RESMGR_HANDLE_T *handle, void *extra)
{
	// read-only device, even for root
	if ((msg->connect.ioflag & _IO_FLAG_WR) != 0) {
		return EROFS;
	}

	return iofunc_open_default(ctp, msg, static_cast<iofunc_attr_t *>(handle), extra);
}

state_ocb_t *QnxStateResmgr::ocb_calloc(resmgr_context_t *ctp, iofunc_attr_t *attr)
{
	state_ocb_t *ocb = static_cast<state_ocb_t *>(calloc(1, sizeof(state_ocb_t)));

	if (ocb != nullptr) {
		ocb->len = g_self->format_state(ocb->text, sizeof(ocb->text));
	}

	return ocb;
}

void QnxStateResmgr::ocb_free(state_ocb_t *ocb)
{
	free(ocb);
}

int QnxStateResmgr::io_read(resmgr_context_t *ctp, io_read_t *msg, RESMGR_OCB_T *ocb_handle)
{
	state_ocb_t *ocb = static_cast<state_ocb_t *>(ocb_handle);
	int status = iofunc_read_verify(ctp, msg, &ocb->hdr, nullptr);

	if (status != EOK) {
		return status;
	}

	if ((msg->i.xtype & _IO_XTYPE_MASK) != _IO_XTYPE_NONE) {
		return ENOSYS;
	}

	const size_t offset = (size_t)ocb->hdr.offset;
	size_t nbytes = 0;

	if (offset < ocb->len) {
		nbytes = math::min(ocb->len - offset, (size_t)msg->i.nbytes);
	}

	if (nbytes > 0) {
		SETIOV(ctp->iov, ocb->text + offset, nbytes);
		ocb->hdr.offset += nbytes;
		g_self->_reads++;
	}

	_IO_SET_READ_NBYTES(ctp, nbytes);
	return _RESMGR_NPARTS(nbytes > 0 ? 1 : 0);
}

void QnxStateResmgr::run()
{
	g_self = this;

	// Fixed priority: don't let a high-priority client lift this thread.
	const int chid = ChannelCreate(_NTO_CHF_FIXED_PRIORITY | _NTO_CHF_DISCONNECT | _NTO_CHF_UNBLOCK);

	if (chid == -1) {
		PX4_ERR("ChannelCreate failed: %s", strerror(errno));
		return;
	}

	dispatch_t *dpp = dispatch_create_channel(chid, DISPATCH_FLAG_NOLOCK);

	if (dpp == nullptr) {
		PX4_ERR("dispatch_create_channel failed: %s", strerror(errno));
		ChannelDestroy(chid);
		return;
	}

	iofunc_func_init(_RESMGR_CONNECT_NFUNCS, &_connect_funcs, _RESMGR_IO_NFUNCS, &_io_funcs);
	_connect_funcs.open = io_open;
	_io_funcs.read = io_read;
	_ocb_funcs.nfuncs = _IOFUNC_NFUNCS;
	_ocb_funcs.ocb_calloc = ocb_calloc;
	_ocb_funcs.ocb_free = ocb_free;
	_mount.funcs = &_ocb_funcs;
	iofunc_attr_init(&_attr, S_IFCHR | 0444, nullptr, nullptr);
	_attr.mount = &_mount;

	resmgr_attr_t rattr{};
	rattr.nparts_max = 1;
	rattr.msg_max_size = 2048;

	const int id = resmgr_attach(dpp, &rattr, kPath, _FTYPE_ANY, 0, &_connect_funcs, &_io_funcs, &_attr);

	if (id == -1) {
		PX4_ERR("resmgr_attach(%s) failed: %s (needs the PATHSPACE ability, e.g. run as root)", kPath, strerror(errno));
		dispatch_destroy(dpp);
		return;
	}

	PX4_INFO("serving vehicle state at %s (fixed priority %d)", kPath, SCHED_PRIORITY_DEFAULT - 30);

	// Wake up every 500 ms so should_exit() is checked even with no readers.
	struct timespec timeout {0, 500 * 1000 * 1000};
	dispatch_timeout(dpp, &timeout);
	dispatch_context_t *ctp = dispatch_context_alloc(dpp);

	while (!should_exit() && ctp != nullptr) {
		dispatch_context_t *ready = dispatch_block(ctp);

		if (ready != nullptr) {
			ctp = ready;
			dispatch_handler(ctp);
		}
	}

	resmgr_detach(dpp, id, _RESMGR_DETACH_ALL);

	if (ctp != nullptr) {
		dispatch_context_free(ctp);
	}

	dispatch_destroy(dpp);
	g_self = nullptr;
}

int QnxStateResmgr::print_status()
{
	PX4_INFO("path %s, %u reads served", kPath, _reads);
	return 0;
}

int QnxStateResmgr::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		QnxStateResmgr *instance = new QnxStateResmgr();

		if (instance == nullptr)
		{
			PX4_ERR("alloc failed");
		}

		return instance;
	}, argc, argv);
}

int QnxStateResmgr::task_spawn(int argc, char *argv[])
{
	desc.task_id = px4_task_spawn_cmd("qnx_state_resmgr",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT - 30,
					  PX4_STACK_ADJUSTED(4096),
					  (px4_main_t)&run_trampoline,
					  (char *const *)argv);

	if (desc.task_id < 0) {
		desc.task_id = -1;
		return -errno;
	}

	return 0;
}

int QnxStateResmgr::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
QNX resource manager that serves live vehicle state as text at /dev/px4/state.
Any QNX process can read it, e.g. `cat /dev/px4/state`.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("qnx_state_resmgr", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int qnx_state_resmgr_main(int argc, char *argv[])
{
	return ModuleBase::main(QnxStateResmgr::desc, argc, argv);
}
