/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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



#include <px4_module.h>
#include <px4_workqueue.h>

#include <tunes/tune_definition.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>

#define UPDATE_RATE	(1000000)	  /* microseconds, 1 Hz */

class RC_Loss_Alarm: public ModuleBase<RC_Loss_Alarm>
{
public:
	RC_Loss_Alarm() = default;
	~RC_Loss_Alarm();
	RC_Loss_Alarm(const RC_Loss_Alarm &other) = delete;
	RC_Loss_Alarm(const RC_Loss_Alarm &&other) = delete;
	RC_Loss_Alarm &operator= (const RC_Loss_Alarm &other) = delete;
	RC_Loss_Alarm &operator= (const RC_Loss_Alarm &&other) = delete;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

private:
	static struct work_s	_work;
	int 			_vehicle_status_sub = -1;
	static orb_advert_t 	_tune_control_pub;
	static bool 		_was_armed;
	static bool 		_had_rc;  // Don't trigger alarm for systems without RC

	static void cycle_trampoline(void *arg);
	void 	    cycle();
	static void pub_tune();
	static void stop_tune();
	static int  reset_module();
};
