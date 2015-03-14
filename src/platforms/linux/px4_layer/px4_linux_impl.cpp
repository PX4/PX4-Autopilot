/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
 * @file px4_linux_impl.cpp
 *
 * PX4 Middleware Wrapper Linux Implementation
 */

#include <px4_defines.h>
#include <px4_middleware.h>
#include <px4_workqueue.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include "systemlib/param/param.h"

__BEGIN_DECLS

// FIXME - This needs to be properly initialized
struct param_info_s      param_array[256];
struct param_info_s      *param_info_base;
struct param_info_s      *param_info_limit;

long PX4_TICKS_PER_SEC = sysconf(_SC_CLK_TCK);

__END_DECLS

namespace px4
{

void init(int argc, char *argv[], const char *app_name)
{
        struct param_info_s test_1 = {
                "TEST_1",
                PARAM_TYPE_INT32
        };
        test_1.val.i = 2;

        struct param_info_s test_2 = {
                "TEST_2",
                PARAM_TYPE_INT32
        };
        test_2.val.i = 4;

        struct param_info_s rc_x = {
                "RC_X",
                PARAM_TYPE_INT32
        };
        rc_x.val.i = 8;

        struct param_info_s rc2_x = {
                "RC2_X",
                PARAM_TYPE_INT32
        };
        rc2_x.val.i = 16;

        param_array[0] = test_1;
        param_array[1] = test_2;
        param_array[2] = rc_x;
        param_array[3] = rc2_x;
        param_info_base = (struct param_info_s *) &param_array[0];
        param_info_limit = (struct param_info_s *) &param_array[4];     // needs to point at the end of the data,
                                                                        // therefore number of params + 1

	printf("App name: %s\n", app_name);
}

}




int work_queue(int qid, struct work_s *work, worker_t worker, void *arg, uint32_t delay)
{
	printf("work_queue: UNIMPLEMENTED\n");
	return PX4_OK;
}

int work_cancel(int qid, struct work_s *work)
{
	printf("work_cancel: UNIMPLEMENTED\n");
	return PX4_OK;
}

