
/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
 * @file wqueue_test.cpp
 * Example for Linux
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 */

#include "wqueue_test.h"

#include <px4_platform_common/time.h>
#include <px4_platform_common/workqueue.h>
#include <unistd.h>
#include <stdio.h>

px4::AppState WQueueTest::appState;

void WQueueTest::hp_worker_cb(void *p)
{
	WQueueTest *wqep = (WQueueTest *)p;

	wqep->do_hp_work();
}

void WQueueTest::lp_worker_cb(void *p)
{
	WQueueTest *wqep = (WQueueTest *)p;

	wqep->do_lp_work();
}

void WQueueTest::do_lp_work()
{
	static int iter = 0;
	printf("done lp work\n");

	if (iter > 5) {
		_lpwork_done = true;
	}

	++iter;

	work_queue(LPWORK, &_lpwork, (worker_t)&lp_worker_cb, this, 1000);
}

void WQueueTest::do_hp_work()
{
	static int iter = 0;
	printf("done hp work\n");

	if (iter > 5) {
		_hpwork_done = true;
	}

	++iter;

	// requeue
	work_queue(HPWORK, &_hpwork, (worker_t)&hp_worker_cb, this, 1000);
}

int WQueueTest::main()
{
	appState.setRunning(true);

	//Put work on HP work queue
	work_queue(HPWORK, &_hpwork, (worker_t)&hp_worker_cb, this, 1000);


	//Put work on LP work queue
	work_queue(LPWORK, &_lpwork, (worker_t)&lp_worker_cb, this, 1000);


	// Wait for work to finsh
	while (!appState.exitRequested() && !(_hpwork_done && _lpwork_done)) {
		printf("  Sleeping for 2 sec...\n");
		sleep(2);
	}

	return 0;
}
