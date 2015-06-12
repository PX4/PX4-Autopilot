/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

#include <string.h>
#include "uORBDevices.hpp"
#include "uORB.h"
#include "uORBCommon.hpp"

#ifndef __PX4_QURT
#include "uORBTest_UnitTest.hpp"
#endif

extern "C" { __EXPORT int uorb_main(int argc, char *argv[]); }

static uORB::DeviceMaster *g_dev = nullptr;
static void usage()
{
  warnx("Usage: uorb 'start', 'test', 'latency_test' or 'status'");
}


int
uorb_main(int argc, char *argv[])
{
  if (argc < 2) {
    usage();
    return -EINVAL;
  }

  /*
   * Start/load the driver.
   *
   * XXX it would be nice to have a wrapper for this...
   */
  if (!strcmp(argv[1], "start")) {

    if (g_dev != nullptr) {
      warnx("already loaded");
      /* user wanted to start uorb, its already running, no error */
      return 0;
    }

    /* create the driver */
    g_dev = new uORB::DeviceMaster(uORB::PUBSUB);

    if (g_dev == nullptr) {
      warnx("driver alloc failed");
      return -ENOMEM;
    }

    if (OK != g_dev->init()) {
      warnx("driver init failed");
      delete g_dev;
      g_dev = nullptr;
      return -EIO;
    }

    return OK;
  }

#ifndef __PX4_QURT
  /*
   * Test the driver/device.
   */
  if (!strcmp(argv[1], "test"))
  {
    uORBTest::UnitTest &t = uORBTest::UnitTest::instance();
    return t.test();
  }

  /*
   * Test the latency.
   */
  if (!strcmp(argv[1], "latency_test")) {

    uORBTest::UnitTest &t = uORBTest::UnitTest::instance();
    if (argc > 2 && !strcmp(argv[2], "medium")) {
      return t.latency_test<struct orb_test_medium>(ORB_ID(orb_test_medium), true);
    } else if (argc > 2 && !strcmp(argv[2], "large")) {
      return t.latency_test<struct orb_test_large>(ORB_ID(orb_test_large), true);
    } else {
      return t.latency_test<struct orb_test>(ORB_ID(orb_test), true);
    }
  }
#endif

  /*
   * Print driver information.
   */
  if (!strcmp(argv[1], "status"))
  {
    return OK;
  }

  usage();
  return -EINVAL;
}
