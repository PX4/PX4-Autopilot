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

#include "uORBTest_UnitTest.hpp"
#include "uORBCommon.hpp"
#include <px4_config.h>
#include <px4_time.h>
#include <stdio.h>

uORBTest::UnitTest &uORBTest::UnitTest::instance()
{
	static uORBTest::UnitTest t;
	return t;
}

int uORBTest::UnitTest::pubsublatency_main(void)
{
  /* poll on test topic and output latency */
  float latency_integral = 0.0f;

  /* wakeup source(s) */
  px4_pollfd_struct_t fds[3];

  int test_multi_sub = orb_subscribe_multi(ORB_ID(orb_test), 0);
  int test_multi_sub_medium = orb_subscribe_multi(ORB_ID(orb_test_medium), 0);
  int test_multi_sub_large = orb_subscribe_multi(ORB_ID(orb_test_large), 0);

  struct orb_test_large t;

  /* clear all ready flags */
  orb_copy(ORB_ID(orb_test), test_multi_sub, &t);
  orb_copy(ORB_ID(orb_test_medium), test_multi_sub_medium, &t);
  orb_copy(ORB_ID(orb_test_large), test_multi_sub_large, &t);

  fds[0].fd = test_multi_sub;
  fds[0].events = POLLIN;
  fds[1].fd = test_multi_sub_medium;
  fds[1].events = POLLIN;
  fds[2].fd = test_multi_sub_large;
  fds[2].events = POLLIN;

  const unsigned maxruns = 1000;
  unsigned timingsgroup = 0;

  unsigned *timings = new unsigned[maxruns];

  for (unsigned i = 0; i < maxruns; i++) {
    /* wait for up to 500ms for data */
    int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);
    if (fds[0].revents & POLLIN) {
      orb_copy(ORB_ID(orb_test), test_multi_sub, &t);
      timingsgroup = 0;
    } else if (fds[1].revents & POLLIN) {
      orb_copy(ORB_ID(orb_test_medium), test_multi_sub_medium, &t);
      timingsgroup = 1;
    } else if (fds[2].revents & POLLIN) {
      orb_copy(ORB_ID(orb_test_large), test_multi_sub_large, &t);
      timingsgroup = 2;
    }

    if (pret < 0) {
      warn("poll error %d, %d", pret, errno);
      continue;
    }

    hrt_abstime elt = hrt_elapsed_time(&t.time);
    latency_integral += elt;
    timings[i] = elt;
  }

  orb_unsubscribe(test_multi_sub);
  orb_unsubscribe(test_multi_sub_medium);
  orb_unsubscribe(test_multi_sub_large);

  if (pubsubtest_print) {
    char fname[32];
    //sprintf(fname, "/fs/microsd/timings%u.txt", timingsgroup);
    sprintf(fname, "/tmp/timings%u.txt", timingsgroup);
    FILE *f = fopen(fname, "w");
    if (f == NULL) {
      warnx("Error opening file!\n");
      return uORB::ERROR;
        }

    for (unsigned i = 0; i < maxruns; i++) {
      fprintf(f, "%u\n", timings[i]);
    }

    fclose(f);
  }

  delete[] timings;

  warnx("mean: %8.4f", static_cast<double>(latency_integral / maxruns));

  pubsubtest_passed = true;

  if (static_cast<float>(latency_integral / maxruns) > 30.0f) {
    pubsubtest_res = uORB::ERROR;
  } else {
    pubsubtest_res = PX4_OK;
  }

  return pubsubtest_res;
}

int uORBTest::UnitTest::test()
{
  struct orb_test t, u;
  int sfd;
  orb_advert_t ptopic;
  bool updated;

  t.val = 0;
  ptopic = orb_advertise(ORB_ID(orb_test), &t);

  if (ptopic == nullptr)
    return test_fail("advertise failed: %d", errno);

  test_note("publish handle 0x%08x", ptopic);
  sfd = orb_subscribe(ORB_ID(orb_test));

  if (sfd < 0)
    return test_fail("subscribe failed: %d", errno);

  test_note("subscribe fd %d", sfd);
  u.val = 1;

  if (PX4_OK != orb_copy(ORB_ID(orb_test), sfd, &u))
    return test_fail("copy(1) failed: %d", errno);

  if (u.val != t.val)
    return test_fail("copy(1) mismatch: %d expected %d", u.val, t.val);

  if (PX4_OK != orb_check(sfd, &updated))
    return test_fail("check(1) failed");

  if (updated)
    return test_fail("spurious updated flag");

  t.val = 2;
  test_note("try publish");

  if (PX4_OK != orb_publish(ORB_ID(orb_test), ptopic, &t))
    return test_fail("publish failed");

  if (PX4_OK != orb_check(sfd, &updated))
    return test_fail("check(2) failed");

  if (!updated)
    return test_fail("missing updated flag");

  if (PX4_OK != orb_copy(ORB_ID(orb_test), sfd, &u))
    return test_fail("copy(2) failed: %d", errno);

  if (u.val != t.val)
    return test_fail("copy(2) mismatch: %d expected %d", u.val, t.val);

  orb_unsubscribe(sfd);

  /* this routine tests the multi-topic support */
  test_note("try multi-topic support");

  int instance0;
  orb_advert_t pfd0 = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance0, ORB_PRIO_MAX);

  test_note("advertised");

  int instance1;
  orb_advert_t pfd1 = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance1, ORB_PRIO_MIN);

  if (instance0 != 0)
    return test_fail("mult. id0: %d", instance0);

  if (instance1 != 1)
    return test_fail("mult. id1: %d", instance1);

  t.val = 103;
  if (PX4_OK != orb_publish(ORB_ID(orb_multitest), pfd0, &t))
    return test_fail("mult. pub0 fail");

  test_note("published");

  t.val = 203;
  if (PX4_OK != orb_publish(ORB_ID(orb_multitest), pfd1, &t))
    return test_fail("mult. pub1 fail");

  /* subscribe to both topics and ensure valid data is received */
  int sfd0 = orb_subscribe_multi(ORB_ID(orb_multitest), 0);

  if (PX4_OK != orb_copy(ORB_ID(orb_multitest), sfd0, &u))
    return test_fail("sub #0 copy failed: %d", errno);

  if (u.val != 103)
    return test_fail("sub #0 val. mismatch: %d", u.val);

  int sfd1 = orb_subscribe_multi(ORB_ID(orb_multitest), 1);

  if (PX4_OK != orb_copy(ORB_ID(orb_multitest), sfd1, &u))
    return test_fail("sub #1 copy failed: %d", errno);

  if (u.val != 203)
    return test_fail("sub #1 val. mismatch: %d", u.val);

  /* test priorities */
  int prio;
  if (PX4_OK != orb_priority(sfd0, &prio))
    return test_fail("prio #0");

  if (prio != ORB_PRIO_MAX)
    return test_fail("prio: %d", prio);

  if (PX4_OK != orb_priority(sfd1, &prio))
    return test_fail("prio #1");

  if (prio != ORB_PRIO_MIN)
    return test_fail("prio: %d", prio);

  if (PX4_OK != latency_test<struct orb_test>(ORB_ID(orb_test), false))
    return test_fail("latency test failed");

  return test_note("PASS");
}

int uORBTest::UnitTest::info()
{
  return OK;
}

int uORBTest::UnitTest::test_fail(const char *fmt, ...)
{
  va_list ap;

  fprintf(stderr, "FAIL: ");
  va_start(ap, fmt);
  vfprintf(stderr, fmt, ap);
  va_end(ap);
  fprintf(stderr, "\n");
  fflush(stderr);
  return uORB::ERROR;
}

int uORBTest::UnitTest::test_note(const char *fmt, ...)
{
  va_list ap;

  fprintf(stderr, "note: ");
  va_start(ap, fmt);
  vfprintf(stderr, fmt, ap);
  va_end(ap);
  fprintf(stderr, "\n");
  fflush(stderr);
  return OK;
}

int uORBTest::UnitTest::pubsubtest_threadEntry(char* const argv[])
{
  uORBTest::UnitTest &t = uORBTest::UnitTest::instance();
  return t.pubsublatency_main();
}
