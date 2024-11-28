/****************************************************************************
 *
 *   Copyright (C) 2015-2022 PX4 Development Team. All rights reserved.
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
 * @file main.cpp
 *
 * This is the main() of PX4 for POSIX.
 *
 * The application is designed as a daemon/server app with multiple clients.
 * Both, the server and the client is started using this main() function.
 *
 * If the executable is called with its usual name 'px4', it will start the
 * server. However, if it is started with an executable name (symlink) starting
 * with 'px4-' such as 'px4-navigator', it will start as a client and try to
 * connect to the server.
 *
 * The symlinks for all modules are created using the build system.
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 * @author Roman Bapst <bapstroman@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include <algorithm>
#include <csignal>
#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#if (_POSIX_MEMLOCK > 0)
#include <sys/mman.h>
#endif

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/init.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>

#include "hrt_work.h"
#include "px4_daemon/client.h"
#include "px4_daemon/pxh.h"
#include "px4_daemon/server.h"
#include <board_ctrl.h>
#include <drivers/drv_hrt.h>
#include <errno.h>
#include <hrt_work.h>
#include <parameters/param.h>
#include <pthread.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/init.h>
#include <px4_platform_common/shutdown.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/workqueue.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <thread>
#include <unistd.h>

#ifndef PATH_MAX
#define PATH_MAX 1024
#endif

pthread_t main_thread;
std::thread *lockstep_thread = nullptr;
static volatile bool _shutdown_seen = false;
static volatile bool _received_sigint = false;

static void sig_int_handler(int sig_num) { Board::instance()->handle_sigint(); }

static void register_sig_handler() {
  // SIGINT
  struct sigaction sig_int {};
  sig_int.sa_handler = sig_int_handler;
  sig_int.sa_flags = 1; // not SA_RESTART!

  // SIGPIPE
  // We want to ignore if a PIPE has been closed.
  struct sigaction sig_pipe {};
  sig_pipe.sa_handler = SIG_IGN;

#ifdef __PX4_CYGWIN
  // Do not catch SIGINT on Cygwin such that the process gets killed
  // TODO: All threads should exit gracefully see https://github.com/PX4/Firmware/issues/11027
  (void)sig_int; // this variable is unused
#else
  sigaction(SIGINT, &sig_int, nullptr);
#endif

  sigaction(SIGTERM, &sig_int, nullptr);
  sigaction(SIGPIPE, &sig_pipe, nullptr);
}

namespace px4 {
void init_once();
}

static void sig_int_handler(int sig_num);

static void register_sig_handler();
static int run_startup_script(const std::string &commands_file,
                              const std::string &absolute_binary_path, int instance) {
  std::string shell_command("/bin/sh ");

  shell_command += commands_file + ' ' + std::to_string(instance);

  // Update the PATH variable to include the absolute_binary_path
  // (required for the px4-alias.sh script and px4-* commands).
  // They must be within the same directory as the px4 binary
  const char *path_variable = "PATH";
  std::string updated_path = absolute_binary_path;
  const char *path = getenv(path_variable);

  if (path) {
    std::string spath = path;

    // Check if absolute_binary_path already in PATH
    bool already_in_path = false;
    std::size_t current, previous = 0;
    current = spath.find(':');

    while (current != std::string::npos) {
      if (spath.substr(previous, current - previous) == absolute_binary_path) {
        already_in_path = true;
      }

      previous = current + 1;
      current = spath.find(':', previous);
    }

    if (spath.substr(previous, current - previous) == absolute_binary_path) {
      already_in_path = true;
    }

    if (!already_in_path) {
      // Prepend to path to prioritize PX4 commands over potentially already installed PX4 commands.
      updated_path = updated_path + ":" + path;
      setenv(path_variable, updated_path.c_str(), 1);
    }
  }

  PX4_INFO("startup script: %s", shell_command.c_str());

  int ret = 0;

  if (!shell_command.empty()) {
    ret = system(shell_command.c_str());

    if (ret == 0) {
      PX4_INFO("Startup script returned successfully");

    } else {
      PX4_ERR("Startup script returned with return value: %d", ret);
    }

  } else {
    PX4_INFO("Startup script empty");
  }

  return ret;
}

void px4_terminate() { px4_platform_fini(); }

Board *Board::instance() {
  static Board _inst;
  return &_inst;
}

void Board::shutdown(bool reset) {
  _reset_req = reset;
  _shutdown_seen = true;
  PX4_INFO("shutdown request ");
  if (!_received_sigint) pthread_kill(main_thread, SIGINT);
}

void Board::handle_sigint() {
  _received_sigint = true;
  PX4_INFO("Sigint ");
  lockstep_start();

  px4_daemon::Pxh::stop();
  PX4_INFO("Sigint stop");
}

void Board::run() {
  main_thread = pthread_self();
  register_sig_handler();
  while (true) {
    this->run_once();
    if (!_reset_req) break;
  }
}

void wait_to_exit() {
  while (!_shutdown_seen) {
    // needs to be a regular sleep not dependent on lockstep (not px4_usleep)
    usleep(100000);
  }
}

int Board::run_once() {

  _received_sigint = false;
  px4_daemon::Server server(parameters.server_instance);
  server.start();

  PX4_INFO("Initting");
  px4::init_once();
  px4::init(parameters.argc, parameters.argv, "px4");
  _shutdown_seen = false;
  _reset_req = false;

  PX4_INFO("start startup script");
  auto ret = run_startup_script(parameters.commands_file, parameters.absolute_binary_paths,
                                parameters.server_instance);

  PX4_INFO("Ret run startup >> %d %d\n", _shutdown_seen, ret);

  if (ret == 0 && !_shutdown_seen) {
    // We now block here until we need to exit.
    if (parameters.pxh_off) {
      wait_to_exit();

    } else {
      px4_daemon::Pxh pxh;
      pxh.run_pxh();
      pxh.stop();
    }
  }

  printf("=========== OFF MAIN LOOP >> %d", _reset_req);
  if (!_shutdown_seen) {
    PX4_INFO("Interrupt - requesting shutdown");
    this->shutdown(false);
    // initial ctrl
    // noreturn
  }

  PX4_INFO("Waiting for exit");
  fflush(stdout);
  printf("\nPX4 Exiting... xoxo\n");
  fflush(stdout);
  wait_to_exit();
  PX4_INFO("Done exit");

  px4_terminate();
  lockstep_cleanup();
  sleep(1);
  return PX4_OK;
}

bool _lockstep_exit_req;

void Board::lockstep_cleanup() {
  _lockstep_exit_req = true;
	px4_lockstep_notify_startup();
  if (lockstep_thread != nullptr) {
    lockstep_thread->join();
    lockstep_thread = nullptr;
  }
}

void Board::lockstep_threadfunc() {
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	PX4_INFO("On lockstep thread");
  struct timespec ts;
  hrt_abstime last_time_mon;
  hrt_abstime last_time_sys;
  px4_clock_gettime(CLOCK_MONOTONIC, &ts);
  last_time_mon = ts_to_abstime(&ts);
  px4_clock_gettime(CLOCK_REALTIME, &ts);
  last_time_sys = ts_to_abstime(&ts);

  using namespace time_literals;
  while (!_lockstep_exit_req) {
    usleep(10);
    px4_clock_gettime(CLOCK_REALTIME, &ts);
    hrt_abstime cur_time_sys = std::max(last_time_sys + 10_us, ts_to_abstime(&ts));

    last_time_mon += cur_time_sys - last_time_sys;
    last_time_sys = cur_time_sys;
    abstime_to_ts(&ts, last_time_mon);
    px4_lockstep_settime_shutdown(&ts);
  }

#endif
}
void Board::lockstep_start() {
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
  if (lockstep_thread != nullptr) return;
  _lockstep_exit_req = false;
	px4_lockstep_notify_shutdown();
  lockstep_thread = new std::thread(&Board::lockstep_threadfunc, this);
#endif
}
