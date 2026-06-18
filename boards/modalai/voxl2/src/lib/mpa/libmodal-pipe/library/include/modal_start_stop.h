/*******************************************************************************
 * Copyright 2025 ModalAI Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 ******************************************************************************/



#ifndef MODAL_START_STOP_H
#define MODAL_START_STOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <sched.h>
#include <pthread.h>
#include <stdint.h>

// This is set to 0 by the signal handler, you should set it to 1 when you are
// done initializing then so your main() while loop can check this for shutdown.
extern volatile int main_running;

/**
 * @brief      Makes a PID file PID_FILE (/run/name.pid) containing the current
 *             PID of your process
 *
 * @param[in]  name  The desired name, usually name of the executable
 *
 * @return     Returns 0 if successful. If that file already exists then it is
 *             not touched and this function returns 1 at which point we suggest
 *             you run kill_exising_process() to kill that process if you only
 *             want one instance running at a time. Returns -1 if there is some
 *             other problem writing to the file.
 */
int make_pid_file(const char* name);


/**
 * @brief      This function is used to make sure any existing program using the
 *             PID file (/run/name.pid) is stopped.
 *
 *             The user doesn't need to integrate this in their own program
 *             However, it's nice if you only want one instance of a process
 *             ever to run at the same time. For example, voxl-vision-px4 starts
 *             as a background process via systemd but users may want to run it
 *             in interactive mode for debugging. voxl-vision-px4 will use this
 *             function to kill the background process before starting itself
 *             again.
 *
 * @param[in]  name       process name to stop, usually your own
 * @param[in]  timeout_s  timeout period to wait for process to close cleanly,
 *                        must be >=0.1, 2.0 seconds is usually good.
 *
 * @return     return values:
 * - -4: invalid argument or other error
 * - -3: insufficient privileges to kill existing process
 * - -2: unreadable or invalid contents in PID_FILE
 * - -1: existing process failed to close cleanly and had to be killed
 * -  0: No existing process was running
 * -  1: An existing process was running but it shut down cleanly.
 */
int kill_existing_process(const char* name, float timeout_s);


/**
 * @brief      Removes the PID file (/run/name.pid) created by make_pid_file().
 *
 *             This should be called before your program closes to make sure its
 *             PID file isn't left in the file system.
 *
 * @param[in]  name  The name of the process to remove the associated file
 *                   (/run/name.pid)
 *
 * @return     Returns 0 whether or not the file was actually there. Returns -1
 *             if there was a file system error.
 */
int remove_pid_file(const char* name);


/**
 * @brief      Enables a generic signal handler. Optional but recommended.
 *
 *             This catches SIGINT, SIGTERM, SIGHUP, and SIGSEGV with the
 *             following behavior:
 *
 * - SIGINT (ctrl-c) and SIGTERM: Sets the global variable main_running to 0
 *     indicating to the process that it's time for threads to shut down cleanly
 * - SIGHUP: Ignored to prevent process from stopping due to bad network
 *     connection when starting processes over SSH
 * - SIGSEGV:  Segfaults will be caught and print some debugging info to stderr
 *     before setting main_running to 0. Behavior with segfaults is not
 *     guaranteed to be predictable.
 *
 * @return     Returns 0 on success or -1 on error
 */
int enable_signal_handler(void);


/**
 * @brief      Disables the signal handlers enabled by enable_signal_handler
 *
 * @return     Returns 0 on success or -1 on error
 */
int disable_signal_handler(void);


/**
 * recommended thread priorities to be used with the following functions:
 * - pipe_pthread_set_priority()
 * - pipe_pthread_create()
 * - pipe_server_set_control_handler_priority()
 * - pipe_client_set_helper_priority()
 *
 * Default priority is to inherit the priority and scheduler of the calling
 * thread. This should be used the vast majority of the time. For special cases
 * such as the flight critical VIO pipeline, the Linux Real-Time FIFO scheduler
 * can be used with a piority between 1 and 99. Use the RT scheduler
 * carefully.
 */
#define THREAD_PRIORITY_DEFAULT		0
#define THREAD_PRIORITY_RT_LOW		20
#define THREAD_PRIORITY_RT_MED		50
#define THREAD_PRIORITY_RT_HIGH		80



/**
 * @brief      Set the policy and priority for the calling process ID.
 *
 *             See THREAD_PRIORITY_* definitions above.Setting priority to 0
 *             indicates the thread should use Linux pthread default scheduler
 *             and priority. Otherwise 1-99 sets the thread to use the Real-Time
 *             FIFO scheduler with specified priority.
 *
 * @param[in]  priority  The priority 0-99
 *
 * @return     0 on success, -1 on failure
 */
int pipe_set_process_priority(int priority);


/**
 * @brief      print the scheduler and priority number to stdout for the
 *             specified thread.
 *
 * @param[in]  thread  Thread ID, set to 0 to act on the calling pthread.
 *
 * @return     0 on success, -1 on failure
 */
int pipe_pthread_print_properties(pthread_t thread);


/**
 * @brief      Set scheduler and priority for a pthread.
 *
 *             See THREAD_PRIORITY_* definitions above.Setting priority to 0
 *             indicates the thread should use Linux pthread default scheduler
 *             and priority. Otherwise 1-99 sets the thread to use the Real-Time
 *             FIFO scheduler with specified priority.
 *
 *             This function should not need to be used unless you absolutely
 *             want to change the priority of a thread after it's been started.
 *             Instead set the priority of client helper thread and server
 *             control pipe handler thread before they start with:
 *
 *            - pipe_server_set_control_handler_priority()
 *            - pipe_client_set_helper_priority()
 *
 *             If you are starting your own threads you can start them with the
 *             desired priority with pipe_pthread_create()
 *
 * @param[in]  thread    Thread ID, set to 0 to act on the calling pthread.
 * @param[in]  priority  The priority 0-99
 *
 * @return     0 on success, -1 on failure
 */
int pipe_pthread_set_priority(pthread_t thread, int priority);


/**
 * @brief      Start a pthread with specified priority.
 *
 *             See THREAD_PRIORITY_* definitions above.Setting priority to 0
 *             indicates the thread should use Linux pthread default scheduler
 *             and priority. Otherwise 1-99 sets the thread to use the Real-Time
 *             FIFO scheduler with specified priority.
 *
 *
 *
 * @param      thread    Pointer to the pthread id
 * @param      func      The function pointer
 * @param      arg       The argument to pass to the pthread
 * @param[in]  priority  The priority to use
 *
 * @return     0 on success, -1 on failure
 */
int pipe_pthread_create(pthread_t *thread, void*(*func)(void*), void* arg, int priority);



/**
 * @brief      Set the CPU affinity for the current thread
 *
 *             This is just a helpful wrapper for pthread_setaffinity_np
 *
 *             The affinity applies to the thread that calls it plus any
 *             child threads that are started afterwards.
 *
 * @param      set    standard cpu_set_t
 *
 * @return     0 on success, -1 on failure
 */
int set_cpu_affinity(cpu_set_t set);
void print_cpu_affinity(void);


/**
 * @brief      These are helpers for creating cpu_set_t sets specific to VOXL boards
 *
 *             VOXL 2's QRB5165 chip has 8 cores arranged as follows
 *             0 A55 small core
 *             1 A55 small core
 *             2 A55 small core
 *             3 A55 small core
 *             4 A77 big core
 *             5 A77 big core
 *             6 A77 big core
 *             4 A77 gold core
 *
 *             VOXL 1's APQ8096 chip has 4 cores arranged as follows
 *             0 small core
 *             1 small core
 *             2 big core
 *             3 big core
 *
 *             If a function is called that doesn't apply to the host processor
 *             or you are running this on a different platform, a set containing
 *             all available cores is returned.
 *
 * @return     a cpu_set_t struct that can be passed directly into set_cpu_affinity(set)
 */
cpu_set_t cpu_set_small_cores(void);
cpu_set_t cpu_set_big_cores(void);
cpu_set_t cpu_set_gold_core(void);
cpu_set_t cpu_set_big_cores_and_gold_core(void);
cpu_set_t cpu_set_all_cores(void);


////////////////////////////////////////////////////////////////////////////////
// not strictly for starting and stopping, but common functions that live in
// main loops all over MPA
////////////////////////////////////////////////////////////////////////////////


int64_t mpa_time_monotonic_ns(void);


int64_t mpa_time_realtime_ns(void);


void mpa_nanosleep(uint64_t ns);


int mpa_loop_sleep(double rate_hz, int64_t* next_time, int en_warning);


#ifdef __cplusplus
}
#endif

#endif // MODAL_START_STOP_H

