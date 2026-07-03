/****************************************************************************
 *
 *   Copyright (c) 2023-2024 PX4 Development Team. All rights reserved.
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

#pragma once

#include <cstring>
#include <cstdlib>

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/time.h>

/**
 * @brief Base class for work-queue drivers that support multiple simultaneous instances.
 *
 * Replaces ModuleBase<T> when more than one instance of the same driver must coexist
 * (e.g. two rudder controllers on a twin-steering vessel).
 *
 * The derived class T must implement:
 *   static int task_spawn(int instance, int argc, char *argv[])
 *   static int custom_command(int argc, char *argv[])
 *   static int print_usage(const char *reason = nullptr)
 *   int        print_status()   — virtual, called per-instance by status_all()
 *
 * Run() must call cleanup_instance(_instance_index) after ScheduleClear()
 * when should_exit() returns true.
 *
 * Shell usage (routed by main()):
 *   <driver> start  [-i <n>]   — start instance n (default 0)
 *   <driver> stop   [-i <n>]   — stop  instance n (default 0)
 *   <driver> status            — print status of all instances
 *
 * @tparam T      Derived driver class (CRTP pattern)
 * @tparam MAX_N  Maximum number of simultaneous instances
 */
template<class T, uint8_t MAX_N>
class MultiInstanceModuleBase
{
public:
	MultiInstanceModuleBase() = default;
	virtual ~MultiInstanceModuleBase() = default;

	/**
	 * Main entry point — routes start / stop [-i N] / status / help.
	 * Call this from the module's extern "C" main function.
	 */
	static int main(int argc, char *argv[])
	{
		if (argc < 2 ||
		    strcmp(argv[1], "-h")    == 0 ||
		    strcmp(argv[1], "help")  == 0 ||
		    strcmp(argv[1], "usage") == 0) {
			return T::print_usage();
		}

		if (strcmp(argv[1], "start") == 0) {
			int instance = _parse_instance(argc - 1, argv + 1);

			if (instance < 0 || instance >= (int)MAX_N) {
				PX4_ERR("invalid instance %d (max %d)", instance, (int)MAX_N - 1);
				return PX4_ERROR;
			}

			if (_instances[instance] != nullptr) {
				PX4_ERR("instance %d already running", instance);
				return PX4_ERROR;
			}

			return T::task_spawn(instance, argc - 1, argv + 1);
		}

		if (strcmp(argv[1], "stop") == 0) {
			int instance = _parse_instance(argc - 1, argv + 1);

			if (instance < 0 || instance >= (int)MAX_N) {
				PX4_ERR("invalid instance %d (max %d)", instance, (int)MAX_N - 1);
				return PX4_ERROR;
			}

			return _stop_instance(instance);
		}

		if (strcmp(argv[1], "status") == 0) {
			return _status_all();
		}

		return T::custom_command(argc - 1, argv + 1);
	}

	/** Returns true if instance index is currently running. */
	static bool is_running(int index)
	{
		return index >= 0 && index < (int)MAX_N && _instances[index] != nullptr;
	}

	/** Returns pointer to instance or nullptr. */
	static T *get_instance(int index)
	{
		if (index >= 0 && index < (int)MAX_N) { return _instances[index]; }

		return nullptr;
	}

	/**
	 * Called from Run() after ScheduleClear() when should_exit() is true.
	 * Nulls the instance pointer first, then deletes the object.
	 * Do not access 'this' after calling this method.
	 */
	static void cleanup_instance(int index)
	{
		if (index >= 0 && index < (int)MAX_N) {
			T *inst = _instances[index];
			_instances[index] = nullptr;
			delete inst;
		}
	}

	/** Request this instance to stop on the next Run() cycle. */
	virtual void request_stop() { _task_should_exit.store(true); }

	/** Returns true if this instance should stop. Check at the top of Run(). */
	bool should_exit() const { return _task_should_exit.load(); }

	/**
	 * Override to print per-instance status information.
	 * Called by status_all() for each running instance.
	 */
	virtual int print_status()
	{
		PX4_INFO("Instance %d: running", _instance_index);
		return 0;
	}

protected:
	static T *_instances[MAX_N];
	int       _instance_index{0};

private:
	px4::atomic_bool _task_should_exit{false};

	static int _parse_instance(int argc, char *argv[])
	{
		for (int i = 0; i < argc - 1; i++) {
			if (!strcmp(argv[i], "-i")) { return atoi(argv[i + 1]); }
		}

		return 0;
	}

	static int _stop_instance(int index)
	{
		if (_instances[index] == nullptr) {
			PX4_WARN("instance %d not running", index);
			return PX4_OK;
		}

		_instances[index]->request_stop();

		// Wait up to 5 s for the work item to call cleanup_instance() from Run()
		for (int i = 0; i < 500 && _instances[index] != nullptr; i++) {
			px4_usleep(10000);
		}

		if (_instances[index] != nullptr) {
			PX4_WARN("instance %d stop timed out, forcing deletion", index);
			T *inst = _instances[index];
			_instances[index] = nullptr;
			delete inst;
		}

		PX4_INFO("instance %d stopped", index);
		return PX4_OK;
	}

	static int _status_all()
	{
		for (int i = 0; i < (int)MAX_N; i++) {
			if (_instances[i] != nullptr) {
				_instances[i]->print_status();

			} else {
				PX4_INFO("Instance %d: not running", i);
			}
		}

		return 0;
	}
};

// Static member definition — one array per (T, MAX_N) instantiation
template<class T, uint8_t MAX_N>
T *MultiInstanceModuleBase<T, MAX_N>::_instances[MAX_N] = {};
