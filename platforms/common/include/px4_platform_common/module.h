/****************************************************************************
 *
 *   Copyright (c) 2017-2025 PX4 Development Team. All rights reserved.
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
 * @file module.h
 *
 * Non-template base class for modules. Replaces the CRTP ModuleBase<T>
 * pattern with a descriptor-based approach that shares a single copy of the
 * common static methods (main, start, stop, status, etc.) across all modules.
 */

#pragma once

#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/tasks.h>
#include <systemlib/px4_macros.h>

#include <pthread.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus

#include <px4_platform_common/atomic.h>

/**
 * @brief This mutex protects against race conditions during startup & shutdown of modules.
 *        There could be one mutex per module instantiation, but to reduce the memory footprint
 *        there is only a single global mutex. This sounds bad, but we actually don't expect
 *        contention here, as module startup is sequential.
 */
extern pthread_mutex_t px4_modules_mutex;

/**
 * @class ModuleBase
 *      Non-template base class for modules, implementing common functionality
 *      such as 'start', 'stop' and 'status' commands.
 *
 *      Each module provides a static Descriptor instance that holds
 *      module-specific function pointers and per-module storage (object
 *      pointer and task ID). The shared static methods operate on the
 *      descriptor instead of CRTP template statics.
 */
class ModuleBase
{
public:
	/**
	 * Per-module descriptor providing module-specific function pointers
	 * and storage (object pointer, task ID).
	 */
	struct Descriptor {
		Descriptor(int (*ts)(int, char **), int (*cc)(int, char **), int (*pu)(const char *))
			: task_spawn(ts), custom_command(cc), print_usage(pu) {}

		int (*task_spawn)(int argc, char *argv[]);
		int (*custom_command)(int argc, char *argv[]);
		int (*print_usage)(const char *reason);
		px4::atomic<ModuleBase *> object{nullptr};
		int task_id{-1};
	};

	ModuleBase() = default;
	virtual ~ModuleBase() = default;

	/**
	 * @brief Main entry point to the module that should be called directly
	 *        from the module's main method.
	 */
	static int main(Descriptor &desc, int argc, char *argv[]);

	/**
	 * @brief Start command: checks if running, calls desc.task_spawn().
	 */
	static int start_command(Descriptor &desc, int argc, char *argv[]);

	/**
	 * @brief Stop command: request stop and wait for exit.
	 */
	static int stop_command(Descriptor &desc);

	/**
	 * @brief Status command: call print_status() on running instance.
	 */
	static int status_command(Descriptor &desc);

	/**
	 * @brief Cleanup: delete object, reset task_id. Called from module thread.
	 */
	static void exit_and_cleanup(Descriptor &desc);

	/**
	 * @brief Check if the module is running.
	 */
	static bool is_running(const Descriptor &desc) { return desc.task_id != -1; }

	/**
	 * @brief Wait until the object is initialized (called from task_spawn).
	 */
	static int wait_until_running(Descriptor &desc, int timeout_ms = 1000);

	/**
	 * @brief Shared run-trampoline for thread-based modules.
	 *
	 * Each thread-based module provides a thin per-module trampoline
	 * that calls this with its descriptor + instantiate function.
	 */
	using instantiate_fn = ModuleBase * (*)(int argc, char *argv[]);
	static int run_trampoline_impl(Descriptor &desc, instantiate_fn instantiate,
				       int argc, char *argv[]);

	/**
	 * @brief Main loop method for modules running in their own thread.
	 */
	virtual void run() {}

	/**
	 * @brief Print the status of the module.
	 */
	virtual int print_status();

	/**
	 * @brief Tells the module to stop.
	 */
	virtual void request_stop() { _task_should_exit.store(true); }

	/**
	 * @brief Checks if the module should stop.
	 */
	bool should_exit() const { return _task_should_exit.load(); }

	/**
	 * @brief Typed accessor for the running module instance.
	 *        Centralizes the ModuleBase* -> T* downcast in one place.
	 */
	template<typename T>
	static T *get_instance(Descriptor &desc)
	{
		return static_cast<T *>(desc.object.load()); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
	}

	static constexpr int task_id_is_work_queue = -2;

protected:
	px4::atomic_bool _task_should_exit{false};
};

#endif /* __cplusplus */


/* -------- C-linkage declarations for PRINT_MODULE_* helpers -------- */

__BEGIN_DECLS

#ifdef __PX4_NUTTX
static inline void PRINT_MODULE_DESCRIPTION(const char *description) {}
#else
__EXPORT void PRINT_MODULE_DESCRIPTION(const char *description);
#endif

__EXPORT void PRINT_MODULE_USAGE_NAME(const char *executable_name, const char *category);
__EXPORT void PRINT_MODULE_USAGE_SUBCATEGORY(const char *subcategory);
__EXPORT void PRINT_MODULE_USAGE_NAME_SIMPLE(const char *executable_name, const char *category);
__EXPORT void PRINT_MODULE_USAGE_COMMAND_DESCR(const char *name, const char *description);

#define PRINT_MODULE_USAGE_COMMAND(name) \
	PRINT_MODULE_USAGE_COMMAND_DESCR(name, NULL);

#define PRINT_MODULE_USAGE_DEFAULT_COMMANDS() \
	PRINT_MODULE_USAGE_COMMAND("stop"); \
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "print status info");

__EXPORT void PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(bool i2c_support, bool spi_support);
__EXPORT void PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(uint8_t default_address);
__EXPORT void PRINT_MODULE_USAGE_PARAMS_I2C_KEEP_RUNNING_FLAG(void);
__EXPORT void PRINT_MODULE_USAGE_PARAM_INT(char option_char, int default_val, int min_val, int max_val,
		const char *description, bool is_optional);
__EXPORT void PRINT_MODULE_USAGE_PARAM_FLOAT(char option_char, float default_val, float min_val, float max_val,
		const char *description, bool is_optional);
__EXPORT void PRINT_MODULE_USAGE_PARAM_FLAG(char option_char, const char *description, bool is_optional);
__EXPORT void PRINT_MODULE_USAGE_PARAM_STRING(char option_char, const char *default_val, const char *values,
		const char *description, bool is_optional);
__EXPORT void PRINT_MODULE_USAGE_PARAM_COMMENT(const char *comment);
__EXPORT void PRINT_MODULE_USAGE_ARG(const char *values, const char *description, bool is_optional);

__END_DECLS
