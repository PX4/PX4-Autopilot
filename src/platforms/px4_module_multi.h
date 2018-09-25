/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file px4_module.h
 */

#pragma once

#include <pthread.h>
#include <unistd.h>
#include <stdbool.h>

#include <px4_log.h>
#include <px4_tasks.h>
#include <systemlib/px4_macros.h>

#ifdef __cplusplus

#include <cstring>

/**
 * @brief This mutex protects against race conditions during startup & shutdown of modules.
 *        There could be one mutex per module instantiation, but to reduce the memory footprint
 *        there is only a single global mutex. This sounds bad, but we actually don't expect
 *        contention here, as module startup is sequential.
 */
extern pthread_mutex_t px4_modules_mutex;

template<class T>
class ModuleBaseMulti;

// Module node in the singly linked list
struct Module {
	int _id;
	void *_module;

	Module *next;
	Module *prev;
};


class ModuleList
{
public:
	ModuleList()
	{
		_head = NULL;
		_tail = NULL;
		_available_ids = 0;
	}

	int _available_ids;

	Module *get_head()
	{
		return _head;
	}

	Module *get_node(int module_id)
	{
		Module *ptr = nullptr;

		// Search across the linked list until we find the ID or until we reach the end of the list.
		for (ptr = _head; (ptr->_id != module_id); ptr = ptr->next) {
			// If next is null, we hit the end
			if (ptr->next == nullptr) {
				ptr = nullptr;
				return ptr;
			}

			// If we found the module_id, return pointer to the node
			if (ptr->_id == module_id) {
				return ptr;
			}
		}

		return ptr;
	}

	int add_node(void *obj)
	{
		Module *temp = new Module;
		temp->_module = obj;
		temp->_id = ++_available_ids;
		temp->next = nullptr;

		if (!_head) {
			_head = temp;
			_tail = temp;
			temp = nullptr;

		} else {
			_tail->next = temp;
			_tail = temp;
		}

		return _tail->_id;
	}

private:
	Module *_head;
	Module *_tail;

	void delete_node(Module *obj)
	{
		// If we are the tail, give it to guy behind us
		if (obj == _tail) {
			_tail = obj->prev;

		} else if (obj == _head) {
			_head = obj->next;

		} else {
			// Link prev to next
			obj->prev->next = obj->next;
			// Link next to prev
			obj->next->prev = obj->prev;
		}

		delete obj;
	}
};

template<class T>
class ModuleBaseMulti
{
public:
	ModuleBaseMulti() = default;
	virtual ~ModuleBaseMulti() {};

	static int main(int argc, char *argv[])
	{
		if (argc <= 1 ||
		    strcmp(argv[1], "-h")    == 0 ||
		    strcmp(argv[1], "help")  == 0 ||
		    strcmp(argv[1], "info")  == 0 ||
		    strcmp(argv[1], "usage") == 0) {
			return T::print_usage();
		}

		if (strcmp(argv[1], "start") == 0) {
			// Pass the 'start' argument too, because later on px4_getopt() will ignore the first argument.
			return start_command_base(argc - 1, argv + 1);
		}

		if (strcmp(argv[1], "status") == 0) {
			int module_id = 0;

			if (argv[2]) {
				module_id = atoi(argv[2]);
			}

			return status_command(module_id);
		}

		if (strcmp(argv[1], "stop") == 0) {
			int module_id = 0;

			if (argv[2]) {
				module_id = atoi(argv[2]);
			}

			return stop_command(module_id);
		}

		lock_module(); // Lock here, as the method could access _object.
		int ret = T::custom_command(argc - 1, argv + 1);
		unlock_module();

		return ret;
	}

	static int run_trampoline(int argc, char *argv[])
	{
		int ret = 0;

#ifdef __PX4_NUTTX
		// On NuttX task_create() adds the task name as first argument.
		argc -= 1;
		argv += 1;
#endif


		// Dervied class must define an instantiate() function. Creates the object and passes it up.
		T *obj = T::instantiate(argc, argv);
		obj->_instance_id = add_instance_to_list(obj);
		PX4_INFO("Object added to list");

		if (obj) {
			// This is the final function and just chills here if in a px4_task_spawn()
			obj->run();

		} else {
			PX4_ERR("failed to instantiate object");
			ret = -1;
		}

		exit_and_cleanup(obj);

		return ret;
	}


	static int start_command_base(int argc, char *argv[])
	{

		// If no module created, create one. If module exits, user must explicity specify they want to create another.
		if (!strcmp(argv[1], "+") == 0 && _module_list.get_head() != nullptr) {
			PX4_ERR("Task already running.");
			return PX4_ERROR;
		}

		lock_module();


		// Task spawn should only fire off the px4_task_spawn function and then enter at run_trampoline()
		// OR
		// If WORK_QUEUE, this is the final function call from ModuleBaseMulti!!
		// If WORK_QUEUE, must use set_object() to pass the module instance into the linked list.
		int task_id = T::task_spawn(argc, argv);

		if (task_id < 0) {
			PX4_ERR("Task start failed (%i)", task_id);
		}

		unlock_module();
		return task_id;
	}


	static int stop_command(int module_id)
	{
		int ret = 0;
		lock_module();

		// get_instance will return a pointer to the module with the ID, or nullptr if it does not exist
		T *obj = get_instance(module_id);

		if (obj) {
			obj->request_stop();

			unsigned int i = 0;

			do {
				unlock_module();
				usleep(20000); // 20 ms
				lock_module();

				if (++i > 100 && _task_id != -1) { // wait at most 2 sec
					if (_task_id != task_id_is_work_queue) {
						px4_task_delete(_task_id);
					}

					_task_id = -1;

					if (obj) {
						delete obj;
					}

					ret = -1;
					break;
				}
			} while (_task_id != -1);

		} else {
			// In the very unlikely event that can only happen on work queues,
			// if the starting thread does not wait for the work queue to initialize,
			// and inside the work queue, the allocation of _object fails
			// and exit_and_cleanup() is not called, set the _task_id as invalid.
			PX4_WARN("Mode with ID %d not found.", module_id);
			_task_id = -1;
		}


		unlock_module();
		return ret;
	}

	/**
	 * @brief Handle 'command status': check if running and call print_status() if it is
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int status_command(int module_id)
	{
		// get_instance will return a pointer to the module with the ID, or nullptr if it does not exist
		T *obj = get_instance(module_id);

		int ret = -1;
		lock_module();

		if (obj) {
			ret = obj->print_status();

		} else {
			PX4_INFO("not running");
		}

		unlock_module();
		return ret;
	}

	/**
	 * @brief Print the status if the module is running. This can be overridden by the module to provide
	 * more specific information.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	virtual int print_status()
	{
		PX4_INFO("running");
		return 0;
	}

	/**
	 * @brief Main loop method for modules running in their own thread. Called from run_trampoline().
	 *        This method must return when should_exit() returns true.
	 */
	virtual void run()
	{
	}

	/**
	 * @brief Returns the status of the module.
	 * @return Returns true if the module is running, false otherwise.
	 */
	static bool is_running()
	{
		return _task_id != -1;
	}

	static int add_instance_to_list(T *obj)
	{
		return _module_list.add_node(obj);
	}


protected:

	/**
	 * @brief Tells the module to stop (used from outside or inside the module thread).
	 */
	virtual void request_stop()
	{
		_task_should_exit = true;
	}

	/**
	 * @brief Checks if the module should stop (used within the module thread).
	 * @return Returns True iff successful, false otherwise.
	 */
	bool should_exit() const
	{
		return _task_should_exit;
	}

	/**
	 * @brief Exits the module and delete the object. Called from within the module's thread.
	 *        For work queue modules, this needs to be called from the derived class in the
	 *        cycle method, when should_exit() returns true.
	 */
	static void exit_and_cleanup(T *obj)
	{
		// Take the lock here:
		// - if startup fails and we're faster than the parent thread, it will set
		//   _task_id and subsequently it will look like the task is running.
		// - deleting the object must take place inside the lock.
		lock_module();

		if (obj) {
			delete obj;
		}

		_task_id = -1; // Signal a potentially waiting thread for the module to exit that it can continue.
		unlock_module();
	}

	// /**
	//  * @brief Waits until _object is initialized, (from the new thread). This can be called from task_spawn().
	//  * @return Returns 0 iff successful, -1 on timeout or otherwise.
	//  */
	// static int wait_until_running()
	// {
	// 	int i = 0;

	// 	do {
	// 		/* Wait up to 1s. */
	// 		usleep(2500);

	// 	} while (!_object && ++i < 400);

	// 	if (i == 400) {
	// 		PX4_ERR("Timed out while waiting for thread to start");
	// 		return -1;
	// 	}

	// 	return 0;
	// }

	/**
	 * @brief Get the module's object instance, (this is null if it's not running).
	 */
	static T *get_instance(int module_id)
	{

		Module *node = _module_list.get_node(module_id);
		T *instance = reinterpret_cast<T *>(node->_module);

		return instance;
	}

	int _instance_id = -1;

	static ModuleList _module_list;

	/** @var _task_id The task handle: -1 = invalid, otherwise task is assumed to be running. */
	static int _task_id;

	/** @var task_id_is_work_queue Value to indicate if the task runs on the work queue. */
	static constexpr const int task_id_is_work_queue = -2;

private:
	/**
	 * @brief lock_module Mutex to lock the module thread.
	 */
	static void lock_module()
	{
		pthread_mutex_lock(&px4_modules_mutex);
	}

	/**
	 * @brief unlock_module Mutex to unlock the module thread.
	 */
	static void unlock_module()
	{
		pthread_mutex_unlock(&px4_modules_mutex);
	}

	/** @var _task_should_exit Boolean flag to indicate if the task should exit. */
	volatile bool _task_should_exit = false;
};


template<class T>
ModuleList ModuleBaseMulti<T>::_module_list = {};



template<class T>
int ModuleBaseMulti<T>::_task_id = -1;


#endif /* __cplusplus */


__BEGIN_DECLS

/**
 * @brief Module documentation and command usage help methods.
 *        These are extracted with the Tools/px_process_module_doc.py
 *        script and must be kept in sync.
 */

#ifdef __PX4_NUTTX
/**
 * @note Disable module description on NuttX to reduce Flash usage.
 *       There's a GCC bug (https://gcc.gnu.org/bugzilla/show_bug.cgi?id=55971), preventing us to use
 *       a macro, but GCC will remove the string as well with this empty inline method.
 * @param description The provided functionality of the module and potentially the most important parameters.
 */
static inline void PRINT_MODULE_DESCRIPTION(const char *description) {}
#else

/**
 * @brief Prints module documentation (will also be used for online documentation). It uses Markdown syntax
 *        and should include these sections:
 * - ### Description
 *   Provided functionality of the module and potentially the most important parameters.
 * - ### Implementation
 *   High-level implementation overview
 * - ### Examples
 *   Examples how to use the CLI interface (if it's non-trivial)
 *
 * In addition to the Markdown syntax, a line beginning with '$ ' can be used to mark a command:
 * $ module start -p param
 */
__EXPORT void PRINT_MODULE_DESCRIPTION(const char *description);
#endif

/**
 * @brief Prints the command name.
 * @param executable_name: command name used in scripts & CLI
 * @param category one of: driver, estimator, controller, system, communication, command, template
 */
__EXPORT void PRINT_MODULE_USAGE_NAME(const char *executable_name, const char *category);

/**
 * @brief Prints the name for a command without any sub-commands (@see PRINT_MODULE_USAGE_NAME()).
 */
__EXPORT void PRINT_MODULE_USAGE_NAME_SIMPLE(const char *executable_name, const char *category);


/**
 * @brief Prints a command with a short description what it does.
 */
__EXPORT void PRINT_MODULE_USAGE_COMMAND_DESCR(const char *name, const char *description);

#define PRINT_MODULE_USAGE_COMMAND(name) \
	PRINT_MODULE_USAGE_COMMAND_DESCR(name, NULL);

/**
 * @brief Prints the default commands: stop & status.
 */
#define PRINT_MODULE_USAGE_DEFAULT_COMMANDS() \
	PRINT_MODULE_USAGE_COMMAND("stop"); \
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "print status info");


/** @note Each of the PRINT_MODULE_USAGE_PARAM_* methods apply to the previous PRINT_MODULE_USAGE_COMMAND_DESCR(). */

/**
 * @brief Prints an integer parameter.
 * @param option_char The option character.
 * @param default_val The parameter default value.
 * @param min_val The parameter minimum value.
 * @param max_val The parameter value.
 * @param description Pointer to the usage description.
 * @param is_optional true if this parameter is optional
 */
__EXPORT void PRINT_MODULE_USAGE_PARAM_INT(char option_char, int default_val, int min_val, int max_val,
		const char *description, bool is_optional);

/**
 * @brief Prints a float parameter.
 * @note See PRINT_MODULE_USAGE_PARAM_INT().
 * @param default_val The parameter default value.
 * @param min_val The parameter minimum value.
 * @param max_val The parameter maximum value.
 * @param description Pointer to the usage description. Pointer to the usage description.
 * @param is_optional true if this parameter is optional
 */
__EXPORT void PRINT_MODULE_USAGE_PARAM_FLOAT(char option_char, float default_val, float min_val, float max_val,
		const char *description, bool is_optional);

/**
 * @brief Prints a flag parameter, without any value.
 * @note See PRINT_MODULE_USAGE_PARAM_INT().
 * @param option_char The option character.
 * @param description Pointer to the usage description.
 * @param is_optional true if this parameter is optional
 */
__EXPORT void PRINT_MODULE_USAGE_PARAM_FLAG(char option_char, const char *description, bool is_optional);

/**
 * @brief Prints a string parameter.
 * @param option_char The option character.
 * @param default_val The default value, can be nullptr.
 * @param values The valid values, it has one of the following forms:
 *               - nullptr: leave unspecified, or any value is valid
 *               - "<file>" or "<file:dev>": a file or more specifically a device file (eg. serial device)
 *               - "<topic_name>": uORB topic name
 *               - "<value1> [<value2>]": a list of values
 *               - "on|off": a concrete set of valid strings separated by "|".
 * @param description Pointer to the usage description.
 * @param is_optional True iff this parameter is optional.
 */
__EXPORT void PRINT_MODULE_USAGE_PARAM_STRING(char option_char, const char *default_val, const char *values,
		const char *description, bool is_optional);

/**
 * @brief Prints a comment, that applies to the next arguments or parameters. For example to indicate that
 *        a parameter applies to several or all commands.
 * @param comment
 */
__EXPORT void PRINT_MODULE_USAGE_PARAM_COMMENT(const char *comment);


/**
 * @brief Prints the definition for an argument, which does not have the typical -p <val> form,
 *        but for example 'param set <param> <value>'
 * @param values eg. "<file>", "<param> <value>" or "<value1> [<value2>]"
 * @param description Pointer to the usage description.
 * @param is_optional true if this parameter is optional
 */
__EXPORT void PRINT_MODULE_USAGE_ARG(const char *values, const char *description, bool is_optional);


__END_DECLS
