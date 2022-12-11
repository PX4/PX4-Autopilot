/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/atomic_bitset.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/sem.h>
#include <px4_platform_common/shutdown.h>
#include <containers/Bitset.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include "tinybson/tinybson.h"
#include "uthash/utarray.h"

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_request.h>
#include <uORB/topics/parameter_value.h>
#include <uORB/topics/parameter_update.h>

#include "param.h"
#include <parameters/px4_parameters.hpp>

using namespace time_literals;

class ParameterServer : public px4::ScheduledWorkItem
{
public:
	ParameterServer();
	~ParameterServer() override;

	/**
	 * Look up a parameter by name.
	 *
	 * @param name		The canonical name of the parameter being looked up.
	 * @return		A handle to the parameter, or PARAM_INVALID if the parameter does not exist.
	 */
	param_t findParameter(const char *name, bool notification = true);

	/**
	 * Return the total number of parameters.
	 *
	 * @return		The number of parameters.
	 */
	unsigned count() const { return param_info_count; }

	/**
	 * Return the actually used number of parameters.
	 *
	 * @return		The number of parameters.
	 */
	unsigned countUsed() const { return _params_active.count(); }

	/**
	 * Wether a parameter is in use in the system.
	 *
	 * @return		True if it has been written or read
	 */
	bool isParameterUsed(param_t param) const;

	/**
	 * Look up a parameter by index.
	 *
	 * @param index		An index from 0 to n, where n is param_count()-1.
	 * @return		A handle to the parameter, or PARAM_INVALID if the index is out of range.
	 */
	param_t forIndex(unsigned index) const;

	/**
	 * Look up an used parameter by index.
	 *
	 * @param index		The parameter to obtain the index for.
	 * @return		The index of the parameter in use, or -1 if the parameter does not exist.
	 */
	param_t forUsedIndex(unsigned index) const;

	/**
	 * Look up the index of a parameter.
	 *
	 * @param param		The parameter to obtain the index for.
	 * @return		The index, or -1 if the parameter does not exist.
	 */
	int getParameterIndex(param_t param) const;

	/**
	 * Look up the index of an used parameter.
	 *
	 * @param param		The parameter to obtain the index for.
	 * @return		The index of the parameter in use, or -1 if the parameter does not exist.
	 */
	int getParameterUsedIndex(param_t param) const;

	/**
	 * Obtain the name of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @return		The name assigned to the parameter, or NULL if the handle is invalid.
	 */
	const char *getParameterName(param_t param) const;

	/**
	 * Obtain the volatile state of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @return			true if the parameter is volatile
	 */
	bool isParameterVolatile(param_t param) const;

	/**
	 * Test whether a parameter's value has changed from the default.
	 *
	 * @return		If true, the parameter's value has not been changed from the default.
	 */
	bool isParameterValueDefault(param_t param);

	/**
	 * Test whether a parameter's value has been changed but not saved.
	 *
	 * @return		If true, the parameter's value has not been saved.
	 */
	bool isParameterValueUnsaved(param_t param);

	/**
	 * Obtain the type of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @return		The type assigned to the parameter.
	 */
	param_type_t getParameterType(param_t param) const;

	/**
	 * Determine the size of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @return		The size of the parameter's value.
	 */
	size_t getParameterSize(param_t param) const;

	/**
	 * Copy the value of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @param val		Where to return the value, assumed to point to suitable storage for the parameter type.
	 * @return		Zero if the parameter's value could be returned, nonzero otherwise.
	 */
	int getParameterValue(param_t param, void *val);

	/**
	 * Copy the (airframe-specific) default value of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @param default_val	Where to return the value, assumed to point to suitable storage for the parameter type.
	 * @return		Zero if the parameter's deafult value could be returned, nonzero otherwise.
	 */
	int getParameterDefaultValue(param_t param, void *default_val);

	/**
	 * Copy the system-wide default value of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @param default_val	Where to return the value, assumed to point to suitable storage for the parameter type.
	 * @return		Zero if the parameter's deafult value could be returned, nonzero otherwise.
	 */
	int getParameterSystemDefaultValue(param_t param, void *default_val);

	/**
	 * Set the value of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @param val		The value to set; assumed to point to a variable of the parameter type.
	 * @return		Zero if the parameter's value could be set from a scalar, nonzero otherwise.
	 */
	int setParameter(param_t param, const void *val, bool mark_saved = true, bool notify_changes = true);

	/**
	 * Set the default value of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @param val		The default value to set; assumed to point to a variable of the parameter type.
	 * @return		Zero if the parameter's default value could be set from a scalar, nonzero otherwise.
	 */
	int setParameterDefaultValue(param_t param, const void *val);

	/**
	 * Mark a parameter as used. Only marked parameters will be sent to a GCS.
	 * A call to param_find() will mark a param as used as well.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 */
	void setParameterUsed(param_t param);

	/**
	 * Notify the system about parameter changes. Can be used for example after several calls to
	 * param_set_no_notification() to avoid unnecessary system notifications.
	 */
	void notifyChanges();

	/**
	 * Reset a parameter to its default value.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @return		Zero on success, nonzero on failure
	 */
	int resetParameter(param_t param, bool notify = true);

	/**
	 * Reset all parameters to their default values.
	 */
	void resetAllParameters(bool auto_save = true);

	/**
	 * Reset all parameters to their default values except for excluded parameters.
	 *
	 * @param excludes			Array of param names to exclude from resetting. Use a wildcard
	 *							at the end to exclude parameters with a certain prefix.
	 * @param num_excludes		The number of excludes provided.
	 */
	void resetExcludes(const char *excludes[], int num_excludes);

	/**
	 * Reset only specific parameters to their default values.
	 *
	 * @param resets Array of param names to reset. Use a wildcard at the end to reset parameters with a certain prefix.
	 * @param num_resets The number of passed reset conditions in the resets array.
	 */
	void resetSpecificParameter(const char *resets[], int num_resets);

	/**
	 * Export changed parameters to a file.
	 * Note: this method requires a large amount of stack size!
	 *
	 * @param filename	Path to the default parameter file.
	 * @param filter	Filter parameters to be exported. The method should return true if
	 * 			the parameter should be exported. No filtering if nullptr is passed.
	 * @return		Zero on success, nonzero on failure.
	 */
	typedef bool(*param_filter_func)(param_t handle);
	int exportToFile(const char *filename, param_filter_func filter);

	/**
	 * Import parameters from a file, discarding any unrecognized parameters.
	 *
	 * This function merges the imported parameters with the current parameter set.
	 *
	 * @param fd		File descriptor to import from (-1 selects the FLASH storage).
	 * @return		Zero on success, nonzero if an error occurred during import.
	 *			Note that in the failure case, parameters may be inconsistent.
	 */
	int importFromFileDescriptor(int fd);

	/**
	 * Load parameters from a file.
	 *
	 * This function resets all parameters to their default values, then loads new
	 * values from a file.
	 *
	 * @param fd		File descriptor to import from (-1 selects the FLASH storage).
	 * @return		Zero on success, nonzero if an error occurred during import.
	 *			Note that in the failure case, parameters may be inconsistent.
	 */
	int loadFromFileDescriptor(int fd);

	/**
	 * Read saved parameters from file and dump to console.
	 *
	 * This function reads the file and dumps all contents to console
	 * values from a file.
	 *
	 * @param fd		File descriptor to read from.
	 * @return		Zero on success, nonzero if an error occurred during import.
	 */
	int bsonDump(int fd);

	/**
	 * Apply a function to each parameter.
	 *
	 * Note that the parameter set is not locked during the traversal. It also does
	 * not hold an internal state, so the callback function can block or sleep between
	 * parameter callbacks.
	 *
	 * @param func		The function to invoke for each parameter.
	 * @param arg		Argument passed to the function.
	 * @param only_changed	If true, the function is only called for parameters whose values have
	 *			been changed from the default.
	 * @param only_used	If true, the function is only called for parameters which have been
	 *			used in one of the running applications.
	 */
	void forEachParameter(void (*func)(void *arg, param_t param), void *arg, bool only_changed, bool only_used);

	/**
	 * Set the default parameter file name.
	 * This has no effect if the FLASH-based storage is enabled.
	 *
	 * @param filename	Path to the default parameter file.  The file is not required to
	 *			exist.
	 * @return		Zero on success.
	 */
	int setDefaultFile(const char *filename);

	/**
	 * Get the default parameter file name.
	 *
	 * @return		The path to the current default parameter file; either as
	 *			a result of a call to param_set_default_file, or the
	 *			built-in default.
	 */
	const char *getDefaultFile() const { return _param_default_file; }

	/**
	 * Set the backup parameter file name.
	 *
	 * @param filename	Path to the backup parameter file. The file is not required to
	 *			exist.
	 * @return		Zero on success.
	 */
	int setBackupFile(const char *filename);

	/**
	 * Get the backup parameter file name.
	 *
	 * @return		The path to the backup parameter file
	 */
	const char *getBackupFile() const { return _param_backup_file; }

	/**
	 * Automatically save the parameters after a timeout and limited rate.
	 *
	 * This needs to be called with the writer lock held (it's not necessary that it's the writer lock, but it
	 * needs to be the same lock as autosave_worker() and param_control_autosave() use).
	 */
	void autoSave(bool now = false);

	/**
	 * Load parameters from the default parameter file.
	 *
	 * @return		Zero on success.
	 */
	int loadDefault();

	/**
	 * Generate the hash of all parameters and their values
	 *
	 * @return		CRC32 hash of all param_ids and values
	 */
	uint32_t hashCheck();

	/**
	 * Print the status of the param system
	 *
	 */
	void printStatus();

	/**
	 * Enable/disable the param autosaving.
	 * Re-enabling with changed params will not cause an autosave.
	 * @param enable true: enable autosaving, false: disable autosaving
	 */
	void controlAutosave(bool enable);

private:

	static constexpr uint16_t param_info_count = sizeof(px4::parameters) / sizeof(param_info_s);

	/**
	 * Test whether a param_t is value.
	 *
	 * @param param			The parameter handle to test.
	 * @return			True if the handle is valid.
	 */
	static constexpr bool handle_in_range(param_t param) { return (param < param_info_count); }

	void lockReader();   // lock the parameter store for read access
	void unlockReader(); // unlock the parameter store

	void lockWriter();   // lock the parameter store for write access
	void unlockWriter(); // unlock the parameter store

	// Storage for modified parameters.
	struct param_wbuf_s {
		union param_value_u val {};
		param_t param{PARAM_INVALID};
	};

	/**
	 * Compare two modified parameter structures to determine ordering.
	 *
	 * This function is suitable for passing to qsort or bsearch.
	 */
	static int compareValues(const void *a, const void *b);

	/**
	 * Locate the modified parameter structure for a parameter, if it exists.
	 *
	 * @param param			The parameter being searched.
	 * @return			The structure holding the modified value, or
	 *				nullptr if the parameter has not been modified.
	*/
	param_wbuf_s *findChanged(param_t param);

	/**
	 * Obtain a pointer to the storage allocated for a parameter.
	 *
	 * @param param			The parameter whose storage is sought.
	 * @return			A pointer to the parameter value, or nullptr
	 *				if the parameter does not exist.
	*/
	const void *getParameterValuePointer(param_t param);

	// get parameter default value, caller is responsible for locking
	int getParameterDefaultValueInternal(param_t param, void *default_val);

	/**
	 * Save parameters to the default file.
	 * Note: this method requires a large amount of stack size!
	 *
	 * This function saves all parameters with non-default values.
	 *
	 * @return		Zero on success.
	 */
	int saveDefault();

	// internal parameter export, caller is responsible for locking
	int exportInternal(int fd, param_filter_func filter);

	int bsonImportCallback(bson_decoder_t decoder, bson_node_t node);
	static int importCallbackTrampoline(bson_decoder_t decoder, void *priv, bson_node_t node);
	int importFromFileDescriptorInternal(int fd);

	int verifyBsonExportCallback(bson_decoder_t decoder, bson_node_t node);
	static int verifyBsonExportTrampoline(bson_decoder_t decoder, void *priv, bson_node_t node);
	int verifyBsonExport(int fd);

	static int bsonDumpCallbackTrampoline(bson_decoder_t decoder, void *priv, bson_node_t node);
	int bsonDumpCallback(bson_decoder_t decoder, bson_node_t node);


	char *_param_default_file{nullptr};
	char *_param_backup_file{nullptr};

	px4::AtomicBitset<param_info_count> _params_active;  // params found
	px4::AtomicBitset<param_info_count> _params_changed; // params non-default
	px4::Bitset<param_info_count> _params_custom_default; // params with runtime default value
	px4::AtomicBitset<param_info_count> _params_unsaved;

	/** flexible array holding modified parameter values */
	UT_array *_param_values{nullptr};
	UT_array *_param_custom_default_values{nullptr};

	const UT_icd param_icd = {sizeof(param_wbuf_s), nullptr, nullptr, nullptr};

	unsigned int _param_instance{0};

	// the following implements an RW-lock using 2 semaphores (used as mutexes). It gives
	// priority to readers, meaning a writer could suffer from starvation, but in our use-case
	// we only have short periods of reads and writes are rare.
	px4_sem_t _param_sem; ///< this protects against concurrent access to _param_values
	int _reader_lock_holders{0};
	px4_sem_t _reader_lock_holders_lock; ///< this protects against concurrent access to reader_lock_holders

	perf_counter_t _export_perf{perf_alloc(PC_ELAPSED, "param: export")};
	perf_counter_t _find_count_perf{perf_alloc(PC_COUNT, "param: find")};
	perf_counter_t _get_count_perf{perf_alloc(PC_COUNT, "param: get")};
	perf_counter_t _set_perf{perf_alloc(PC_ELAPSED, "param: set")};

	px4_sem_t _param_sem_save; ///< this protects against concurrent param saves (file or flash access).
	///< we use a separate lock to allow concurrent param reads and saves.
	///< a param_set could still be blocked by a param save, because it
	///< needs to take the reader lock


	void Run() override;

	uORB::Publication<parameter_value_s> _param_response_pub{ORB_ID(parameter_value)};
	uORB::Publication<parameter_update_s> _parameter_update_pub{ORB_ID(parameter_update)};

	uORB::SubscriptionCallbackWorkItem _param_request_sub{this, ORB_ID(parameter_request)};

	uORB::SubscriptionData<actuator_armed_s> _armed_sub{ORB_ID(actuator_armed)};


	// autosaving variables
	hrt_abstime _last_autosave_timestamp{0};
	px4::atomic_bool _autosave_scheduled{false};
	bool _autosave_disabled{false};

	px4::atomic_bool _notify_scheduled{false};

};
