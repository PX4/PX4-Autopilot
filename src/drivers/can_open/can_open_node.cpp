/****************************************************************************
 *
 *   Copyright (c) 2014-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#include <stdio.h>

#include "can_open_node.hpp"

using namespace time_literals;

CanOpenNode *CanOpenNode::_instance{nullptr};

can_open_node_error_s CanOpenNode::_can_open_node_error[2]{0};

uint64_t ODRecord::_current_timestamp;

CanOpenNode::CanOpenNode(uint8_t node_id, int32_t bitrate) :
	px4::ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::can), ModuleParams(nullptr)
{
	_CO_instance = CO_new(NULL, &_heap_memory_used);

	if (_CO_instance == NULL) {
		PX4_ERR("Error: Can't allocate memory");

	} else {
		PX4_DEBUG("Allocated %lu bytes for CANopen objects", _heap_memory_used);
	}

	_node_id = node_id;
	_bitrate = bitrate;
	_run_end = hrt_absolute_time();
}

CanOpenNode::~CanOpenNode()
{
	if (_instance) {
		/* tell the task we want it to go away */
		_task_should_exit.store(true);
		ScheduleNow();

		unsigned i = 1000;

		do {
			/* Wait for it to exit or timeout */
			usleep(5000);

			if (--i == 0) {
				PX4_ERR("Failed to Stop Task - reboot needed");
				break;
			}

		} while (_instance);
	}
	perf_free(_cycle_perf);
	perf_free(_interval_perf);

	PX4_DEBUG("CANopenNode finished");
}

int CanOpenNode::start(uint8_t node_id, int32_t bitrate)
{
	if (_instance != nullptr) {
		PX4_WARN("Already started");
		return -1;
	}

	_instance = new CanOpenNode(node_id, bitrate);

	if (_instance == nullptr) {
		PX4_ERR("Out of memory");
		return -1;
	}

	_instance->ScheduleOnInterval(SCHEDULE_INTERVAL);

	return PX4_OK;
}

void CanOpenNode::can_open_error_cb(const uint16_t ident, const uint16_t error_code, const uint8_t error_register, const uint8_t error_bit, const uint32_t info_code)
{
	static constexpr uint16_t EMERGENCY_COB_ID = 0x80;
	uint16_t index = 0;

	// an ident of 0 means the emergency came from this device.
	// Otherwise, subtract the emergency message COBID off ident to get the
	// node the emergency message came from.
	if(ident != 0) {
		index = ident - EMERGENCY_COB_ID;
		if(index != 1) {
			PX4_ERR("Error from unknown CANOpen node: %d", index);
			return;
		}
	}
	_can_open_node_error[index].error_code = error_code;
	_can_open_node_error[index].error_register = error_register;
	_can_open_node_error[index].error_bit = error_bit;
	_can_open_node_error[index].info_code = info_code;
}

void CanOpenNode::init()
{
	CO_ReturnError_t err;

	// these are unused.  They are here as a hack because
	// app_programStart is expecting them.
	uint16_t bitrate;
	uint8_t node_id;

	if (_CO_instance == nullptr) {
		PX4_ERR("init called with CO_instance nullptr");
		return;
	}

	/* CANopen communication reset - initialize CANopen objects *******************/
	PX4_DEBUG("CANopenNode - Reset communication...");

	/* Execute optional external application code */
	err = app_programStart(&bitrate, &node_id, &_err_info_app);

	if (err != CO_ERROR_NO) {
		PX4_ERR("app_programStart error: 0x%x", err);
		return;
	}

	/* initialize CANopen */
	err = CO_CANinit(_CO_instance, NULL, _bitrate / 1000);

	if (err != CO_ERROR_NO) {
		PX4_ERR("Error: CAN initialization failed: %d", err);
		return;
	}

	err = CO_CANopenInit(_CO_instance,      /* CANopen object */
			     NULL,                      /* alternate NMT */
			     NULL,                      /* alternate em */
			     OD,                        /* Object dictionary */
			     NULL,    	                /* Optional OD_statusBits */
			     (CO_NMT_control_t)(CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG |
						CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION),       /* CO_NMT_control_t */
			     500,       /* firstHBTime_ms */
			     1000,      /* SDOserverTimeoutTime_ms */
			     500,       /* SDOclientTimeoutTime_ms */
			     false,     /* SDOclientBlockTransfer */
			     _node_id,
			     &_err_info_stack);

	if (err != CO_ERROR_NO) {
		if (err == CO_ERROR_OD_PARAMETERS) {
			PX4_ERR("Error: Object Dictionary entry 0x%lX", _err_info_stack);

		} else {
			PX4_ERR("Error: CANopen initialization failed: %d", err);
		}

		return;
	}

	CO_EM_initCallbackRx(_CO_instance->em, can_open_error_cb);

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
	uint8_t storageEntriesCount = sizeof(_storage_entries) / sizeof(_storage_entries[0]);

	_storage_entries[0].addr = &OD_PERSIST_COMM;
	_storage_entries[0].len = sizeof(OD_PERSIST_COMM);
	_storage_entries[0].subIndexOD = 2;
	_storage_entries[0].attr = CO_storage_cmd | CO_storage_auto | CO_storage_restore;
	//_storage_entries[0].attr = CO_storage_cmd | CO_storage_restore;
	strncpy(_storage_entries[0].filename, PX4_STORAGEDIR"/canopen/od_comm.persist", CO_STORAGE_PATH_MAX);
	err = CO_storage_nuttx_init(&_storage,
				    _CO_instance->CANmodule,
				    OD_ENTRY_H1010_storeParameters,
				    OD_ENTRY_H1011_restoreDefaultParameters,
				    _storage_entries,
				    storageEntriesCount,
				    &_storage_error);

	if (err != CO_ERROR_NO) {
		PX4_ERR("CO_storage_nuttx_init error: %d, _storage_error: %lu", err, _storage_error);
	}

	if (_storage_error != 0) {
		CO_errorReport(_CO_instance->em, CO_EM_NON_VOLATILE_MEMORY,
			       CO_EMC_HARDWARE, _storage_error);
	}

#endif

	err = CO_CANopenInitPDO(_CO_instance, _CO_instance->em, OD, _node_id, &_err_info_stack);

	if (err != CO_ERROR_NO) {
		if (err == CO_ERROR_OD_PARAMETERS) {
			PX4_ERR("Error: Object Dictionary entry 0x%lX", _err_info_stack);

		} else {
			PX4_ERR("Error: PDO initialization failed: %d", err);
		}

		return;
	}

	/* Configure CANopen callbacks, etc */
	if (_CO_instance->nodeIdUnconfigured) {
		PX4_DEBUG("CANopenNode - Node-id not initialized");
	}

	/* start CAN */
	CO_CANsetNormalMode(_CO_instance->CANmodule);
	CO_port_set_baud_rate(_CO_instance->CANmodule, _bitrate);

	app_communicationReset(_CO_instance);

	_run_end = hrt_absolute_time();
	_initialized = true;
	PX4_DEBUG("CANopenNode - Init Complete");
}

void CanOpenNode::CO_high_pri_work(uint32_t time_difference_us)
{
	bool_t syncWas = false;

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
	syncWas = CO_process_SYNC(_CO_instance, time_difference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
	CO_process_RPDO(_CO_instance, syncWas, time_difference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
	CO_process_TPDO(_CO_instance, syncWas, time_difference_us, NULL);
#endif

}

void CanOpenNode::CO_medium_pri_work(uint32_t time_difference_us)
{
	app_programRt(_CO_instance, time_difference_us);
#if defined(CANOPEN_EXAMPLE_APPLICATION)
	_app.update();
#endif
}

void CanOpenNode::CO_low_pri_work(uint32_t time_difference_us)
{
#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
	uint32_t mask = CO_storage_nuttx_auto_process(&_storage, false);

	if (mask != _storage_error_prev && !_CO_instance->nodeIdUnconfigured) {
		if (mask != 0) {
			CO_errorReport(_CO_instance->em, CO_EM_NON_VOLATILE_AUTO_SAVE,
				       CO_EMC_HARDWARE, mask);

		} else {
			CO_errorReset(_CO_instance->em, CO_EM_NON_VOLATILE_AUTO_SAVE, 0);
		}
	}

	_storage_error_prev = mask;
#endif
	app_programAsync(_CO_instance, time_difference_us);
}

void CanOpenNode::Run()
{
	static int16_t start_counter = 0;
	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	_run_end = hrt_absolute_time();

	if (_instance != nullptr && !_initialized) {
		// delay a bit of time before initializing
		// things to let the bus settle
		if(start_counter < 100) {
			start_counter++;
		} else {
			init();
			start_counter = 0;
		}
		perf_end(_cycle_perf);
		return;
	}

	ODRecord::set_timestamp(_run_end);

	_medium_pri_timer += SCHEDULE_INTERVAL;
	_low_pri_timer += SCHEDULE_INTERVAL;

	CO_driver_receive(_CO_instance->CANmodule);
	CO_high_pri_work(SCHEDULE_INTERVAL);

	if (_medium_pri_timer >= 10_ms) {
		CO_NMT_reset_cmd_t reset;

		/* CANopen process */
		reset = CO_process(_CO_instance, false, _medium_pri_timer, NULL);

		if (reset != CO_RESET_NOT) {
			if (reset == CO_RESET_COMM || reset == CO_RESET_APP) {
				_initialized = false;
				PX4_DEBUG("CanOpen - resetting stack");

			} else {
				PX4_ERR("Run() - reset: 0x%x", reset);
			}
		}

		CO_medium_pri_work(_medium_pri_timer);
		_medium_pri_timer = 0;
	}

	if (_low_pri_timer >= 5_s) {
		CO_low_pri_work(_low_pri_timer);
		_low_pri_timer = 0;
	}

	if (_task_should_exit.load()) {
		app_programEnd();
#if defined(CANOPEN_EXAMPLE_APPLICATION)
		_app.shutdown();
#endif
		ScheduleClear();

		if (_initialized) {
#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
			CO_storage_nuttx_auto_process(&_storage, true);
#endif
			CO_delete(_CO_instance);
			_initialized = false;
		}

		_instance = nullptr;
	}

	perf_end(_cycle_perf);
}

void CanOpenNode::print_info()
{
	PX4_INFO("Heap: %lu, Node Id: %u, Bitrate %ld", _heap_memory_used, _node_id, _bitrate);
	PX4_INFO("Error Stack: 0x%lx, Error App: 0x%lx", _err_info_stack, _err_info_app);
	PX4_INFO("Storage error: 0x%lx, Storage error prev: 0x%lx", _storage_error, _storage_error_prev);

	for(uint8_t i = 0; i < 2; i++) {
		PX4_INFO("error code 0x%x, error register 0x%x, error bit 0x%x, info code 0x%lx",
			 _can_open_node_error[i].error_code,
			 _can_open_node_error[i].error_register,
			 _can_open_node_error[i].error_bit,
			 _can_open_node_error[i].info_code);
	}

	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);
}

static void print_usage()
{
	PX4_INFO("usage: \n"
		 "\tcanopen {start|status|stop|start_sensors|stop_sensors}");
}

extern "C" __EXPORT int canopen_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		// CAN bitrate
		int32_t bitrate = 1000000;
		param_get(param_find("CANOPEN_BITRATE"), &bitrate);

		// Node ID
		int32_t node_id = 8;
		param_get(param_find("CANOPEN_NODE_ID"), &node_id);

		// Start
		PX4_INFO("Node ID %ld, bitrate %ld", node_id, bitrate);
		return CanOpenNode::start(node_id, bitrate);
	}

	/* commands below require the app to be started */
	CanOpenNode *const inst = CanOpenNode::instance();

	if (!inst) {
		PX4_ERR("application not running");
		return 1;
	}

	if (!strcmp(argv[1], "status") || !strcmp(argv[1], "info")) {
		inst->print_info();
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		delete inst;
		return 0;
	}

	print_usage();
	return 1;
}
