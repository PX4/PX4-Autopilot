

#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <systemlib/mavlink_log.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/input_rc.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/uORB.h>
#include <board_config.h>

extern "C" __EXPORT int loop_main(int argc, char *argv[]);

class Loop : public ModuleBase<Loop>, public px4::ScheduledWorkItem
{

public:
	Loop();
	~Loop()  override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void Run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	/* data */
	orb_advert_t _mavlink_log_pub = nullptr;
	perf_counter_t	_loop_perf;
	//uORB::Subscription _sub_rc_input{ORB_ID(input_rc)};
	//uORB::Subscription _sub_slave_rc_input{ORB_ID(slave_rc)};
	uORB::Subscription _sub_rc_input[4] {
		{ORB_ID(input_rc), 0},
		{ORB_ID(input_rc), 1},
		{ORB_ID(input_rc), 2},
		{ORB_ID(input_rc), 3},
	};

	uORB::Subscription _sub_distance_sensor[2] {
		{ORB_ID(distance_sensor), 0},
		{ORB_ID(distance_sensor), 1},

	};

	uORB::Subscription _sub_man {ORB_ID(manual_control_setpoint)};
	manual_control_setpoint_s man;
	//int _sub_rc_input[4];
	void set_signal_validity(const input_rc_s &input);
	void set_slave_signal_validity(const input_rc_s &slave_rc_input);
	bool input_signal_valid = false;
	bool slave_input_signal_valid = false;


	struct InputRC {
		input_rc_s input = {};
	} _inputs_rc[4];

	struct Distance {
		distance_sensor_s distance_sensor = {};
	} _distance_sensor[2];

};





Loop::Loop():
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, "loop"))
	// ModuleParams(nullptr),
	// ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	// _loop_perf(perf_alloc(PC_ELAPSED, "loop"))
{

}

Loop::~Loop()
{
	//orb_unsubscribe(_sub_rc_input);
	perf_free(_loop_perf);

}


bool Loop::init()
{
	//for (unsigned i = 0; i < 4 ; i++) {
// 	_sub_rc_input[i] = orb_subscribe_multi(ORB_ID(input_rc), i);
// 	}
	// initially run manually
	ScheduleDelayed(10);
	return true;
}


void Loop::Run()
{
	//bool rc_input_flag = false;
	bool _sub_distance_sensor_flag = false;

	//bool updated = false;
	//bool slave_rc_input_flag = false;

	//mavlink_log_critical(&_mavlink_log_pub, "Loop RUN ");

	perf_begin(_loop_perf);
	// backup schedule as a watchdog timeout
	ScheduleDelayed(10);

	for (unsigned i = 0; i < 2 ; i++) {
	// if (_sub_rc_input[i].updated()) {
	// 	rc_input_flag = _sub_rc_input[i].copy(&_inputs_rc[i].input);
	// 	if(rc_input_flag){
	// 	mavlink_log_critical(&_mavlink_log_pub,"Loop RUN %d ", _inputs_rc[i].input.values[0]);
	// }
	// }
	// if(_sub_man.updated()){
	// 	_sub_man.copy(&man);
	// 	mavlink_log_critical(&_mavlink_log_pub,"Loop manual %d ", man.mode_slot);

	// }


		if (_sub_distance_sensor[i].updated()) {
		_sub_distance_sensor_flag = _sub_distance_sensor[i].copy(&_inputs_rc[i].input);
		if(_sub_distance_sensor_flag){
		mavlink_log_critical(&_mavlink_log_pub,"Loop RUN %f ", (double)_distance_sensor[i].distance_sensor.current_distance);
	}
	}
	// if(_sub_man.updated()){
	// 	_sub_man.copy(&man);
	// 	mavlink_log_critical(&_mavlink_log_pub,"Loop manual %d ", man.mode_slot);

	// }

	}

	// if (_sub_slave_rc_input.update(&slave_rc_input)) {
	// 	slave_rc_input_flag = true;
	// }
	// if(slave_rc_input_flag){
	// set_slave_signal_validity(slave_rc_input);
	// }
	//PX4_INFO("Loop RUN IN IN");
	perf_end(_loop_perf);
}

int Loop::task_spawn(int argc, char *argv[])
{
Loop *instance = new Loop();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		PX4_INFO("loop task  test loc");
		if (instance->init()) {
			PX4_INFO("loop task ok");
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}


int Loop::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Loop::print_status()
{

	return 0;
}


int Loop::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
my loop test
The provided functionality includes:

### Implementation
It runs in its own thread and polls on the currently selected gyro topic.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("loop", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('h', "Start in HIL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


extern "C" __EXPORT int loop_main(int argc, char *argv[])
{
	return Loop::main(argc, argv);
}


void
Loop::set_signal_validity(const input_rc_s &input)
{
	// detect RC signal loss
	input_signal_valid = true;

	const bool rc_timeout = (hrt_absolute_time() - input.timestamp_last_signal) > hrt_abstime(
					0.5f* 1e6f);

	// check flags and require at least four channels to consider the signal valid
	if (input.rc_lost || input.rc_failsafe || input.channel_count < 4 || rc_timeout) {
		// signal is lost or not enough channels
		int aaa,bbb,ccc,ddd,eee;
		aaa = input.rc_lost;
		bbb = input.rc_failsafe;
		ccc= input.channel_count;
		ddd = rc_timeout;
		eee = input.values[0];
		input_signal_valid = false;
		mavlink_log_critical(&_mavlink_log_pub, "input_set.signal_valid = false;");
		mavlink_log_emergency(&_mavlink_log_pub, "%d %d %d %d %d", aaa, bbb, ccc,ddd,eee);

	}
}


void
Loop::set_slave_signal_validity(const input_rc_s &slave_rc_input)
{	mavlink_log_critical(&_mavlink_log_pub, "set_slave_signal_validity");
 	//PX4_INFO("set_slave_signal_validity(InputSlaveRCset &input_slave_set)");
	// detect RC signal loss
	slave_input_signal_valid = true;

	const bool rc_timeout = (hrt_absolute_time() - slave_rc_input.timestamp_last_signal) > hrt_abstime(
					0.5f * 1e6f);

	// check flags and require at least four channels to consider the signal valid
	if (slave_rc_input.rc_lost || slave_rc_input.rc_failsafe
	    || slave_rc_input.channel_count < 4 || rc_timeout) {
		// signal is lost or not enough channels
		slave_input_signal_valid = false;
		mavlink_log_critical(&_mavlink_log_pub, "slave_input_signal_valid = false;");
	}
}
