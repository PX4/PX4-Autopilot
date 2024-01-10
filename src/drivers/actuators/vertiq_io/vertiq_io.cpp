
#include "vertiq_io.hpp"

#include <px4_platform_common/log.h>

px4::atomic_bool VertiqIo::_request_telemetry_init{false};
char VertiqIo::_telemetry_device[] {};

VertiqIo::VertiqIo() :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_serial_interface(NUM_CLIENTS),
	_prop_motor_control(0), //Initialize with a module ID of 0
	_brushless_drive(0) //Initialize with a module ID of 0

{
	_client_array[0] = &_prop_motor_control;
	_client_array[1] = &_brushless_drive;

	//Make sure we get the correct initial values for our parameters
	update_params();
}

VertiqIo::~VertiqIo()
{
	//Free our counters/timers
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

//called by our task_spawn function
bool VertiqIo::init()
{
	//Grab the number of IFCI control values the user wants to use
        _cvs_in_use = (uint8_t)_param_vertiq_number_of_cvs.get();

	//Grab the bitmask that we're going to use to decide who we get telemetry from
	_telem_bitmask = (uint16_t)_param_vertiq_telemetry_mask.get();

	//Go find the first and last positions of modules whose telemetry we need
	find_first_and_last_telemetry_positions();
	PX4_INFO("first telem: %d last telem: %d", _first_module_for_telem, _last_module_for_telem);

	find_next_motor_for_telemetry();
	find_next_motor_for_telemetry();
	find_next_motor_for_telemetry();
	find_next_motor_for_telemetry();

	//Make sure we get our thread into execution
	ScheduleNow();

	return true;
}

//This is the same as a while(1) loop. Gets called at a set interval, or
//is triggered by some uORB publication
void VertiqIo::Run()
{
	//Start the loop timer
	//Increment our loop counter
	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	//If we should leave, then clean up our mess and get out
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// telemetry device update request?
	//This happens whenever the module is started
	if (_request_telemetry_init.load()) {
		// PX4_INFO("Asked for serial init");
		_serial_interface.init_serial(_telemetry_device, _param_vertiq_baud.get());
		_request_telemetry_init.store(false);
	}

	//Handle IQUART reception and transmission
	handle_iquart();

	if(_telem_bitmask){
		update_telemetry();
	}

	//Get the most up to date version of our parameters
	update_params();

	//Make sure we also update the mixing output to get the most up to date configuration
	_mixing_output.update();
	_mixing_output.updateSubscriptions(true);

	//stop our timer
	perf_end(_loop_perf);
}

int VertiqIo::task_spawn(int argc, char *argv[])
{
	VertiqIo *instance = new VertiqIo();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
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

int VertiqIo::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);

	_mixing_output.printStatus();
	return 0;
}

int VertiqIo::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_USAGE_NAME("vertiq_io", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");

	PRINT_MODULE_USAGE_COMMAND_DESCR("telemetry", "Enable Telemetry on a UART");
	PRINT_MODULE_USAGE_ARG("<device>", "UART device", false);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int VertiqIo::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	if (!strcmp(verb, "telemetry")) {
		if (argc > 1) {
			// telemetry can be requested before the module is started
			strncpy(_telemetry_device, argv[1], sizeof(_telemetry_device) - 1);
			_telemetry_device[sizeof(_telemetry_device) - 1] = '\0';
			_request_telemetry_init.store(true);
		}

		return 0;
	}

	return print_usage("unknown command");
}

void VertiqIo::update_telemetry(){
	bool got_reply = false;

	//Get the current time to check for timeout
	hrt_abstime timestamp_for_timeout = hrt_absolute_time();

	//We timed out for this request if the time since the last request going out is greater than our timeout period
	bool timed_out = (timestamp_for_timeout - _time_of_last_telem_request) > _telem_timeout;

	//We got a telemetry response
	if(_motor_interface.telemetry_.IsFresh()){
		//grab the data
		_motor_interface.telemetry_.get_reply();
		got_reply = true;
	}

	if(got_reply || timed_out){//} || (hrt_elapsed_time(&timestamp_for_timeout) > _telem_timeout)){
		//update the telem target
		find_next_motor_for_telemetry();
		_time_of_last_telem_request = hrt_absolute_time();
		PX4_INFO("Target telem is now %d", _current_telemetry_target_module_id);
	}
}

void VertiqIo::find_next_motor_for_telemetry(){
	//If the current telemetry is the highest available, wrap around to the first one
	//If we're below the max, find the next available.

	//The bitmask shifted by the number of telemetry targets we've alredy hit
	uint16_t next_telem_target_mask = _telem_bitmask >> (_current_telemetry_target_module_id + 1);

	//The bit position of the next telemetry target
	uint16_t next_telem_target_position = _current_telemetry_target_module_id  + 1;

	//Keep trying to find the next value until you're out of bits. if you run out of bits, your next telem is the lowest value that we saved before
	while(next_telem_target_mask > 0){
		if(next_telem_target_mask & 0x0001){
			_current_telemetry_target_module_id = next_telem_target_position;
			return; //get out
		}

		//keep trying
		next_telem_target_position++;
		next_telem_target_mask = next_telem_target_mask >> 1;
	}

	//We didn't find anyone new. Go back to the start.
	_current_telemetry_target_module_id = _first_module_for_telem;
}

void VertiqIo::find_first_and_last_telemetry_positions(){
	uint16_t shift_val = 0x0001;
	bool found_first = false;

	//Go through from 0 to the max number of CVs we can have, and determine the first and last place we have a 1
	for(uint8_t i = 0; i < MAX_SUPPORTABLE_IFCI_CVS; i++){
		if(shift_val & _telem_bitmask){

			//We only want to set the lowest value once
			if(!found_first){
				_first_module_for_telem = i;

				//Also initialize the current target
				_current_telemetry_target_module_id = _first_module_for_telem;
				found_first = true;
			}

			//Keep updating the last module for every time we hit a 1
			_last_module_for_telem = i;
		}

		shift_val = shift_val << 1;
	}
}

void VertiqIo::handle_iquart(){
	//Add a get message to our transmission queue
	_brushless_drive.obs_velocity_.get(*_serial_interface.get_iquart_interface());

	//Update our serial rx
	_serial_interface.process_serial_rx(&_motor_interface, _client_array);

	//Update our serial tx
	_serial_interface.process_serial_tx();

	//Read the velo we got
	if (_brushless_drive.obs_velocity_.IsFresh()) {
		// PX4_INFO("Got velocity response %f", (double)_brushless_drive.obs_velocity_.get_reply());
	}
}

void VertiqIo::update_params()
{
	updateParams();
}

bool VertiqIo::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
			     unsigned num_control_groups_updated)
{
	//We already get a mixer value from [0, 65535]. We can send that right to the motor, and let the input parser handle
	//conversions
	_motor_interface.BroadcastPackedControlMessage(*_serial_interface.get_iquart_interface(), outputs, _cvs_in_use, _current_telemetry_target_module_id);

	return true;
}


extern "C" __EXPORT int vertiq_io_main(int argc, char *argv[])
{
	return VertiqIo::main(argc, argv);
}
