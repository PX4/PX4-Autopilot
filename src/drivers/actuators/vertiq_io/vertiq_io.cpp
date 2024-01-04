
#include "vertiq_io.hpp"

#include <px4_platform_common/log.h>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#define UART_BAUDRATE 115200
char VertiqIo::_telemetry_device[] {};

px4::atomic_bool VertiqIo::_request_telemetry_init{false};

VertiqIo::VertiqIo() :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::ttyS3)
{

}

VertiqIo::~VertiqIo()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

void VertiqIo::deinit_serial(){
	if (_uart_fd >= 0) {
		close(_uart_fd);
		_uart_fd = -1;
	}
}

int VertiqIo::init_serial(const char *uart_device){
	deinit_serial();
	_uart_fd = ::open(uart_device, O_RDWR | O_NOCTTY);

	if (_uart_fd < 0) {
		PX4_ERR("failed to open serial port: %s err: %d", uart_device, errno);
		return -errno;
	}

	PX4_INFO("Opened serial port successfully");

	return setBaudrate(UART_BAUDRATE);
}

int VertiqIo::setBaudrate(unsigned baud){
		int speed;

	switch (baud) {
	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

	default:
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_uart_fd, &uart_config);

	//
	// Input flags - Turn off input processing
	//
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
				 INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	//
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	uart_config.c_oflag = 0;

	//
	// No line processing
	//
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	/* no parity, one stop bit, disable flow control */
	uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		return -errno;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		return -errno;
	}

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
		return -errno;
	}

	return 0;
}

int VertiqIo::updateSerial(){
	if (_uart_fd < 0) {
		return -1;
	}

	// read from the uart. This must be non-blocking, so check first if there is data available
	int bytes_available = 0;
	int ret = ioctl(_uart_fd, FIONREAD, (unsigned long)&bytes_available);

	if (ret != 0){
		PX4_ERR("Reading error");
		return -1;
	}else{
		PX4_INFO("able to check the port for data and found %d bytes of data", bytes_available);
	}

	if(bytes_available > 0){
		const int buf_length = FRAME_SIZE;
		uint8_t buf[buf_length];

		int num_read = read(_uart_fd, buf, buf_length);
		for(uint8_t i = 0; i < num_read; i++){
			uint8_t char_to_write = buf[i];
			write(_uart_fd, &char_to_write, 1);
		}
	}

	return 1;
}

//called by our task_spawn function
bool VertiqIo::init()
{
	// //Open a serial port const
	// if(open_serial_port(serial_port, 115200) >= 0){
	// 	PX4_INFO("Opened serial port successfully");
	// }else{
	// 	PX4_INFO("Failed to open serial port");
	// }

	// init_serial(_telemetry_device);

	PX4_INFO("Init function of vertiqio");

	//Schedule to run every 100 ms
	//calls Run() every second
	ScheduleOnInterval(2500_us);

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
	if (_request_telemetry_init.load()) {
		PX4_INFO("Asked for serial init");
		init_serial(_telemetry_device);
		_request_telemetry_init.store(false);
	}

	updateSerial();

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

bool VertiqIo::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs, unsigned num_control_groups_updated){
	return true;
}


extern "C" __EXPORT int vertiq_io_main(int argc, char *argv[])
{
	return VertiqIo::main(argc, argv);
}
