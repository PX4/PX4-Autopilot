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

/**
 * @file serial_socat.cpp
 *
 * Serial to TCP bridge for direct Ethernet connections.
 *
 * PARAMETER-BASED CONFIGURATION (FMUv6s):
 * =======================================
 *
 * To enable automatic startup on boot, configure via parameters:
 *
 * GPS Port (ttyS1, TCP 5760):
 *   param set SOCAT_GPS_BAUD 115200
 *
 * TELEM2 Port (ttyS4, TCP 5761):
 *   param set SOCAT_TEL2_BAUD 57600
 *
 * ESC Port (ttyS5, TCP 5762):
 *   param set SOCAT_ESC_BAUD 115200
 *
 * RC Port (ttyS6, TCP 5763):
 *   param set SOCAT_RC_BAUD 115200
 *
 * Set to 0 to disable a port. Then save and reboot:
 *   param save
 *   reboot
 *
 * MANUAL USAGE:
 * =============
 *
 * Start manually:
 *   serial_socat start -p 5760 -d /dev/ttyS6 -b 115200
 *
 * Connect from Linux:
 *   socat /dev/ttyUSB0,raw,echo=0,b115200 TCP:px4_ip:5760,retry,forever
 *
 * Or using netcat:
 *   nc px4_ip 5760
 */

#include <px4_platform_common/px4_config.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/Serial.hpp>

using device::SerialConfig::ByteSize;
using device::SerialConfig::Parity;
using device::SerialConfig::StopBits;
using device::SerialConfig::FlowControl;

static constexpr int TASK_STACK_SIZE   = PX4_STACK_ADJUSTED(1224 * 2);
static constexpr int THREAD_STACK_SIZE = PX4_STACK_ADJUSTED(1224 * 2);
static constexpr int MAX_INSTANCES = 10;

class SERIALSOCAT
{
public:

	SERIALSOCAT(int tcp_port, const char *internal_path, unsigned baudrate, bool swap_rxtx, bool single_wire, bool two_stop_bits);
	~SERIALSOCAT();

	static int task_spawn(int argc, char *argv[]);
	static SERIALSOCAT *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);
	void run();


	void thread_run();

	void thread_start();
	void thread_stop();
	static void *trampoline(void *context);

	// Getters for status reporting
	int get_tcp_port() const { return _tcp_port; }
	const char *get_serial_port() const { return _int_path; }
	unsigned get_baudrate() const { return _baudrate; }
	bool is_connected() const { return _client_fd >= 0; }
	uint64_t get_bytes_received() const { return _bytes_received.load(); }
	uint64_t get_bytes_transmitted() const { return _bytes_transmitted.load(); }
	const char *get_remote_addr() const
	{
		static char addr_str[INET_ADDRSTRLEN + 6]; // IP + : + port

		if (_client_fd >= 0) {
			inet_ntop(AF_INET, &_client_addr.sin_addr, addr_str, INET_ADDRSTRLEN);
			int len = strlen(addr_str);
			snprintf(addr_str + len, sizeof(addr_str) - len, ":%d", ntohs(_client_addr.sin_port));
			return addr_str;
		}

		return "none";
	}

private:
	device::Serial _serial_port; ///< serial port interface
	int        _fd_listen{-1};    ///< the TCP listening socket
	int        _client_fd{-1};    ///< the connected TCP client socket
	unsigned   _baudrate{0};      ///< baudrate passed
	int        _tcp_port{0};      ///< TCP port number
	char       _int_path[20] {}; ///< internal device / serial port path
	bool       _swap_rxtx{false}; ///< swap RX and TX pins
	bool       _single_wire{false}; ///< single wire (half-duplex) mode
	bool       _two_stop_bits{false}; ///< use two stop bits instead of one
	struct sockaddr_in _client_addr; ///< connected client address
	px4::atomic_bool _should_exit{false}; ///< flag to signal exit
	uint8_t _ser_buf[256];
	uint8_t _tcp_buf[256];

	// Statistics
	px4::atomic<uint64_t> _bytes_received{0}; ///< bytes received from TCP
	px4::atomic<uint64_t> _bytes_transmitted{0}; ///< bytes transmitted to TCP

	bool should_exit() const { return _should_exit.load(); }
	void request_stop() { _should_exit.store(true); }

#if defined(DEBUG_BUILD)
	enum dbg_t {
		NONE = 0,
		INT   = 1,
		EXT  = 2,
		BAUD = 4,
	};

	enum dbg_t _debug_level {BAUD};

#endif

	pthread_t _thread{0};      ///< worker task id
	px4::atomic_bool _thread_should_exit{false};

	int createTCPSocket();
	void acceptClient();
	void dump(const char *dirin, const char *dirout, int read, int written,
		  char *buffer);
};

// Global array to track instances
static SERIALSOCAT *g_instances[MAX_INSTANCES] = {nullptr};
static pthread_mutex_t g_instances_mutex = PTHREAD_MUTEX_INITIALIZER;

static int register_instance(SERIALSOCAT *instance)
{
	pthread_mutex_lock(&g_instances_mutex);

	for (int i = 0; i < MAX_INSTANCES; i++) {
		if (g_instances[i] == nullptr) {
			g_instances[i] = instance;
			pthread_mutex_unlock(&g_instances_mutex);
			return i;
		}
	}

	pthread_mutex_unlock(&g_instances_mutex);
	return -1;
}

static void unregister_instance(SERIALSOCAT *instance)
{
	pthread_mutex_lock(&g_instances_mutex);

	for (int i = 0; i < MAX_INSTANCES; i++) {
		if (g_instances[i] == instance) {
			g_instances[i] = nullptr;
			break;
		}
	}

	pthread_mutex_unlock(&g_instances_mutex);
}



void SERIALSOCAT::dump(const char *dirin, const char *dirout, int read, int written,
		       char *buffer)
{
#if defined(DEBUG_BUILD)
	enum dbg_t mgtype =  dirin[0] == 't' ? EXT : INT;

	if ((_debug_level & mgtype) && read > 0) {
		fprintf(stderr, "%s %d bytes read\n", dirin, read);
		fprintf(stderr, "%s %d bytes written\n", dirout, written);

		for (int i = 0; i < read; i++) {
			fprintf(stderr, "|%X", buffer[i]);
		}

		fprintf(stderr, "\n");
	}

#endif
}

void SERIALSOCAT::acceptClient()
{
	struct sockaddr_in client_addr;
	socklen_t client_len = sizeof(client_addr);

	int client_fd = accept(_fd_listen, (struct sockaddr *)&client_addr, &client_len);

	if (client_fd < 0) {
		if (errno != EAGAIN && errno != EWOULDBLOCK) {
			PX4_ERR("Accept failed: %s", strerror(errno));
		}
		return;
	}

	// Close any existing client connection
	if (_client_fd >= 0) {
		close(_client_fd);
	}

	_client_fd = client_fd;
	_client_addr = client_addr;

	char ip_str[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, &client_addr.sin_addr, ip_str, INET_ADDRSTRLEN);
	PX4_INFO("TCP client connected from %s:%d", ip_str, ntohs(client_addr.sin_port));
}

void SERIALSOCAT::thread_run()
{
	px4_prctl(PR_SET_NAME, "serial_socat-tcp->uart", px4_getpid());

	do {
		if (_fd_listen < 0 || !_serial_port.isOpen()) {
			// Not ready yet, sleep and retry
			px4_usleep(100000);
			continue;
		}

		// If no client connected, try to accept one
		if (_client_fd < 0) {
			pollfd fds[1];
			fds[0].fd = _fd_listen;
			fds[0].events = POLLIN;

			int ret = poll(fds, 1, 1000); // Wait up to 1 second for connection

			if (ret > 0 && (fds[0].revents & POLLIN)) {
				acceptClient();
			}

			continue;
		}

		// We have a client, poll for data
		pollfd fds[1];
		fds[0].fd = _client_fd;
		fds[0].events = POLLIN;

		int ret = poll(fds, 1, 100);

		if (ret > 0) {
			if (fds[0].revents & POLLIN) {
				int nread = recv(_client_fd, &_tcp_buf, sizeof(_tcp_buf), 0);

				if (nread > 0) {
					ssize_t nwrite = _serial_port.write((uint8_t *)&_tcp_buf, nread);
					dump("tcp", "uart", nread, nwrite, (char *)_tcp_buf);

					// Update statistics
					if (nwrite > 0) {
						_bytes_received.fetch_add(nwrite);
					}

				} else if (nread == 0) {
					// Client disconnected
					char ip_str[INET_ADDRSTRLEN];
					inet_ntop(AF_INET, &_client_addr.sin_addr, ip_str, INET_ADDRSTRLEN);
					PX4_INFO("TCP client %s:%d disconnected", ip_str, ntohs(_client_addr.sin_port));
					close(_client_fd);
					_client_fd = -1;

				} else if (errno != EAGAIN && errno != EWOULDBLOCK) {
					PX4_ERR("TCP recv error: %s", strerror(errno));
					close(_client_fd);
					_client_fd = -1;
				}
			}

			if (fds[0].revents & (POLLERR | POLLHUP)) {
				// Connection error or hangup
				PX4_INFO("TCP client connection closed (POLLERR/POLLHUP)");
				close(_client_fd);
				_client_fd = -1;
			}
		}

	} while (!_thread_should_exit.load());
}

void SERIALSOCAT::thread_start()
{
	pthread_attr_t loop_attr;
	pthread_attr_init(&loop_attr);

	struct sched_param param;
	(void)pthread_attr_getschedparam(&loop_attr, &param);
	param.sched_priority = SCHED_PRIORITY_SLOW_DRIVER - 1;
	(void)pthread_attr_setschedparam(&loop_attr, &param);

	pthread_attr_setstacksize(&loop_attr, THREAD_STACK_SIZE);
	pthread_create(&_thread, &loop_attr, [](void *context) -> void* {
		reinterpret_cast<SERIALSOCAT *>(context)->thread_run();
		return nullptr;
	}, this);
	pthread_attr_destroy(&loop_attr);
}

void SERIALSOCAT::thread_stop()
{
	_thread_should_exit.store(true);
	pthread_join(_thread, nullptr);
}

SERIALSOCAT::SERIALSOCAT(int tcp_port, const char *int_path, unsigned baudrate, bool swap_rxtx, bool single_wire, bool two_stop_bits) :
	_serial_port(int_path, baudrate,
		     ByteSize::EightBits,
		     Parity::None,
		     two_stop_bits ? StopBits::Two : StopBits::One,
		     FlowControl::Disabled),
	_baudrate(baudrate),
	_tcp_port(tcp_port),
	_swap_rxtx(swap_rxtx),
	_single_wire(single_wire),
	_two_stop_bits(two_stop_bits)
{
	strncpy(_int_path, int_path, sizeof(_int_path) - 1);
	_int_path[sizeof(_int_path) - 1] = '\0';

	memset(&_client_addr, 0, sizeof(_client_addr));
}

SERIALSOCAT::~SERIALSOCAT()
{
	thread_stop();

	if (_serial_port.isOpen()) {
		_serial_port.close();
	}

	if (_client_fd >= 0) {
		close(_client_fd);
	}

	if (_fd_listen >= 0) {
		close(_fd_listen);
	}

	unregister_instance(this);
}


int SERIALSOCAT::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION("Pass data from TCP stream to a serial device.\n"
				 "\n"
				 "This can be used to bridge a TCP connection to a serial port.\n"
				 "Listens on a TCP port and forwards data bidirectionally.\n"
				);

	PRINT_MODULE_USAGE_NAME_SIMPLE("serial_socat", "command");
	PRINT_MODULE_USAGE_PARAM_INT('p', 5760, 0, 65535, "TCP port to listen on", true);
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Internal device path", false);
	PRINT_MODULE_USAGE_PARAM_INT('b', 115200, 0, 3000000, "Baudrate", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('x', "Swap RX/TX pins", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('s', "Single-wire (half-duplex) mode", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('2', "Use two stop bits instead of one", true);
	return 0;
}

int SERIALSOCAT::createTCPSocket()
{
	struct sockaddr_in serv_addr;

	// Create TCP socket
	int sock = socket(AF_INET, SOCK_STREAM, 0);

	if (sock < 0) {
		PX4_ERR("Failed to create TCP socket: %s", strerror(errno));
		return -1;
	}

	// Set socket options to reuse address
	int opt = 1;

	if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt)) < 0) {
		PX4_WARN("Failed to set SO_REUSEADDR: %s", strerror(errno));
		// Don't fail on this, continue anyway
	}

	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(_tcp_port);

	if (bind(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
		PX4_ERR("Failed to bind to TCP port %d: %s", _tcp_port, strerror(errno));
		close(sock);
		return -1;
	}

	// Listen for incoming connections (backlog of 1)
	if (listen(sock, 1) < 0) {
		PX4_ERR("Failed to listen on TCP port %d: %s", _tcp_port, strerror(errno));
		close(sock);
		return -1;
	}

	PX4_INFO("TCP socket listening on port %d", _tcp_port);
	return sock;
}

void
SERIALSOCAT::run()
{
	px4_prctl(PR_SET_NAME, "serial_socat-uart->tcp", px4_getpid());

	// Create TCP socket once
	if (_fd_listen < 0) {
		_fd_listen = createTCPSocket();

		if (_fd_listen < 0) {
			PX4_ERR("Failed to create TCP socket, exiting");
			return;
		}
	}

	// Open and configure serial port using Serial class
	if (!_serial_port.isOpen()) {
		if (!_serial_port.open()) {
			PX4_ERR("Failed to open serial port %s", _int_path);
			close(_fd_listen);
			return;
		}

		// Configure special modes
		if (_swap_rxtx) {
			_serial_port.setSwapRxTxMode();
		}

		if (_single_wire) {
			_serial_port.setSingleWireMode();
		}
	}

	// Start thread for TCP->Serial direction
	if (_thread == 0) {
		thread_start();
	}

	while (!should_exit()) {

		// If no client connected, wait for one
		if (_client_fd < 0) {
			px4_usleep(100000); // Sleep 100ms and check again
			continue;
		}

		// Block until at least 1 byte is available (or timeout)
		// This uses internal semaphores, no busy-wait needed
		constexpr size_t min_bytes = 1;
		constexpr int timeout_ms = 100; // Wake up periodically to check should_exit()

		ssize_t nread = _serial_port.readAtLeast(_ser_buf, sizeof(_ser_buf), min_bytes, timeout_ms);

		if (nread > 0) {
			// Only send if we have a connected client
			if (_client_fd >= 0) {
				ssize_t nwrite = send(_client_fd, _ser_buf, nread, 0);

				if (nwrite < 0) {
					if (errno == EPIPE || errno == ECONNRESET) {
						// Client disconnected
						PX4_INFO("TCP client disconnected (send failed)");
						close(_client_fd);
						_client_fd = -1;
					} else if (errno != EAGAIN && errno != EWOULDBLOCK) {
						PX4_ERR("TCP send error: %s", strerror(errno));
					}

				} else {
					dump("uart", "tcp", nread, nwrite, (char *)_ser_buf);
					// Update statistics
					_bytes_transmitted.fetch_add(nwrite);
				}
			}
		} else if (nread < 0) {
			// Error occurred (not timeout)
			PX4_ERR("Serial read error: %d", nread);
			break;
		}
		// If nread == 0, it was a timeout, loop will check should_exit()
	}

	// Cleanup
	if (_thread != 0) {
		thread_stop();
	}

	if (_serial_port.isOpen()) {
		_serial_port.close();
	}

	if (_client_fd >= 0) {
		close(_client_fd);
		_client_fd = -1;
	}

	if (_fd_listen >= 0) {
		close(_fd_listen);
		_fd_listen = -1;
	}
}

int SERIALSOCAT::task_spawn(int argc, char *argv[])
{
	SERIALSOCAT *instance = instantiate(argc, argv);

	if (!instance) {
		PX4_ERR("Failed to instantiate serial_socat");
		return PX4_ERROR;
	}

	int instance_id = register_instance(instance);

	if (instance_id < 0) {
		PX4_ERR("Too many instances (max %d)", MAX_INSTANCES);
		delete instance;
		return PX4_ERROR;
	}

	// Start running directly in this context
	instance->run();

	// Cleanup after run() completes
	delete instance;

	return PX4_OK;
}

SERIALSOCAT *SERIALSOCAT::instantiate(int argc, char *argv[])
{
	int tcp_port = 5760;
	const char *int_device = nullptr;
	int baudrate = 115200;
	bool swap_rxtx = false;
	bool single_wire = false;
	bool two_stop_bits = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "2b:d:p:sx", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		case '2':
			two_stop_bits = true;
			break;

		case 's':
			single_wire = true;
			break;

		case 'x':
			swap_rxtx = true;
			break;

		case 'b':
			baudrate = strtol(myoptarg, nullptr, 0);
			break;

		case 'p':
			tcp_port = strtol(myoptarg, nullptr, 0);
			break;

		case 'd':
			int_device = myoptarg;
			break;

		}
	}

	SERIALSOCAT *serial_socat = nullptr;
	bool rok =  int_device && (access(int_device, R_OK | W_OK) == 0);

	if (rok) {
		serial_socat = new SERIALSOCAT(tcp_port, int_device, baudrate, swap_rxtx, single_wire, two_stop_bits);

	} else {
		PX4_ERR("Invalid internal device (-d) %s", int_device ? int_device  : "");
	}

	return serial_socat;
}


extern "C" __EXPORT int serial_socat_main(int argc, char *argv[]);

int serial_socat_main(int argc, char *argv[])
{
	if (argc < 2) {
		SERIALSOCAT::print_usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		// Spawn in background task
		char task_name[32];
		static int next_instance = 0;
		snprintf(task_name, sizeof(task_name), "socat_%d", next_instance++);

		px4_task_t task_id = px4_task_spawn_cmd(
					     task_name,
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_SLOW_DRIVER,
					     TASK_STACK_SIZE,
		[](int task_argc, char *task_argv[]) -> int {
			return SERIALSOCAT::task_spawn(task_argc, task_argv);
		},
		argv + 1);  // Skip "start" argument

		if (task_id < 0) {
			PX4_ERR("Failed to start serial_socat: %d", errno);
			return 1;
		}

		return 0;

	} else if (!strcmp(argv[1], "stop")) {
		PX4_INFO("Stop not implemented for multi-instance, instances will stop when signaled");
		return 0;

	} else if (!strcmp(argv[1], "status")) {
		pthread_mutex_lock(&g_instances_mutex);
		int count = 0;

		PX4_INFO("serial_socat instances:");
		PX4_INFO("%-12s %-10s %-10s %-20s %-12s %-12s",
			 "Serial Port", "Baudrate", "TCP Port", "Remote Client", "RX Bytes", "TX Bytes");
		PX4_INFO("--------------------------------------------------------------------------------------------");

		for (int i = 0; i < MAX_INSTANCES; i++) {
			if (g_instances[i] != nullptr) {
				SERIALSOCAT *inst = g_instances[i];
				PX4_INFO("%-12s %-10u %-10d %-20s %-12llu %-12llu",
					 inst->get_serial_port(),
					 inst->get_baudrate(),
					 inst->get_tcp_port(),
					 inst->get_remote_addr(),
					 (unsigned long long)inst->get_bytes_received(),
					 (unsigned long long)inst->get_bytes_transmitted());
				count++;
			}
		}

		pthread_mutex_unlock(&g_instances_mutex);

		if (count == 0) {
			PX4_INFO("No instances running");

		} else {
			PX4_INFO("\nTotal: %d instance(s) running", count);
		}

		return 0;

	} else {
		SERIALSOCAT::print_usage("unknown command");
		return 1;
	}
}

