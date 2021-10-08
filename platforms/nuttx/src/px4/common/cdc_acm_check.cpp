/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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

#if defined(CONFIG_SYSTEM_CDCACM)
__BEGIN_DECLS
#include <syslog.h>
#include <nuttx/wqueue.h>
#include <builtin/builtin.h>

#include <termios.h>
#include <sys/ioctl.h>

extern int sercon_main(int c, char **argv);
extern int serdis_main(int c, char **argv);
__END_DECLS

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>

#define USB_DEVICE_PATH "/dev/ttyACM0"

static struct work_s usb_serial_work;
static bool vbus_present_prev = false;
static int ttyacm_fd = -1;

enum class UsbAutoStartState {
	disconnected,
	connecting,
	connected,
	disconnecting,
} usb_auto_start_state{UsbAutoStartState::disconnected};


static void mavlink_usb_check(void *arg)
{
	int rescheduled = -1;

	uORB::SubscriptionData<actuator_armed_s> actuator_armed_sub{ORB_ID(actuator_armed)};

	const bool armed = actuator_armed_sub.get().armed;
	const bool vbus_present = (board_read_VBUS_state() == PX4_OK);

	if (!armed) {
		switch (usb_auto_start_state) {
		case UsbAutoStartState::disconnected:
			if (vbus_present && vbus_present_prev) {
				if (sercon_main(0, nullptr) == EXIT_SUCCESS) {
					usb_auto_start_state = UsbAutoStartState::connecting;
					rescheduled = work_queue(LPWORK, &usb_serial_work, mavlink_usb_check, nullptr, USEC2TICK(100000));
				}

			} else if (vbus_present && !vbus_present_prev) {
				// check again sooner if USB just connected
				rescheduled = work_queue(LPWORK, &usb_serial_work, mavlink_usb_check, nullptr, USEC2TICK(100000));
			}

			break;

		case UsbAutoStartState::connecting:
			if (vbus_present && vbus_present_prev) {
				if (ttyacm_fd < 0) {
					ttyacm_fd = ::open(USB_DEVICE_PATH, O_RDONLY | O_NONBLOCK);
				}

				if (ttyacm_fd >= 0) {
					int bytes_available = 0;
					int retval = ::ioctl(ttyacm_fd, FIONREAD, &bytes_available);

					if ((retval == OK) && (bytes_available >= 3)) {
						char buffer[80];

						// non-blocking read
						int nread = ::read(ttyacm_fd, buffer, sizeof(buffer));

#if defined(DEBUG_BUILD)

						if (nread > 0) {
							fprintf(stderr, "%d bytes\n", nread);

							for (int i = 0; i < nread; i++) {
								fprintf(stderr, "|%X", buffer[i]);
							}

							fprintf(stderr, "\n");
						}

#endif // DEBUG_BUILD


						if (nread > 0) {
							bool launch_mavlink = false;
							bool launch_nshterm = false;

							static constexpr int MAVLINK_HEARTBEAT_MIN_LENGTH = 9;

							if (nread >= MAVLINK_HEARTBEAT_MIN_LENGTH) {
								// scan buffer for mavlink HEARTBEAT (v1 & v2)
								for (int i = 0; i < nread - MAVLINK_HEARTBEAT_MIN_LENGTH; i++) {
									if ((buffer[i] = 0xFE) && (buffer[i + 1] = 9) && (buffer[i + 5] == 0)) {
										// mavlink v1 HEARTBEAT
										//  buffer[0]: start byte (0xFE for mavlink v1)
										//  buffer[1]: length (9 for HEARTBEAT)
										//  buffer[3]: SYSID
										//  buffer[4]: COMPID
										//  buffer[5]: mavlink message id (0 for HEARTBEAT)
										syslog(LOG_INFO, "%s: launching mavlink (HEARTBEAT v1 from SYSID:%d COMPID:%d)\n",
										       USB_DEVICE_PATH, buffer[i + 3], buffer[i + 4]);
										launch_mavlink = true;
										break;

									} else if ((buffer[i] = 0xFD) && (buffer[i + 1] = 9)
										   && (buffer[i + 7] == 0) && (buffer[i + 8] == 0) && (buffer[i + 9] == 0)) {
										// mavlink v2 HEARTBEAT
										//  buffer[0]: start byte (0xFD for mavlink v2)
										//  buffer[1]: length (9 for HEARTBEAT)
										//  buffer[5]: SYSID
										//  buffer[6]: COMPID
										//  buffer[7:9]: mavlink message id (0 for HEARTBEAT)
										syslog(LOG_INFO, "%s: launching mavlink (HEARTBEAT v2 from SYSID:%d COMPID:%d)\n",
										       USB_DEVICE_PATH, buffer[i + 5], buffer[i + 6]);
										launch_mavlink = true;
										break;
									}
								}
							}

							if (!launch_mavlink && (nread >= 3)) {
								// nshterm (3 carriage returns)
								// scan buffer looking for 3 consecutive carriage returns (0xD)
								for (int i = 1; i < nread - 1; i++) {
									if (buffer[i - 1] == 0xD && buffer[i] == 0xD && buffer[i + 1] == 0xD) {
										syslog(LOG_INFO, "%s: launching nshterm\n", USB_DEVICE_PATH);
										launch_nshterm = true;
										break;
									}
								}
							}

							if (launch_mavlink || launch_nshterm) {
								// cleanup serial port
								close(ttyacm_fd);
								ttyacm_fd = -1;

								static const char *mavlink_argv[] {"mavlink", "start", "-d", USB_DEVICE_PATH, nullptr};
								static const char *nshterm_argv[] {"nshterm", USB_DEVICE_PATH, nullptr};

								char **exec_argv = nullptr;

								if (launch_nshterm) {
									exec_argv = (char **)nshterm_argv;

								} else if (launch_mavlink) {
									exec_argv = (char **)mavlink_argv;
								}

								sched_lock();

								if (exec_builtin(exec_argv[0], exec_argv, nullptr, 0) > 0) {
									usb_auto_start_state = UsbAutoStartState::connected;

								} else {
									usb_auto_start_state = UsbAutoStartState::disconnecting;
								}

								sched_unlock();
							}
						}
					}
				}

			} else {
				// cleanup
				if (ttyacm_fd >= 0) {
					close(ttyacm_fd);
					ttyacm_fd = -1;
				}

				usb_auto_start_state = UsbAutoStartState::disconnecting;
			}

			break;

		case UsbAutoStartState::connected:
			if (!vbus_present && !vbus_present_prev) {
				sched_lock();
				static const char app[] {"mavlink"};
				static const char *stop_argv[] {"mavlink", "stop", "-d", USB_DEVICE_PATH, NULL};
				exec_builtin(app, (char **)stop_argv, NULL, 0);
				sched_unlock();

				usb_auto_start_state = UsbAutoStartState::disconnecting;
			}

			break;

		case UsbAutoStartState::disconnecting:
			// serial disconnect if unused
			serdis_main(0, NULL);
			usb_auto_start_state = UsbAutoStartState::disconnected;
			break;
		}
	}

	vbus_present_prev = vbus_present;

	if (rescheduled != PX4_OK) {
		work_queue(LPWORK, &usb_serial_work, mavlink_usb_check, NULL, USEC2TICK(1000000));
	}
}


void cdcacm_init(void)
{
	work_queue(LPWORK, &usb_serial_work, mavlink_usb_check, nullptr, 0);
}

#endif // CONFIG_SYSTEM_CDCACM
