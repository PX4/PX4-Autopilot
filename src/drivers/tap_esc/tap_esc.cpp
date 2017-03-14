/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#include <stdint.h>

#include <px4_tasks.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <errno.h>
#include <termios.h>
#include <cmath>	// NAN

#include <lib/mathlib/mathlib.h>
#include <systemlib/px4_macros.h>
#include <drivers/device/device.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/test_motor.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/multirotor_motor_limits.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/param/param.h>
#include <systemlib/pwm_limit/pwm_limit.h>

#define NAN_VALUE	(0.0f/0.0f)

#ifndef B250000
#define B250000 250000
#endif

#define ESC_HAVE_CURRENT_SENSOR

#include "drv_tap_esc.h"

/*
 * This driver connects to TAP ESCs via serial.
 */

static int _uart_fd = -1; //todo:refactor in to class
class TAP_ESC : public device::CDev
{
public:
	enum Mode {
		MODE_NONE,
		MODE_2PWM,
		MODE_2PWM2CAP,
		MODE_3PWM,
		MODE_3PWM1CAP,
		MODE_4PWM,
		MODE_6PWM,
		MODE_8PWM,
		MODE_4CAP,
		MODE_5CAP,
		MODE_6CAP,
	};
	TAP_ESC(int channels_count);
	virtual ~TAP_ESC();
	virtual int	init();
	virtual int	ioctl(file *filp, int cmd, unsigned long arg);
	void cycle();
protected:
	void select_responder(uint8_t sel);
private:

	static const uint8_t crcTable[256];
	static const uint8_t device_mux_map[TAP_ESC_MAX_MOTOR_NUM];
	static const uint8_t device_dir_map[TAP_ESC_MAX_MOTOR_NUM];

	bool _is_armed;

	unsigned	_poll_fds_num;
	Mode		_mode;
	// subscriptions
	int	_armed_sub;
	int _test_motor_sub;
	orb_advert_t        	_outputs_pub = nullptr;
	actuator_outputs_s      _outputs;
	static actuator_armed_s	_armed;

	//todo:refactor dynamic based on _channels_count
	// It needs to support the numbe of ESC
	int	_control_subs[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];

	pollfd	_poll_fds[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];

	actuator_controls_s _controls[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];

	orb_id_t	_control_topics[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];

	orb_advert_t        _esc_feedback_pub = nullptr;
	orb_advert_t      _to_mixer_status; 	///< mixer status flags
	esc_status_s      _esc_feedback;
	uint8_t           _channels_count; // The number of ESC channels

	MixerGroup	*_mixers;
	uint32_t	_groups_required;
	uint32_t	_groups_subscribed;
	volatile bool	_initialized;
	unsigned	_pwm_default_rate;
	unsigned	_current_update_rate;
	ESC_UART_BUF uartbuf;
	EscPacket  		_packet;
	void		subscribe();

	void		work_start();
	void		work_stop();
	void send_esc_outputs(const float *pwm, const unsigned num_pwm);
	uint8_t crc8_esc(uint8_t *p, uint8_t len);
	uint8_t crc_packet(EscPacket &p);
	int send_packet(EscPacket &p, int responder);
	void read_data_from_uart();
	bool parse_tap_esc_feedback(ESC_UART_BUF *serial_buf, EscPacket *packetdata);
	static int control_callback(uintptr_t handle,
				    uint8_t control_group,
				    uint8_t control_index,
				    float &input);
};

const uint8_t TAP_ESC::crcTable[256] = TAP_ESC_CRC;
const uint8_t TAP_ESC::device_mux_map[TAP_ESC_MAX_MOTOR_NUM] = ESC_POS;
const uint8_t TAP_ESC::device_dir_map[TAP_ESC_MAX_MOTOR_NUM] = ESC_DIR;

actuator_armed_s TAP_ESC::_armed = {};

namespace
{
TAP_ESC	*tap_esc = nullptr;
}

# define TAP_ESC_DEVICE_PATH	"/dev/tap_esc"

TAP_ESC::TAP_ESC(int channels_count):
	CDev("tap_esc", TAP_ESC_DEVICE_PATH),
	_is_armed(false),
	_poll_fds_num(0),
	_mode(MODE_4PWM), //FIXME: what is this mode used for???
	_armed_sub(-1),
	_test_motor_sub(-1),
	_outputs_pub(nullptr),
	_control_subs{ -1},
	_esc_feedback_pub(nullptr),
	_to_mixer_status(nullptr),
	_esc_feedback{},
	_channels_count(channels_count),
	_mixers(nullptr),
	_groups_required(0),
	_groups_subscribed(0),
	_initialized(false),
	_pwm_default_rate(400),
	_current_update_rate(0)

{
	_control_topics[0] = ORB_ID(actuator_controls_0);
	_control_topics[1] = ORB_ID(actuator_controls_1);
	_control_topics[2] = ORB_ID(actuator_controls_2);
	_control_topics[3] = ORB_ID(actuator_controls_3);
	memset(_controls, 0, sizeof(_controls));
	memset(_poll_fds, 0, sizeof(_poll_fds));
	uartbuf.head = 0;
	uartbuf.tail = 0;
	uartbuf.dat_cnt = 0;
	memset(uartbuf.esc_feedback_buf, 0, sizeof(uartbuf.esc_feedback_buf));

	for (size_t i = 0; i < sizeof(_outputs.output) / sizeof(_outputs.output[0]); i++) {
		_outputs.output[i] = NAN;
	}

	_outputs.noutputs = 0;
}

TAP_ESC::~TAP_ESC()
{
	if (_initialized) {
		/* tell the task we want it to go away */
		work_stop();

		int i = 10;

		do {
			/* wait 50ms - it should wake every 100ms or so worst-case */
			usleep(50000);
			i--;

		} while (_initialized && i > 0);
	}

	// clean up the alternate device node
	//unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

	tap_esc = nullptr;
}

int
TAP_ESC::init()
{
	int ret;

	ASSERT(!_initialized);

	/* Respect boot time required by the ESC FW */

	hrt_abstime uptime_us = hrt_absolute_time();

	if (uptime_us < MAX_BOOT_TIME_MS * 1000) {
		usleep((MAX_BOOT_TIME_MS * 1000) - uptime_us);
	}

	/* Issue Basic Config */

	EscPacket packet = {0xfe, sizeof(ConfigInfoBasicRequest), ESCBUS_MSG_ID_CONFIG_BASIC};
	ConfigInfoBasicRequest   &config = packet.d.reqConfigInfoBasic;
	memset(&config, 0, sizeof(ConfigInfoBasicRequest));
	config.maxChannelInUse = _channels_count;

	/* Asign the id's to the ESCs to match the mux */

	for (uint8_t phy_chan_index = 0; phy_chan_index < _channels_count; phy_chan_index++) {
		config.channelMapTable[phy_chan_index] = device_mux_map[phy_chan_index] &
				ESC_CHANNEL_MAP_CHANNEL;
		config.channelMapTable[phy_chan_index] |= (device_dir_map[phy_chan_index] << 4) &
				ESC_CHANNEL_MAP_RUNNING_DIRECTION;
	}

	config.maxChannelValue = RPMMAX;
	config.minChannelValue = RPMMIN;

	ret = send_packet(packet, 0);

	if (ret < 0) {
		return ret;
	}

#if !defined(TAP_ESC_NO_VERIFY_CONFIG)

	/* Verify All ESC got the config */

	for (uint8_t cid = 0; cid < _channels_count; cid++) {

		/* Send the InfoRequest querying  CONFIG_BASIC */
		EscPacket packet_info = {0xfe, sizeof(InfoRequest), ESCBUS_MSG_ID_REQUEST_INFO};
		InfoRequest &info_req = packet_info.d.reqInfo;
		info_req.channelID = cid;
		info_req.requestInfoType = REQEST_INFO_BASIC;

		ret = send_packet(packet_info, cid);

		if (ret < 0) {
			return ret;
		}

		/* Get a response */

		int retries = 10;
		bool valid = false;

		while (retries--) {

			read_data_from_uart();

			if (parse_tap_esc_feedback(&uartbuf, &_packet)) {
				valid = (_packet.msg_id == ESCBUS_MSG_ID_CONFIG_INFO_BASIC
					 && _packet.d.rspConfigInfoBasic.channelID == cid
					 && 0 == memcmp(&_packet.d.rspConfigInfoBasic.resp, &config, sizeof(ConfigInfoBasicRequest)));
				break;

			} else {

				/* Give it time to come in */

				usleep(1000);
			}
		}

		if (!valid) {
			return -EIO;
		}

	}

#endif //

	/* To Unlock the ESC from the Power up state we need to issue 10
	 * ESCBUS_MSG_ID_RUN request with all the values 0;
	 */

	EscPacket unlock_packet = {0xfe, _channels_count, ESCBUS_MSG_ID_RUN};
	unlock_packet.len *= sizeof(unlock_packet.d.reqRun.rpm_flags[0]);
	memset(unlock_packet.d.bytes, 0, sizeof(packet.d.bytes));

	int unlock_times = 10;

	while (unlock_times--) {

		send_packet(unlock_packet, -1);

		/* Min Packet to Packet time is 1 Ms so use 2 */

		usleep(1000 * 2);
	}

	/* do regular cdev init */

	ret = CDev::init();

	return ret;
}

int TAP_ESC::send_packet(EscPacket &packet, int responder)
{
	if (responder >= 0) {

		if (responder > _channels_count) {
			return -EINVAL;
		}

		select_responder(responder);
	}

	int packet_len = crc_packet(packet);
	int ret = ::write(_uart_fd, &packet.head, packet_len);

	if (ret != packet_len) {
		PX4_WARN("TX ERROR: ret: %d, errno: %d", ret, errno);
	}

	return ret;
}

void
TAP_ESC::subscribe()
{
	/* subscribe/unsubscribe to required actuator control groups */
	uint32_t sub_groups = _groups_required & ~_groups_subscribed;
	uint32_t unsub_groups = _groups_subscribed & ~_groups_required;
	_poll_fds_num = 0;

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (sub_groups & (1 << i)) {
			DEVICE_DEBUG("subscribe to actuator_controls_%d", i);
			_control_subs[i] = orb_subscribe(_control_topics[i]);
		}

		if (unsub_groups & (1 << i)) {
			DEVICE_DEBUG("unsubscribe from actuator_controls_%d", i);
			orb_unsubscribe(_control_subs[i]);
			_control_subs[i] = -1;
		}

		if (_control_subs[i] > 0) {
			_poll_fds[_poll_fds_num].fd = _control_subs[i];
			_poll_fds[_poll_fds_num].events = POLLIN;
			_poll_fds_num++;
		}
	}
}

uint8_t TAP_ESC::crc8_esc(uint8_t *p, uint8_t len)
{
	uint8_t crc = 0;

	for (uint8_t i = 0; i < len; i++) {
		crc = crcTable[crc^*p++];
	}

	return crc;
}

uint8_t TAP_ESC::crc_packet(EscPacket &p)
{
	/* Calculate the crc over Len,ID,data */
	p.d.bytes[p.len] = crc8_esc(&p.len, p.len + 2);
	return p.len + offsetof(EscPacket, d) + 1;
}
void TAP_ESC::select_responder(uint8_t sel)
{
#if defined(GPIO_S0)
	px4_arch_gpiowrite(GPIO_S0, sel & 1);
	px4_arch_gpiowrite(GPIO_S1, sel & 2);
	px4_arch_gpiowrite(GPIO_S2, sel & 4);
#endif
}


void TAP_ESC:: send_esc_outputs(const float *pwm, const unsigned num_pwm)
{

	uint16_t rpm[TAP_ESC_MAX_MOTOR_NUM];
	memset(rpm, 0, sizeof(rpm));
	uint8_t motor_cnt = num_pwm;
	static uint8_t which_to_respone = 0;

	for (uint8_t i = 0; i < motor_cnt; i++) {
		rpm[i] = pwm[i];

		if (rpm[i] > RPMMAX) {
			rpm[i] = RPMMAX;

		} else if (rpm[i] < RPMSTOPPED) {
			rpm[i] = RPMSTOPPED;
		}
	}

	rpm[which_to_respone] |= (RUN_FEEDBACK_ENABLE_MASK | RUN_BLUE_LED_ON_MASK);


	EscPacket packet = {0xfe, _channels_count, ESCBUS_MSG_ID_RUN};
	packet.len *= sizeof(packet.d.reqRun.rpm_flags[0]);

	for (uint8_t i = 0; i < _channels_count; i++) {
		packet.d.reqRun.rpm_flags[i] = rpm[i];
	}

	int ret = send_packet(packet, which_to_respone);

	if (++which_to_respone == _channels_count) {
		which_to_respone = 0;
	}

	if (ret < 1) {
		PX4_WARN("TX ERROR: ret: %d, errno: %d", ret, errno);
	}
}

void TAP_ESC::read_data_from_uart()
{
	uint8_t tmp_serial_buf[UART_BUFFER_SIZE];

	int len =::read(_uart_fd, tmp_serial_buf, arraySize(tmp_serial_buf));

	if (len > 0 && (uartbuf.dat_cnt + len < UART_BUFFER_SIZE)) {
		for (int i = 0; i < len; i++) {
			uartbuf.esc_feedback_buf[uartbuf.tail++] = tmp_serial_buf[i];
			uartbuf.dat_cnt++;

			if (uartbuf.tail >= UART_BUFFER_SIZE) {
				uartbuf.tail = 0;
			}
		}
	}
}

bool TAP_ESC:: parse_tap_esc_feedback(ESC_UART_BUF *serial_buf, EscPacket *packetdata)
{
	static PARSR_ESC_STATE state = HEAD;
	static uint8_t data_index = 0;
	static uint8_t crc_data_cal;

	if (serial_buf->dat_cnt > 0) {
		int count = serial_buf->dat_cnt;

		for (int i = 0; i < count; i++) {
			switch (state) {
			case HEAD:
				if (serial_buf->esc_feedback_buf[serial_buf->head] == 0xFE) {
					packetdata->head = 0xFE; //just_keep the format
					state = LEN;
				}

				break;

			case LEN:
				if (serial_buf->esc_feedback_buf[serial_buf->head] < sizeof(packetdata->d)) {
					packetdata->len = serial_buf->esc_feedback_buf[serial_buf->head];
					state = ID;

				} else {
					state = HEAD;
				}

				break;

			case ID:
				if (serial_buf->esc_feedback_buf[serial_buf->head] < ESCBUS_MSG_ID_MAX_NUM) {
					packetdata->msg_id = serial_buf->esc_feedback_buf[serial_buf->head];
					data_index = 0;
					state = DATA;

				} else {
					state = HEAD;
				}

				break;

			case DATA:
				packetdata->d.bytes[data_index++] = serial_buf->esc_feedback_buf[serial_buf->head];

				if (data_index >= packetdata->len) {

					crc_data_cal = crc8_esc((uint8_t *)(&packetdata->len), packetdata->len + 2);
					state = CRC;
				}

				break;

			case CRC:
				if (crc_data_cal == serial_buf->esc_feedback_buf[serial_buf->head]) {
					packetdata->crc_data = serial_buf->esc_feedback_buf[serial_buf->head];

					if (++serial_buf->head >= UART_BUFFER_SIZE) {
						serial_buf->head = 0;
					}

					serial_buf->dat_cnt--;
					state = HEAD;
					return true;
				}

				state = HEAD;
				break;

			default:
				state = HEAD;
				break;

			}

			if (++serial_buf->head >= UART_BUFFER_SIZE) {
				serial_buf->head = 0;
			}

			serial_buf->dat_cnt--;

		}


	}

	return false;
}

void
TAP_ESC::cycle()
{

	if (!_initialized) {
		_current_update_rate = 0;
		/* advertise the mixed control outputs, insist on the first group output */
		_outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &_outputs);
		_esc_feedback_pub = orb_advertise(ORB_ID(esc_status), &_esc_feedback);
		multirotor_motor_limits_s multirotor_motor_limits = {};
		_to_mixer_status = orb_advertise(ORB_ID(multirotor_motor_limits), &multirotor_motor_limits);
		_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
		_test_motor_sub = orb_subscribe(ORB_ID(test_motor));
		_initialized = true;
	}

	if (_groups_subscribed != _groups_required) {
		subscribe();
		_groups_subscribed = _groups_required;
		_current_update_rate = 0;
	}

	unsigned max_rate = _pwm_default_rate ;

	if (_current_update_rate != max_rate) {
		_current_update_rate = max_rate;
		int update_rate_in_ms = int(1000 / _current_update_rate);

		/* reject faster than 500 Hz updates */
		if (update_rate_in_ms < 2) {
			update_rate_in_ms = 2;
		}

		/* reject slower than 10 Hz updates */
		if (update_rate_in_ms > 100) {
			update_rate_in_ms = 100;
		}

		DEVICE_DEBUG("adjusted actuator update interval to %ums", update_rate_in_ms);

		for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_control_subs[i] > 0) {
				orb_set_interval(_control_subs[i], update_rate_in_ms);
			}
		}

		// set to current max rate, even if we are actually checking slower/faster
		_current_update_rate = max_rate;
	}



	/* check if anything updated */
	int ret = ::poll(_poll_fds, _poll_fds_num, 5);


	/* this would be bad... */
	if (ret < 0) {
		DEVICE_LOG("poll error %d", errno);

	} else { /* update even in the case of a timeout, to check for test_motor commands */

		/* get controls for required topics */
		unsigned poll_id = 0;

		for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_control_subs[i] > 0) {
				if (_poll_fds[poll_id].revents & POLLIN) {
					orb_copy(_control_topics[i], _control_subs[i], &_controls[i]);

				}

				poll_id++;
			}
		}

		size_t num_outputs = _channels_count;

		/* can we mix? */
		if (_is_armed && _mixers != nullptr) {

			/* do mixing */
			num_outputs = _mixers->mix(&_outputs.output[0], num_outputs, NULL);
			_outputs.noutputs = num_outputs;
			_outputs.timestamp = hrt_absolute_time();

			/* publish mixer status */
			multirotor_motor_limits_s multirotor_motor_limits = {};
			multirotor_motor_limits.saturation_status = _mixers->get_saturation_status();

			orb_publish(ORB_ID(multirotor_motor_limits), _to_mixer_status, &multirotor_motor_limits);

			/* disable unused ports by setting their output to NaN */
			for (size_t i = num_outputs; i < sizeof(_outputs.output) / sizeof(_outputs.output[0]); i++) {
				_outputs.output[i] = NAN;
			}

			/* iterate actuators */
			for (unsigned i = 0; i < num_outputs; i++) {
				/* last resort: catch NaN, INF and out-of-band errors */
				if (i < _outputs.noutputs &&
				    PX4_ISFINITE(_outputs.output[i])) {
					/* scale for PWM output 1200 - 1900us */
					_outputs.output[i] = (RPMMAX + RPMMIN) / 2 + ((RPMMAX - RPMMIN) / 2) * _outputs.output[i];
					math::constrain(_outputs.output[i], (float)RPMMIN, (float)RPMMAX);

				} else {
					/*
					 * Value is NaN, INF - stop the motor.
					 * This will be clearly visible on the servo status and will limit the risk of accidentally
					 * spinning motors. It would be deadly in flight.
					 */
					_outputs.output[i] = RPMSTOPPED;
				}
			}

		} else {

			_outputs.noutputs = num_outputs;
			_outputs.timestamp = hrt_absolute_time();

			/* check for motor test commands */
			bool test_motor_updated;
			orb_check(_test_motor_sub, &test_motor_updated);

			if (test_motor_updated) {
				struct test_motor_s test_motor;
				orb_copy(ORB_ID(test_motor), _test_motor_sub, &test_motor);
				_outputs.output[test_motor.motor_number] = RPMSTOPPED + ((RPMMAX - RPMSTOPPED) * test_motor.value);
				PX4_INFO("setting motor %i to %.1lf", test_motor.motor_number,
					 (double)_outputs.output[test_motor.motor_number]);
			}

			/* set the invalid values to the minimum */
			for (unsigned i = 0; i < num_outputs; i++) {
				if (!PX4_ISFINITE(_outputs.output[i])) {
					_outputs.output[i] = RPMSTOPPED;
				}
			}

			/* disable unused ports by setting their output to NaN */
			for (size_t i = num_outputs; i < sizeof(_outputs.output) / sizeof(_outputs.output[0]); i++) {
				_outputs.output[i] = NAN;
			}

		}

		const unsigned esc_count = num_outputs;
		float motor_out[TAP_ESC_MAX_MOTOR_NUM];

		// We need to remap from the system default to what PX4's normal
		// scheme is
		if (num_outputs == 6) {
			motor_out[0] = _outputs.output[3];
			motor_out[1] = _outputs.output[0];
			motor_out[2] = _outputs.output[4];
			motor_out[3] = _outputs.output[2];
			motor_out[4] = _outputs.output[1];
			motor_out[5] = _outputs.output[5];
			motor_out[6] = RPMSTOPPED;
			motor_out[7] = RPMSTOPPED;

		} else if (num_outputs == 4) {

			motor_out[0] = _outputs.output[2];
			motor_out[2] = _outputs.output[0];
			motor_out[1] = _outputs.output[1];
			motor_out[3] = _outputs.output[3];

		} else {

			// Use the system defaults
			for (int i = 0; i < esc_count; ++i) {
				motor_out[i] = _outputs.output[i];
			}
		}

		send_esc_outputs(motor_out, esc_count);
		read_data_from_uart();

		if (parse_tap_esc_feedback(&uartbuf, &_packet) == true) {
			if (_packet.msg_id == ESCBUS_MSG_ID_RUN_INFO) {
				RunInfoRepsonse &feed_back_data = _packet.d.rspRunInfo;

				if (feed_back_data.channelID < esc_status_s::CONNECTED_ESC_MAX) {
					_esc_feedback.esc[feed_back_data.channelID].esc_rpm = feed_back_data.speed;
//					_esc_feedback.esc[feed_back_data.channelID].esc_voltage = feed_back_data.voltage;
					_esc_feedback.esc[feed_back_data.channelID].esc_state = feed_back_data.ESCStatus;
					_esc_feedback.esc[feed_back_data.channelID].esc_vendor = esc_status_s::ESC_VENDOR_TAP;
					// printf("vol is %d\n",feed_back_data.voltage );
					// printf("speed is %d\n",feed_back_data.speed );

					_esc_feedback.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_SERIAL;
					_esc_feedback.counter++;
					_esc_feedback.esc_count = esc_count;

					_esc_feedback.timestamp = hrt_absolute_time();

					orb_publish(ORB_ID(esc_status), _esc_feedback_pub, &_esc_feedback);
				}
			}
		}

		/* and publish for anyone that cares to see */
		orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &_outputs);

	}

	bool updated;

	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);

		if (_is_armed != _armed.armed) {
			/* reset all outputs */
			for (size_t i = 0; i < sizeof(_outputs.output) / sizeof(_outputs.output[0]); i++) {
				_outputs.output[i] = NAN;
			}
		}

		/* do not obey the lockdown value, as lockdown is for PWMSim */
		_is_armed = _armed.armed;

	}


}

void TAP_ESC::work_stop()
{
	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_control_subs[i] > 0) {
			orb_unsubscribe(_control_subs[i]);
			_control_subs[i] = -1;
		}
	}

	orb_unsubscribe(_armed_sub);
	_armed_sub = -1;
	orb_unsubscribe(_test_motor_sub);
	_test_motor_sub = -1;

	DEVICE_LOG("stopping");
	_initialized = false;
}

int
TAP_ESC::control_callback(uintptr_t handle,
			  uint8_t control_group,
			  uint8_t control_index,
			  float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	input = controls[control_group].control[control_index];

	/* limit control input */
	if (input > 1.0f) {
		input = 1.0f;

	} else if (input < -1.0f) {
		input = -1.0f;
	}

	/* motor spinup phase - lock throttle to zero */
	// if (_pwm_limit.state == PWM_LIMIT_STATE_RAMP) {
	// 	if ((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
	// 	     control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
	// 	    control_index == actuator_controls_s::INDEX_THROTTLE) {
	// 		/* limit the throttle output to zero during motor spinup,
	// 		 * as the motors cannot follow any demand yet
	// 		 */
	// 		input = 0.0f;
	// 	}
	// }

	/* throttle not arming - mark throttle input as invalid */
	if (_armed.prearmed && !_armed.armed) {
		if ((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
		     control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
		    control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* set the throttle to an invalid value */
			input = NAN_VALUE;
		}
	}

	return 0;
}

int
TAP_ESC::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	switch (cmd) {

	case MIXERIOCRESET:
		if (_mixers != nullptr) {
			delete _mixers;
			_mixers = nullptr;
			_groups_required = 0;
		}

		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strnlen(buf, 1024);

			if (_mixers == nullptr) {
				_mixers = new MixerGroup(control_callback, (uintptr_t)_controls);
			}

			if (_mixers == nullptr) {
				_groups_required = 0;
				ret = -ENOMEM;

			} else {

				ret = _mixers->load_from_buf(buf, buflen);

				if (ret != 0) {
					DEVICE_DEBUG("mixer load failed with %d", ret);
					delete _mixers;
					_mixers = nullptr;
					_groups_required = 0;
					ret = -EINVAL;

				} else {

					_mixers->groups_required(_groups_required);
				}
			}

			break;
		}

	default:
		ret = -ENOTTY;
		break;
	}



	return ret;
}

namespace tap_esc_drv
{


volatile bool _task_should_exit = false; // flag indicating if tap_esc task should exit
static char _device[32] = {};
static bool _is_running = false;         // flag indicating if tap_esc app is running
static px4_task_t _task_handle = -1;     // handle to the task main thread
static int _supported_channel_count = 0;

static bool _flow_control_enabled = false;

void usage();

void start();
void stop();
int tap_esc_start(void);
int tap_esc_stop(void);

void task_main_trampoline(int argc, char *argv[]);

void task_main(int argc, char *argv[]);

int initialise_uart();

int deinitialize_uart();

int enable_flow_control(bool enabled);

int tap_esc_start(void)
{
	int ret = OK;

	if (tap_esc == nullptr) {

		tap_esc = new TAP_ESC(_supported_channel_count);

		if (tap_esc == nullptr) {
			ret = -ENOMEM;

		} else {
			ret = tap_esc->init();

			if (ret != OK) {
				PX4_ERR("failed to initialize tap_esc (%i)", ret);
				delete tap_esc;
				tap_esc = nullptr;
			}
		}
	}

	return ret;
}

int tap_esc_stop(void)
{
	int ret = OK;

	if (tap_esc != nullptr) {

		delete tap_esc;
		tap_esc = nullptr;
	}

	return ret;
}

int initialise_uart()
{
	// open uart
	_uart_fd = open(_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
	int termios_state = -1;

	if (_uart_fd < 0) {
		PX4_ERR("failed to open uart device!");
		return -1;
	}

	// set baud rate
	int speed = B250000;
	struct termios uart_config;
	tcgetattr(_uart_fd, &uart_config);

	// clear ONLCR flag (which appends a CR for every LF)
	uart_config.c_oflag &= ~ONLCR;

	// set baud rate
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("failed to set baudrate for %s: %d\n", _device, termios_state);
		close(_uart_fd);
		return -1;
	}

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("tcsetattr failed for %s\n", _device);
		close(_uart_fd);
		return -1;
	}

	// setup output flow control
	if (enable_flow_control(false)) {
		PX4_WARN("hardware flow disable failed");
	}

	return _uart_fd;
}

int enable_flow_control(bool enabled)
{
	struct termios uart_config;

	int ret = tcgetattr(_uart_fd, &uart_config);

	if (enabled) {
		uart_config.c_cflag |= CRTSCTS;

	} else {
		uart_config.c_cflag &= ~CRTSCTS;
	}

	ret = tcsetattr(_uart_fd, TCSANOW, &uart_config);

	if (!ret) {
		_flow_control_enabled = enabled;
	}

	return ret;
}

int deinitialize_uart()
{
	return close(_uart_fd);
}

void task_main(int argc, char *argv[])
{

	_is_running = true;

	if (initialise_uart() < 0) {
		PX4_ERR("Failed to initialize UART.");

		while (_task_should_exit == false) {
			usleep(100000);
		}

		_is_running = false;
		return;
	}

	if (tap_esc_start() != OK) {
		PX4_ERR("failed to start tap_esc.");
		_is_running = false;
		return;
	}


	// Main loop
	while (!_task_should_exit) {

		tap_esc->cycle();

	}


	_is_running = false;
}

void task_main_trampoline(int argc, char *argv[])
{
	task_main(argc, argv);
}

void start()
{
	ASSERT(_task_handle == -1);

	_task_should_exit = false;

	/* start the task */
	_task_handle = px4_task_spawn_cmd("tap_esc_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MAX,
					  1100,
					  (px4_main_t)&task_main_trampoline,
					  nullptr);

	if (_task_handle < 0) {
		PX4_ERR("task start failed");
		_task_handle = -1;
		return;
	}
}

void stop()
{
	_task_should_exit = true;

	while (_is_running) {
		usleep(200000);
		PX4_INFO("tap_esc_stop");
	}

	tap_esc_stop();
	deinitialize_uart();
	_task_handle = -1;
}

void usage()
{
	PX4_INFO("usage: tap_esc start -d /dev/ttyS2 -n <1-8>");
	PX4_INFO("       tap_esc stop");
	PX4_INFO("       tap_esc status");
}

} // namespace tap_esc

// driver 'main' command
extern "C" __EXPORT int tap_esc_main(int argc, char *argv[]);

int tap_esc_main(int argc, char *argv[])
{
	const char *device = nullptr;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	char *verb = nullptr;

	if (argc >= 2) {
		verb = argv[1];
	}

	while ((ch = px4_getopt(argc, argv, "d:n:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			strncpy(tap_esc_drv::_device, device, strlen(device));
			break;

		case 'n':
			tap_esc_drv::_supported_channel_count = atoi(myoptarg);
			break;
		}
	}

	if (!tap_esc && tap_esc_drv::_task_handle != -1) {
		tap_esc_drv::_task_handle = -1;
	}

	// Start/load the driver.
	if (!strcmp(verb, "start")) {
		if (tap_esc_drv::_is_running) {
			PX4_WARN("tap_esc already running");
			return 1;
		}

		// Check on required arguments
		if (tap_esc_drv::_supported_channel_count == 0 || device == nullptr || strlen(device) == 0) {
			tap_esc_drv::usage();
			return 1;
		}

		tap_esc_drv::start();
	}

	else if (!strcmp(verb, "stop")) {
		if (!tap_esc_drv::_is_running) {
			PX4_WARN("tap_esc is not running");
			return 1;
		}

		tap_esc_drv::stop();
	}

	else if (!strcmp(verb, "status")) {
		PX4_WARN("tap_esc is %s", tap_esc_drv::_is_running ? "running" : "not running");
		return 0;

	} else {
		tap_esc_drv::usage();
		return 1;
	}

	return 0;
}
