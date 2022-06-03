/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

/*
 * MXS.hpp
 *
 * Sagetech MXS transponder driver
 * @author Megan McCormick megan.mccormick@sagetech.com
 * @author Check Faber chuck.faber@sagetech.com
 */

#ifndef DRIVERS_TRANSPONDER_SAGETECH_MXS_MXS_HPP_
#define DRIVERS_TRANSPONDER_SAGETECH_MXS_MXS_HPP_

extern "C"
{
	#include "sg.h"
}


#include <termios.h>
#include <stdint.h>
#include <lib/geo/geo.h>

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/perf/perf_counter.h>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>


using namespace time_literals;


/*********************************************************************************
 * Defines
 *********************************************************************************/
//Scaling
#define SAGETECH_SCALE_FEET_TO_M 						0.3048f
#define SAGETECH_SCALE_KNOTS_TO_M_PER_SEC 				0.514444f
#define SAGETECH_SCALE_M_PER_SEC_TO_KNOTS				1.94384F
#define SAGETECH_SCALE_FT_PER_MIN_TO_M_PER_SEC 			0.00508f

//Altitude
#define ADSB_ALTITUDE_TYPE_PRESSURE_QNH 				0
#define ADSB_ALTITUDE_TYPE_GEOMETRIC					1

#define PAYLOAD_MXS_MAX_SIZE  							255
#define START_BYTE										0xAA
#define SAGETECH_PI										3.14159
#define SAGETECH_USEC_PER_HOUR							3600000000
#define SAGETECH_USEC_PER_MIN							60000000
#define SAGETECH_USEC_PER_SEC							1000000

#define SAGETECH_HFOM_UNKNOWN 							19000.0f
#define SAGETECH_VFOM_UNKNOWN 							151.0f
#define SAGETECH_HPL_UNKNOWN 							38000.0f

#define MAX_VEHICLES_TRACKED							25

//Timing
#define FIVE_HZ_MOD										10
#define TWO_HZ_MOD										25
#define	ONE_HZ_MOD										50
#define EIGHT_TWO_SEC_MOD								410
#define SAGETECH_MXS_POLL_RATE     						20_ms

/*********************************************************************************
 * Enum definitions
 *********************************************************************************/
typedef enum
{
	startByte,
	msgByte,
	idByte,
	lengthByte,
	payload,
	checksumByte
}parse_state_t;

/*********************************************************************************
 * Struct definitions
 *********************************************************************************/
typedef struct
{
	uint8_t state;
	uint8_t index;
	uint8_t start;
	uint8_t type;
	uint8_t id;
	uint8_t length;
	uint8_t payload[PAYLOAD_MXS_MAX_SIZE];
	uint8_t checksum;
}sagetech_packet_t;

typedef struct
{
	transponder_report_s targetInfo;
	bool matched;
}targets_t;

typedef struct
{
	uint16_t listCount;
	targets_t list[60];

}target_list_t;


/*********************************************************************************
 * Class definition
 *********************************************************************************/

class MXS : public ModuleBase<MXS>,public px4::ScheduledWorkItem ,public device::Device,public ModuleParams
{
public:
	MXS(const char *serial_port,unsigned baudrate);
	~MXS() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int init();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

	/**
	 * Initialise the automatic measurement state machine and start it.
	 */
	void start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void stop();

private:

	//Functions

	void Run() override;

	int collect();

	int open_serial_port();

	void parameters_update();

	void handle_msg();

	void parse_byte(uint8_t data);

	void handle_svr(sg_svr_t svr);

	void handle_msr(sg_msr_t msr);

	uint8_t determine_emitter(sg_adsb_emitter_t emit);

	void send_gps_msg();

	void send_flight_id_msg();

	void send_op_msg();

	void send_target_req_msg();

	void buff_to_hex(char*out,const uint8_t *buff, int len);

	sg_nacv_t determine_nacv(float velAcc);

	speed_t convert_baudrate(unsigned baud);

	bool get_vehicle_by_ICAO(const uint32_t icao, transponder_report_s &vehicle) const;

	bool find_index(const transponder_report_s &vehicle, uint16_t *index) const;

	void set_vehicle(const uint16_t index, const transponder_report_s &vehicle);

	void delete_vehicle(const uint16_t index);

	void handle_vehicle(const transponder_report_s &vehicle);

	void determine_furthest_aircraft(void);

	char 				*_serial_port{nullptr};
	unsigned			_baudrate{57600};

	int 				_file_descriptor{-1};

	uint8_t 			_msgId{0};

	uint8_t 			_buffer[128];
	uint8_t 			_buffer_len{0};

	sagetech_packet_t	_msgIn;

	hrt_abstime			_last_gps_send{0};

	uint64_t 			_loop_count{0};

	//Sensor data
	sensor_gps_s 		_gps;
	vehicle_land_detected_s _landed;

	// External Vehicle List
	transponder_report_s vehicle_list[MAX_VEHICLES_TRACKED];
	uint16_t vehicle_count{0};
	uint16_t furthest_vehicle_index{0};
	float furthest_vehicle_distance{0.0};

	struct {
			// cached variables to compare against params so we can send msg on param change.
			uint16_t        operating_squawk;
			int32_t         operating_alt;
			bool            failXpdr;
			bool            failSystem;
			struct {
				uint8_t     id;
				uint8_t     type;
			} msg;
		} last;




	//Custom Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MXS_MODE>) _mxs_mode,
		(ParamInt<px4::params::MXS_SQUAWK>) _mxs_squawk,
		(ParamInt<px4::params::MXS_IDENT>) _mxs_ident
	)

	//Publications
	uORB::PublicationMulti<transponder_report_s> 	_transponder_pub{ORB_ID(transponder_report)};

	//Subscriptions
	uORB::SubscriptionInterval 			_parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription                 _sensor_gps_sub{ORB_ID(sensor_gps)};
	uORB::Subscription                 _transponder_sub{ORB_ID(transponder_report)};
	uORB::Subscription                 _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	//Performance counters
	perf_counter_t	_loop_count_perf{perf_alloc(PC_COUNT, MODULE_NAME": count")};
	perf_counter_t	_loop_elapsed_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
	perf_counter_t  _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t  _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
};


#endif /* DRIVERS_TRANSPONDER_SAGETECH_MXS_MXS_HPP_ */
