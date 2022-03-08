/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "WorkItemExample.hpp"
#include <drivers/drv_hrt.h>
 
#if 1
#ifdef __clang__
#ifdef __ICC // icpc defines the __clang__ macro
#pragma warning(push)
#pragma warning(disable : 161 1682)
#else // __ICC
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpadded"
#pragma clang diagnostic ignored "-Wswitch-enum"
#pragma clang diagnostic ignored "-Wcovered-switch-default"
#endif
#elif defined __GNUC__
// Because REQUIREs trigger GCC's -Wparentheses, and because still
// supported version of g++ have only buggy support for _Pragmas,
// Wparentheses have to be suppressed globally.
#pragma GCC diagnostic ignored "-Wparentheses" // See #674 for details

#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#pragma GCC diagnostic ignored "-Wpadded"
#endif
#endif

using namespace time_literals;

WorkItemExample::WorkItemExample() : ModuleParams(nullptr),
									 ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

WorkItemExample::~WorkItemExample()

{

	isSwitchOn = false;

	///close(_serial_fd);

	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool WorkItemExample::init()
{

	ScheduleOnInterval(1000_ms); // 1000 us interval, 1000 Hz rate

	return true;
}

void WorkItemExample::Run()
{

	if (should_exit())
	{
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

char kart_baglanti[14];
char low_side_output_ayarla[14];
char kanal_on[14];
char kanal_off[14];


kart_baglanti[0]= 0x03;
kart_baglanti[1]= 0x01;
kart_baglanti[2]= 0x00;
kart_baglanti[3]= 0x00;
kart_baglanti[4]= 0x00;
kart_baglanti[5]= 0x00;
kart_baglanti[6]= 0x00;
kart_baglanti[7]= 0x00;
kart_baglanti[8]= 0x00;
kart_baglanti[9]= 0x00;
kart_baglanti[10]= 0x36; 
kart_baglanti[11]= 0xC7;
kart_baglanti[12]= 0xC6;
kart_baglanti[13]= 0x1F;



low_side_output_ayarla[0]= 0x03;
low_side_output_ayarla[1]= 0x02;
low_side_output_ayarla[2]= 0x00;
low_side_output_ayarla[3]= 0x02;
low_side_output_ayarla[4]= 0x00;
low_side_output_ayarla[5]= 0x00;
low_side_output_ayarla[6]= 0x00;
low_side_output_ayarla[7]= 0x00;
low_side_output_ayarla[8]= 0x00;
low_side_output_ayarla[9]= 0x00;
low_side_output_ayarla[10]= 0xDA; 
low_side_output_ayarla[11]= 0xEA;
low_side_output_ayarla[12]= 0xD4;
low_side_output_ayarla[13]= 0xB1;

kanal_on[0]= 0x03;
kanal_on[1]= 0x08;
kanal_on[2]= 0x00;
kanal_on[3]= 0x01;
kanal_on[4]= 0x00;
kanal_on[5]= 0x00;
kanal_on[6]= 0x00;
kanal_on[7]= 0x00;
kanal_on[8]= 0x00;
kanal_on[9]= 0x00;
kanal_on[10]= 0xD9; 
kanal_on[11]= 0x7A;
kanal_on[12]= 0x12;
kanal_on[13]= 0x15;


kanal_off[0]= 0x03;
kanal_off[1]= 0x08;
kanal_off[2]= 0x00;
kanal_off[3]= 0x00;
kanal_off[4]= 0x00;
kanal_off[5]= 0x00;
kanal_off[6]= 0x00;
kanal_off[7]= 0x00;
kanal_off[8]= 0x00;
kanal_off[9]= 0x00;
kanal_off[10]= 0x6D; 
kanal_off[11]= 0x71;
kanal_off[12]= 0x65;
kanal_off[13]= 0xB3;




#if 1

	int size = 0;

if(_serial_fd<0){

	_serial_fd = ::open("/dev/ttyS3", O_RDWR | O_NOCTTY);

		if (_serial_fd < 0) {
			PX4_ERR("GPS: failed to open serial port: %s err: %d", "" , errno);
			
		}
}

/* process baud rate */
	int speed;
	int _baudrate = 9600;
	speed = B9600; 

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_serial_fd, &uart_config);

	/* properly configure the terminal (see also https://en.wikibooks.org/wiki/Serial_Programming/termios ) */

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


	/* no parity, one stop bit, disable flow control */
	uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

	// Set: 8 data bits, enable receiver, ignore modem status lines.
	uart_config.c_cflag |= (CS8 | CREAD | CLOCAL);

	// Clear: echo, echo new line, canonical input and extended input.
	uart_config.c_lflag &= (ECHO | ECHONL | ICANON | IEXTEN);

	// Clear ONLCR flag (which appends a CR for every LF).
	uart_config.c_oflag &= ~ONLCR;


	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		PX4_ERR("ERR: %d (cfsetispeed)", termios_state);
		_serial_fd = -11;
	
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		PX4_ERR("ERR: %d (cfsetospeed)", termios_state);
			_serial_fd = -12;
		
	}

	if ((termios_state = tcsetattr(_serial_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("ERR: %d (tcsetattr)", termios_state);
			_serial_fd = -13;
		
	}



	if (_serial_fd < 0) {
			PX4_ERR("UART: failed to open serial port:");
			return;
		}else{
			PX4_INFO("Serial Fd %d",_serial_fd);
		}

	char sample_test_uart[25];// = {'S', 'A', 'M', 'P', 'L', 'E', ' ', '\n'};

	int i, n;

	uint64_t start_time = hrt_absolute_time();

	
	//	n = sprintf(sample_test_uart, "SAMPLE #1\n");

	size=write(_serial_fd, kart_baglanti,14);
	sleep(1);
	size=write(_serial_fd, low_side_output_ayarla,14);
	sleep(1);


	if(this->isSwitchOn == true){

		size = write(_serial_fd, kanal_on,14);
		sleep(1);
		isSwitchOn = true;
	}else{
		size = write(_serial_fd, kanal_off,14);
		sleep(1);
		isSwitchOn = false;
	}
	
#if 0
	int readSize = -1;
	int cntr = 0;

	const int max_timeout = 50;
	int timeout = 100;

	pollfd fds[1];
	fds[0].fd = _serial_fd;
	fds[0].events = POLLIN;

	int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), math::min(max_timeout, timeout));

	if (ret > 0) {
		/* if we have new data from GPS, go handle it */
		if (fds[0].revents & POLLIN) {
			/*
			 * We are here because poll says there is some data, so this
			 * won't block even on a blocking device. But don't read immediately
			 * by 1-2 bytes, wait for some more data to save expensive read() calls.
			 * If we have all requested data available, read it without waiting.
			 * If more bytes are available, we'll go back to poll() again.
			 */
			const unsigned character_count = 32; // minimum bytes that we want to read
			unsigned baudrate = _baudrate == 0 ? 115200 : _baudrate;
			const unsigned sleeptime = character_count * 1000000 / (baudrate / 10);

#ifdef __PX4_NUTTX
			int err = 0;
			int bytes_available = 0;
			err = ioctl(_serial_fd, FIONREAD, (unsigned long)&bytes_available);

			if (err != 0 || bytes_available < (int)character_count) {
				px4_usleep(sleeptime);
			}

#else
			px4_usleep(sleeptime);
#endif

			ret = ::read(_serial_fd, sample_test_uart, sizeof(sample_test_uart));

			ret < 0 ? :	write(_serial_fd,sample_test_uart,ret);

		} else {
			ret = -1;
		}
	}

#endif

#endif


	PX4_INFO(" position_setpoint_triplet.next.lat%d", _serial_fd);

	bool is_target_rec = false;
	bool isglobal_pos_rec = false;
	bool isgps_pos_rec = false;
	bool is_wind_est_rec = false;
	bool is_air_data_rec = false;

	if (_position_setpoint_triplet_s.updated())
	{

		if (_position_setpoint_triplet_s.copy(&position_setpoint_triplet))
		{
		}
	}
/*
	PX4_INFO(" position_setpoint_triplet.next.lat%f", position_setpoint_triplet.next.lat);
	PX4_INFO(" position_setpoint_triplet.next.lon%f", position_setpoint_triplet.next.lon);
	PX4_INFO(" position_setpoint_triplet.next.alt%f", position_setpoint_triplet.next.alt);
	PX4_INFO(" position_setpoint_triplet.next.type%d", position_setpoint_triplet.next.type);

	PX4_INFO(" position_setpoint_triplet.current.lat%f", position_setpoint_triplet.current.lat);
	PX4_INFO(" position_setpoint_triplet.current.lon%f", position_setpoint_triplet.current.lon);
	PX4_INFO(" position_setpoint_triplet.current.alt%f", position_setpoint_triplet.current.alt);
	PX4_INFO(" position_setpoint_triplet.current.type%d", position_setpoint_triplet.current.type);

	PX4_INFO(" position_setpoint_triplet.previous.lat%f", position_setpoint_triplet.previous.lat);
	PX4_INFO(" position_setpoint_triplet.previous.lon%f", position_setpoint_triplet.previous.lon);
	PX4_INFO(" position_setpoint_triplet.previous.alt%f", position_setpoint_triplet.previous.alt);
	PX4_INFO(" position_setpoint_triplet.previous.type%d", position_setpoint_triplet.previous.type);

*/
	if (_togan_hedef.updated())
	{

		if (_togan_hedef.copy(&togan_hedef_data))
		{
			is_target_rec = true;
		
		}
	}

	PX4_INFO("target.lat:%f", togan_hedef_data.lat);
	PX4_INFO("target.lon:%f", togan_hedef_data.lon);
	PX4_INFO("target.alt:%f", togan_hedef_data.alt);

	if (_my_global_pos.updated())
	{

		if (_my_global_pos.copy(&my_global_pos))
		{
			isglobal_pos_rec = true;
	//		PX4_INFO("my_global_pos.lat:%f", my_global_pos.lat);
		//	PX4_INFO("my_global_pos.lon:%f", my_global_pos.lon);
	//		PX4_INFO("my_global_pos.alt:%f", my_global_pos.alt);
		}
	}

	if (_my_gps_pos.updated())
	{

		if (_my_gps_pos.copy(&my_gps_pos))
		{
			isgps_pos_rec = true;
		//	PX4_INFO("my_gps_pos.heading:%f", my_gps_pos.heading);
	//		PX4_INFO("my_gps_pos.vel_n_m_s:%f", my_gps_pos.vel_n_m_s);
		//	PX4_INFO("my_gps_pos.vel_e_m_s:%f", my_gps_pos.vel_e_m_s);
		//	PX4_INFO("my_gps_pos.vel_d_m_s%f", my_gps_pos.vel_d_m_s);
		//	PX4_INFO("my_gps_pos.vel_m_s%f", my_gps_pos.vel_m_s);
		}
	}
	if (_wind_est_rec.updated())
	{
		if (_wind_est_rec.copy(&wind_est_rec))
		{
			is_wind_est_rec = true;
		//	PX4_INFO("wind_est_rec.windspeed_north:%f", wind_est_rec.windspeed_north);
		//	PX4_INFO("wind_est_rec.windspeed_east:%f", wind_est_rec.windspeed_east);
		}
	}

	if (_vehicle_air_data.updated())
	{
		if (_vehicle_air_data.copy(&vehicle_air_data_d))
		{
			is_air_data_rec = true;
		//	PX4_INFO("vehicle_air_data_d.baro_pressure_pa: %lf", vehicle_air_data_d.baro_pressure_pa);
		//	PX4_INFO("vehicle_air_data_d.baro_temp_celcius: %lf", vehicle_air_data_d.baro_temp_celcius);
		//	PX4_INFO("vehicle_air_data_d.rho: %lf", vehicle_air_data_d.rho);
		}
	}

	if (isgps_pos_rec && isglobal_pos_rec && is_air_data_rec)
	{
	//	CalculateAndPublishTOGANLAR();
	}

	perf_end(_loop_perf);
}

void WorkItemExample::CalculateAndPublishTOGANLAR()
{

#if 1

	//float start_time = time.time();

	float xtilde[6];
	float xhat[6];
	float aircraftGroundTrack_der;
	//float triangle_rad;
	float windspeed_ms[2];
	float timestep_s = 0.01;
	float cartposenu_m[3];
	float ucakdowncross_m[2];
	float velne_ms[2];
	bool is_valid = false;
	bool is_air_dens_valid = false;
	bool is_air_temp_valid = false;
	bool is_air_press_valid = false;
	bool isDlzValid = false;
	float deltaMomentArm_m[3];
#if 0
	uint16_t message15T_DezEntryRadius_m;
	uint16_t message15T_DezEntryStartAngle_deg;
	uint16_t message15T_DezEntryStopAngle_deg;
	uint16_t message15T_DlzEntryRadius_m;
	uint16_t message15T_DlzEntryStartAngle_deg;
	uint16_t message15T_DlzEntryStopAngle_deg;
	uint16_t message15T_DlzExitRadius_m;
	uint16_t message15T_DlzExitStartAngle_deg;
	uint16_t message15T_DlzExitStopAngle_deg;
	uint16_t message15T_RangeToTarget_m;
	//uint16_t message15T_releaseHeading_deg;
	uint16_t message15T_TimeToDezDlz;
	uint16_t message15T_TimeToImpact;
	//uint16_t message15T_safetyRadius_m;
#endif
	float message2R_velocityx_ms, message2R_velocityy_ms, message2R_velocityz_ms, message2R_latitude_deg,
		message2R_longitude_deg, message2R_altitude_m;
	float message2R_wanderangle_deg, message2R_trueheading_deg, message2R_pitch_deg, message2R_roll_deg;
	float message9R_momentarmx_m, message9R_momentarmy_m, message9R_momentarmz_m;
	float message9R_pitchoffset_deg, message9R_yawoffset_deg, message9R_rolloffset_deg;
	float message15R_windspeede_ms = 0, message15R_windspeedn_ms = 0; // message15R_tas_ms{0};
	float message15R_airtemperature_c{0}, message15R_airpressure_pa{0}, message15R_airdensity_kgm3 = 0.0f;
	float message15R_airtemperature_k = 0;
	float message17R_latitude = 0, message17R_longitude = 0, message17R_altitude = 0, message17R_impactangle = 0, message17R_impactazimuth = 0;
	float message15T_tossRange_m, message15T_releaseDownrange_m, message15T_releaseCrossrange_m;
	float message15T_tossTime_s, message15T_releaseTime_s, message15T_tof_s;
	float message15T_safetyRadius_m, message15T_dlzEntry_m, message15T_dlzExit_m;
	float message15T_dlzPlatform_m, message15T_releaseHeading_deg, message15T_steeringAngle_deg;
	float local_releaseTime_s{-1.1f};
	float ctsstime_ms{-1.0f};
	float rangeToTarget_m, crossSapmasi_der, releaseRange_m, groundSpeed_ms;
	float altitudeAboveTarget_ft;
	float min_alt_ft, max_alt_ft, min_speed_kt, max_speed_kt;
	float abs_wind_speed_ms, wind_limit_ms;
	float temperatureCorrection, pressureCorrection, densityCorrection;

	int dlzTimeDelay_s = 1 +			   /*this->pContext->GetReleaseDurationMs() / 1000 +*/
						 DLZTIMEDELAY_SEC; // DLZ araligi pickle farkindaligi yerine atis araligi gosterimini ifade ediyor.

	if (true /* == this->pNavModule->IsFreshDataExist()*/)
	{
#if 0
		if (position_setpoint_triplet.next.type == position_setpoint_triplet.next.SETPOINT_TYPE_FOLLOW_TARGET)
		{

			message17R_altitude = position_setpoint_triplet.next.alt - 100; // my_global_pos.alt;
			message17R_latitude = position_setpoint_triplet.next.lat;	   // my_global_pos.lat + 0.0001;
			message17R_longitude = position_setpoint_triplet.next.lon;	   //my_global_pos.lon;
			message17R_impactangle = 0;
			message17R_impactazimuth = 0;
		}
		else if (position_setpoint_triplet.current.type == position_setpoint_triplet.current.SETPOINT_TYPE_FOLLOW_TARGET)
		{
			
			message17R_altitude = position_setpoint_triplet.current.alt - 100; // my_global_pos.alt;
			message17R_latitude = position_setpoint_triplet.current.lat;	   // my_global_pos.lat + 0.0001;
			message17R_longitude = position_setpoint_triplet.current.lon;	   //my_global_pos.lon;
			message17R_impactangle = 0;
			message17R_impactazimuth = 0;
		} else{
			message17R_altitude = 0;//position_setpoint_triplet.current.alt - 100; // my_global_pos.alt;
			message17R_latitude = 0;//position_setpoint_triplet.current.lat;	   // my_global_pos.lat + 0.0001;
			message17R_longitude = 0; //position_setpoint_triplet.current.lon;	   //my_global_pos.lon;
			message17R_impactangle = 0;
			message17R_impactazimuth = 0;
		}
#endif
			message17R_altitude = togan_hedef_data.alt - 100; // my_global_pos.alt;
			message17R_latitude = togan_hedef_data.lat;	   // my_global_pos.lat + 0.0001;
			message17R_longitude = togan_hedef_data.lon;	   //my_global_pos.lon;
			message17R_impactangle = 0;
			message17R_impactazimuth = 0;


		message2R_latitude_deg = my_global_pos.lat;	 // this->pNavModule->GetLastACKinematicData()->GetLatitudeRad(&is_valid) * RAD_TO_DEG;
		message2R_longitude_deg = my_global_pos.lon; //this->pNavModule->GetLastACKinematicData()->GetLongitudeRad(&is_valid) * RAD_TO_DEG;
		//message2R_velocityx_ms = this->pNavModule->GetLastACKinematicData()->GetVelocityXMeterPerSec(&is_valid);
		//message2R_velocityy_ms = this->pNavModule->GetLastACKinematicData()->GetVelocityYMeterPerSec(&is_valid);
		//message2R_velocityz_ms = this->pNavModule->GetLastACKinematicData()->GetVelocityZMeterPerSec(&is_valid);

		message2R_altitude_m = my_global_pos.alt;		//->pNavModule->GetLastACKinematicData()->GetAltitudeMeters(&is_valid);
														//	message2R_wanderangle_deg = this->pNavModule->GetLastACKinematicData()->GetWanderAngleRad(&is_valid) * RAD_TO_DEG;
		message2R_trueheading_deg = my_gps_pos.heading; //  this->pNavModule->GetLastACKinematicData()->GettrueHeadingRad(&is_valid) * RAD_TO_DEG;
		message2R_pitch_deg = 0.0f;						//vehicle_attitude_setpoint den al;  this->pNavModule->GetLastACKinematicData()->GetPitchRad(&is_valid) * RAD_TO_DEG;
		message2R_roll_deg = 0.0f;						//vehicle_attitude_setpoint den al; this->pNavModule->GetLastACKinematicData()->GetRollRad(&is_valid) * RAD_TO_DEG;

		velne_ms[0] = my_gps_pos.vel_n_m_s;
		velne_ms[1] = my_gps_pos.vel_e_m_s;

		double absVel = my_gps_pos.vel_m_s;

		if (absVel > MAX_VEL_DRONES_MS)
		{
			//hiz max hizdan buyukse atis yaptirmiyoruz
			local_releaseTime_s = 88888.0f;
			message15T_steeringAngle_deg = 88888.0f;
		}
		else
		{ //if (absVel > MIN_VEL_DRONES_MS) {

			/////////////////////////////////////////////
			firecontrol_llh2cart(cartposenu_m, message17R_latitude, message17R_longitude, message17R_altitude,
								 message2R_latitude_deg, message2R_longitude_deg, message2R_altitude_m);
			rangeToTarget_m = pow((pow(cartposenu_m[1], (float)2.0f) + pow(cartposenu_m[0], (float)2.0)), (float)0.5);

			local_releaseTime_s = 99999.0f;
			message15T_steeringAngle_deg = 99999.0f;

			/*
						if ((pContext->GetIsSATEValid() == true) && (rangeToTarget_m <= SATE_UNCLEAR_RADIUS_METER)) {

							//pContext->SetSATEValid(false);
							local_releaseTime_s = 0;
							message15T_steeringAngle_deg = 0;

							message15T_dlzEntry_m = 10;
							message15T_dlzExit_m = 10;
							message15T_dlzPlatform_m = 0;
							message15T_releaseTime_s = 0;
							message15T_tossRange_m = 0;
							message15T_tossTime_s = 0;
							message15T_tof_s = 0;
							message15T_safetyRadius_m = 0;
							message15T_releaseHeading_deg = 99990.0f;

						} else if (!pContext->GetIsSATEValid()) {

							// bir defa circle disina ciktik mi?
							if (rangeToTarget_m > SATE_CLEAR_RADIUS_METER) {
								pContext->SetSATEValid(true);

							}

							local_releaseTime_s = 99999.0f;
							message15T_steeringAngle_deg = 99999.0f;

						} else {

			if (this->pContext->GetPlatform() == PLATFORM_DRONES) {
				//TODO zarf degerlerini drondan alal�m

			} else if (this->pContext->GetPlatform() == PLATFORM_HURKUS_LEFT
				   || this->pContext->GetPlatform() == PLATFORM_HURKUS_RIGHT) {
				//zarf degerlerini hurkustan alal�m
			}

			this->pNavModule->SetACDataisConsumed();
*/

			message9R_momentarmx_m = 0; //MomentArmContract.GetMomentArmXMeters(&is_valid);

			if (!is_valid)
			{
				message9R_momentarmx_m = 0;
			}

			message9R_momentarmy_m = 0; // MomentArmContract.GetMomentArmYMeters(&is_valid);

			if (!is_valid)
			{
				message9R_momentarmy_m = 0;
			}

			message9R_momentarmz_m = 0; //MomentArmContract.GetMomentArmZMeters(&is_valid);

			if (!is_valid)
			{
				message9R_momentarmz_m = 0;
			}

			message9R_pitchoffset_deg = 0; // MomentArmContract.GetPitchOffsetRad(&is_valid) * RAD_TO_DEG;

			if (!is_valid)
			{
				message9R_pitchoffset_deg = 0;
			}

			message9R_yawoffset_deg = 0; //MomentArmContract.GetYawOffsetRad(&is_valid) * RAD_TO_DEG;

			if (!is_valid)
			{
				message9R_yawoffset_deg = 0;
			}

			message9R_rolloffset_deg = 0; // MomentArmContract.GetRollOffsetRad(&is_valid) * RAD_TO_DEG;

			if (!is_valid)
			{
				message9R_rolloffset_deg = 0;
			}

			message15R_windspeedn_ms = 0; //wind_est_rec.windspeed_north;

			/* 	if (!is_valid)
			{
				message15R_windspeedn_ms = 0;
			}
 */
			message15R_windspeede_ms = 0; // wind_est_rec.windspeed_east;

			/* 	if (!is_valid) {
				message15R_windspeede_ms = 0;
			} */

			message15R_airtemperature_c = vehicle_air_data_d.baro_temp_celcius; //  EnvDataContract.GetAirTemperatureDegreesCelcius(     &is_air_temp_valid); // todo: Invalid olma durumunda standart gun hesabindan alinacak.

			is_air_temp_valid = false;

			if (is_air_temp_valid == true)
			{
				message15R_airtemperature_k = message15R_airtemperature_c + 273.15f;
				temperatureCorrection = message15R_airtemperature_k - firecontrol_getairtemperature(message2R_altitude_m);
			}
			else
			{
				message15R_airtemperature_k = firecontrol_getairtemperature(message2R_altitude_m);
				temperatureCorrection = 0.0f;
			}

			message15R_airpressure_pa = vehicle_air_data_d.baro_pressure_pa; /// (EnvDataContract.GetAirPressureKiloPascal(&is_air_press_valid)) * kKilopascalToPascal;
			pressureCorrection = message15R_airpressure_pa / firecontrol_getairpressure(message15R_airtemperature_k);
			is_air_press_valid = false;

			if (is_air_press_valid != true)
			{
				message15R_airpressure_pa = firecontrol_getairpressure(message15R_airtemperature_k);
				pressureCorrection = 1.0f;
			}

			//	message15R_airdensity_kgm3 =  vehicle_air_data_d.rho; // EnvDataContract.GetAirDensityKgM3(&is_air_dens_valid);
			//	densityCorrection = message15R_airdensity_kgm3 / firecontrol_getairdensity(message15R_airtemperature_k, message15R_airpressure_pa);
			is_air_press_valid = false;
			if (is_air_dens_valid != true)
			{
				message15R_airdensity_kgm3 = firecontrol_getairdensity(message15R_airtemperature_k, message15R_airpressure_pa);
				densityCorrection = 1.0f;
			}

			/*
			message17R_altitude = TargetDataContract.GetAltitudeMeters(&is_valid);
			message17R_latitude = TargetDataContract.GetLatitudeRad(&is_valid) * RAD_TO_DEG;
			message17R_longitude = TargetDataContract.GetLongitudeRad(&is_valid) * RAD_TO_DEG;
			message17R_impactangle = TargetDataContract.GetImpactAngleRad(&is_valid) * RAD_TO_DEG;
			message17R_impactazimuth = TargetDataContract.GetBearingToTargetRad(&is_valid) * RAD_TO_DEG;

			*/

			/////////////////////////////////////////////
			firecontrol_llh2cart(cartposenu_m, message17R_latitude, message17R_longitude, message17R_altitude,
								 message2R_latitude_deg, message2R_longitude_deg, message2R_altitude_m);
			rangeToTarget_m = pow((pow(cartposenu_m[1], (float)2.0f) + pow(cartposenu_m[0], (float)2.0)), (float)0.5);

			aircraftGroundTrack_der = (float)(90.0f - (atan2f(velne_ms[0], velne_ms[1]) * (float)kRadiansToDegrees));
			groundSpeed_ms = pow((pow(velne_ms[1], (float)2.0f) + pow(velne_ms[0], (float)2.0)),
								 (float)0.5);

			windspeed_ms[0] = message15R_windspeedn_ms;
			windspeed_ms[1] = message15R_windspeede_ms;

			firecontrol_momentarm2ned(deltaMomentArm_m, localmomentarmx_m,
									  localmomentarmy_m, localmomentarmz_m, message9R_rolloffset_deg, message9R_pitchoffset_deg, message9R_yawoffset_deg);
			message9R_momentarmx_m = message9R_momentarmx_m + deltaMomentArm_m[0];
			message9R_momentarmy_m = message9R_momentarmy_m + deltaMomentArm_m[1];
			message9R_momentarmz_m = message9R_momentarmz_m + deltaMomentArm_m[2];
			firecontrol_momentarm2ned(deltaMomentArm_m, message9R_momentarmx_m, message9R_momentarmy_m, message9R_momentarmz_m,
									  message2R_roll_deg, message2R_pitch_deg, message2R_trueheading_deg);
			xhat[0] = deltaMomentArm_m[0];						  // north
			xhat[1] = message2R_altitude_m - deltaMomentArm_m[2]; // up
			xhat[2] = deltaMomentArm_m[1];						  // east
			xhat[3] = velne_ms[0];
			xhat[4] = -my_gps_pos.vel_d_m_s; //todo:
			xhat[5] = velne_ms[1];

			firecontrol_forwardsimulation(xhat, message17R_altitude, temperatureCorrection, pressureCorrection, densityCorrection,
										  windspeed_ms, timestep_s, message2R_latitude_deg);
			memcpy(xtilde, xhat, sizeof(xtilde));
			firecontrol_eastnorth2downcross(ucakdowncross_m, xtilde[2], xtilde[0], aircraftGroundTrack_der);

			message15T_releaseDownrange_m = ucakdowncross_m[0];
			message15T_releaseCrossrange_m = ucakdowncross_m[1];

			message15T_releaseHeading_deg = 90.0f - (atan2(cartposenu_m[1], cartposenu_m[0]) * kRadiansToDegrees);
			crossSapmasi_der = asin(message15T_releaseCrossrange_m / rangeToTarget_m) * kRadiansToDegrees;
			message15T_releaseHeading_deg = 180.0f + message15T_releaseHeading_deg - crossSapmasi_der;

			message15T_tossRange_m = message15T_releaseDownrange_m + (groundSpeed_ms * (float)(tossTimeAdvance_s + dlzTimeAdvance_s));

			releaseRange_m = pow((pow(message15T_releaseDownrange_m, (float)2.0f) + pow(message15T_releaseCrossrange_m,
																						(float)2.0)),
								 (float)0.5);

			while (message15T_releaseHeading_deg > 360.0f)
			{
				message15T_releaseHeading_deg = message15T_releaseHeading_deg - 360.0f;
			}

			while (message15T_releaseHeading_deg <= 0.0f)
			{
				message15T_releaseHeading_deg = message15T_releaseHeading_deg + 360.0f;
			}

			message15T_steeringAngle_deg = (message15T_releaseHeading_deg - aircraftGroundTrack_der);

			while (message15T_steeringAngle_deg > 180.0f)
			{
				message15T_steeringAngle_deg = message15T_steeringAngle_deg - 360.0f;
			}

			while (message15T_steeringAngle_deg <= -180.0f)
			{
				message15T_steeringAngle_deg = message15T_steeringAngle_deg + 360.0f;
			}

			firecontrol_eastnorth2downcross(ucakdowncross_m, cartposenu_m[0], cartposenu_m[1], aircraftGroundTrack_der);

			message15T_tof_s = xtilde[5];
			message15T_safetyRadius_m = fabs(sin(message15T_steeringAngle_deg * DEG_TO_RAD)) * releaseRange_m;
			message15T_dlzEntry_m = message15T_releaseDownrange_m + (float)(dlzTimeAdvance_s * groundSpeed_ms);
			message15T_dlzExit_m = message15T_releaseDownrange_m - (dlzTimeDelay_s * groundSpeed_ms);
			message15T_dlzPlatform_m = rangeToTarget_m * cos(crossSapmasi_der * kDegreesToRadians);

			if (groundSpeed_ms > 0.05)
			{
				local_releaseTime_s = (message15T_dlzPlatform_m - message15T_releaseDownrange_m) / groundSpeed_ms;
			}
			else
			{
				local_releaseTime_s = 987987.0;
			}
/*
			PX4_INFO("groundSpeed_ms%f", groundSpeed_ms);
			PX4_INFO("message15T_dlzPlatform_m%f", message15T_dlzPlatform_m);
			PX4_INFO("message15T_releaseDownrange_m%f", message15T_releaseDownrange_m);
*/
			if (message15T_dlzPlatform_m >= message15T_dlzEntry_m)
			{

				message15T_releaseTime_s = local_releaseTime_s - dlzTimeAdvance_s; // release time artik time to DLZ oldu.
			}
			else if (message15T_dlzPlatform_m > message15T_dlzExit_m)
			{

				message15T_releaseTime_s = local_releaseTime_s + dlzTimeDelay_s;
			}
			else
			{
				message15T_releaseTime_s = 0;
			}

			message15T_tossTime_s = local_releaseTime_s - tossTimeAdvance_s - dlzTimeAdvance_s;
			//}
		}

		//		// todo: bunu dusun
		//		else if (absVel <= MIN_VEL_DRONES_MS) {
		//			//hiz min hizdan kucukse dogrudan atis yapiyoruz
		//			local_releaseTime_s = -(dlzTimeDelay_s) -dlzTimeAdvance_s;
		//			message15T_steeringAngle_deg = 99999.0f;
		//
		//		}

/*		PX4_INFO("local_releaseTime_s %f", local_releaseTime_s);

		PX4_INFO("dlzTimeDelay_s %d", dlzTimeDelay_s);
*/
		if ((local_releaseTime_s) < -(dlzTimeDelay_s))
		{
			// hedefe at�� ��z�m�nden dlz ��k���ndan daha fazla yakla��rsa at�� zaman�n� s�rekli �teleyerek m�himmat� salmaz
			ctsstime_ms = /*(time.time()) +*/ dlzTimeAdvance_s * S_MS;
		}
		else
		{
			ctsstime_ms = /*(time.time()) +*/ ((local_releaseTime_s)*S_MS) - ctssAdvanceTime_ms;
		}

		// Daha sonra kullan�lacak
		if (fabs(message15T_steeringAngle_deg) < dlzToleranceAngle_deg)
		{
			isDlzValid = true;
		}
		else
		{
			isDlzValid = false;
		}

		if (isDlzValid == false)
		{
			message15T_dlzEntry_m = 0;
			message15T_dlzExit_m = 0;
			message15T_dlzPlatform_m = 0;
			message15T_releaseTime_s = 0;
			message15T_tossRange_m = 0;
			message15T_tossTime_s = 0;
			message15T_tof_s = 0;
			message15T_safetyRadius_m = 0;
		}

		//this->pContext->SetCTSSTimeMs(ctsstime_ms);

		// 20201111 hURKUS sel
		//message15T_releaseTime_s = local_releaseTime_s - dlzTimeAdvance_s;
		//if((message15T_releaseTime_s < 0) && (message15T_releaseTime_s > - (dlzTimeDelay_s + dlzTimeAdvance_s))){
		if ((local_releaseTime_s < dlzTimeAdvance_s) && (local_releaseTime_s > -(dlzTimeDelay_s)))
		{

			//	LARcontract.SetFCSStatus(true, true);
		}
		else
		{
			//	LARcontract.SetFCSStatus(true, false);
		}

		altitudeAboveTarget_ft = (message2R_altitude_m - message17R_altitude) * METER_TO_FEET;

		if (altitudeAboveTarget_ft < ALT_LOW_FT)
		{
			//	LARcontract.SetAltitudeLow(true, true);
		}

		if (message2R_altitude_m * METER_TO_FEET > ALT_HIGH_FT)
		{
			//	LARcontract.SetAltitudeHigh(true, true);
		}

		if (message2R_pitch_deg < PITCH_LIMIT_LOW_DEG || message2R_pitch_deg > PITCH_LIMIT_HIGH_DEG)
		{
			//	LARcontract.SetPitchLimitOOBI(true, true);
		}

		if (message2R_roll_deg < ROLL_LIMIT_LOW_DEG || message2R_roll_deg > ROLL_LIMIT_HIGH_DEG)
		{
			//	LARcontract.SetRollLimitOOBI(true, true);
		}

		abs_wind_speed_ms = pow((pow(message15R_windspeede_ms, (float)2.0f) + pow(message15R_windspeedn_ms, (float)2.0)),
								(float)0.5);

		wind_limit_ms = KT2MS * (100 * (message2R_altitude_m * METER_TO_FEET + 4000) / 44000);

		if (abs_wind_speed_ms > wind_limit_ms)
		{

			//	LARcontract.SetWindSpeedLimitOOBI(true, true);
		}

		PX4_INFO("CTSS TIME %f", ctsstime_ms);

		// CTSS degerini de set edelim.
		//		LARcontract.SetCpuTimeMs(true, time.time());
		//		LARcontract.SetTossRange(true, message15T_tossRange_m);
		//		LARcontract.SetGroundSpeedKT(true, groundSpeed_ms * MS2KT);
		//		LARcontract.SetAltitudeAboveTargetFeet(true, altitudeAboveTarget_ft);
		//		LARcontract.SetTossTime(true, message15T_tossTime_s);
		//		LARcontract.SetReleaseTime(true, message15T_releaseTime_s);
		//		LARcontract.SetTimeofFlight(true, message15T_tof_s);
		//		LARcontract.SetSafetyFootprintRadius(true, message15T_safetyRadius_m);
		//
		//		LARcontract.SetDLZEntry(true, message15T_dlzEntry_m);
		//		LARcontract.SetDLZExit(true, message15T_dlzExit_m);
		//		LARcontract.SetDLZPlatform(true, message15T_dlzPlatform_m);
		//		LARcontract.SetReleaseHeadingSemicircle(true, CMathLib::FixPlusPIMinusPI(message15T_releaseHeading_deg * DEG_TO_RAD) * RAD_TO_SEMICIRCLE);
		//		LARcontract.SetSteeringAngleSemicircle(true, CMathLib::FixPlusPIMinusPI(message15T_steeringAngle_deg * DEG_TO_RAD) * RAD_TO_SEMICIRCLE);
		//		LARcontract.SetCalculationTime(true, local_releaseTime_s);
		//		LARcontract.SetCTSSTimeAlone(true, ctsstime_ms - time.time());

		/*
		 Inputs:
		 message15T_tossRange_m;
		 message15T_releaseHeading_deg;
		 message15T_dlzEntry_m;
		 message15T_dlzExit_m;
		 message15T_dlzPlatform_m;
		 message15T_tossTime_s;
		 message15T_releaseTime_s;
		 message15T_tof_s;
		 message15T_safetyRadius_m;

		 Outputs:
		 message15T_DezEntryRadius_m;
		 message15T_DezEntryStartAngle_deg;
		 message15T_DezEntryStopAngle_deg;
		 message15T_DlzEntryRadius_m;
		 message15T_DlzEntryStartAngle_deg;
		 message15T_DlzEntryStopAngle_deg;
		 message15T_DlzExitRadius_m;
		 message15T_DlzExitStartAngle_deg;
		 message15T_DlzExitStopAngle_deg;
		 message15T_RangeToTarget_m;
		 message15T_releaseHeading_deg;
		 message15T_TimeToDezDlz;
		 message15T_TimeToImpact;
		 message15T_safetyRadius_m;
		 */

		/*

		if (message15T_releaseHeading_deg < 99990.0f){
			message15T_DezEntryRadius_m = message15T_tossRange_m;

			message15T_DezEntryStartAngle_deg = message15T_releaseHeading_deg + 180.0f - DEZ_ARC_WIDTH_DEG *0.5f;

			// Start and stop angles should be within 0 to 359 degress closed interval!
			while (message15T_DezEntryStartAngle_deg >= 360.0f){
				message15T_DezEntryStartAngle_deg = message15T_DezEntryStartAngle_deg - 360.0f;
			}

			while (message15T_DezEntryStartAngle_deg < 0.0f){
				message15T_DezEntryStartAngle_deg = message15T_DezEntryStartAngle_deg + 360.0f;
			}

			message15T_DezEntryStopAngle_deg = message15T_releaseHeading_deg + 180.0f + DEZ_ARC_WIDTH_DEG *0.5f;

			while (message15T_DezEntryStopAngle_deg >= 360.0f){
				message15T_DezEntryStopAngle_deg = message15T_DezEntryStopAngle_deg - 360.0f;
			}

			while (message15T_DezEntryStopAngle_deg < 0.0f){
				message15T_DezEntryStopAngle_deg = message15T_DezEntryStopAngle_deg + 360.0f;
			}

			message15T_DlzEntryRadius_m = message15T_dlzEntry_m;

			message15T_DlzEntryStartAngle_deg = message15T_releaseHeading_deg + 180.0f - DLZ_ENTRY_ARC_WIDTH_DEG *0.5f;

			while (message15T_DlzEntryStartAngle_deg >= 360.0f){
				message15T_DlzEntryStartAngle_deg = message15T_DlzEntryStartAngle_deg - 360.0f;
			}

			while (message15T_DlzEntryStartAngle_deg < 0.0f){
				message15T_DlzEntryStartAngle_deg = message15T_DlzEntryStartAngle_deg + 360.0f;
			}

			message15T_DlzEntryStopAngle_deg = message15T_releaseHeading_deg + 180.0f + DLZ_ENTRY_ARC_WIDTH_DEG *0.5f;

			while (message15T_DlzEntryStopAngle_deg >= 360.0f){
				message15T_DlzEntryStopAngle_deg = message15T_DlzEntryStopAngle_deg - 360.0f;
			}

			while (message15T_DlzEntryStopAngle_deg < 0.0f){
				message15T_DlzEntryStopAngle_deg = message15T_DlzEntryStopAngle_deg + 360.0f;
			}

			message15T_DlzExitRadius_m = fabs(message15T_dlzExit_m);
//			uint16_t RangeToTargetMeters;

			if (message15T_dlzExit_m >= 0.0f){
				message15T_DlzExitStartAngle_deg = message15T_releaseHeading_deg + 180.0f - DLZ_EXIT_ARC_WIDTH_DEG *0.5f;

				while (message15T_DlzExitStartAngle_deg >= 360.0f){
					message15T_DlzExitStartAngle_deg = message15T_DlzExitStartAngle_deg - 360.0f;
				}

				while (message15T_DlzExitStartAngle_deg < 0.0f){
					message15T_DlzExitStartAngle_deg = message15T_DlzExitStartAngle_deg + 360.0f;
				}

				message15T_DlzExitStopAngle_deg = message15T_releaseHeading_deg + 180.0f + DLZ_EXIT_ARC_WIDTH_DEG *0.5f;

				while (message15T_DlzExitStopAngle_deg >= 360.0f){
					message15T_DlzExitStopAngle_deg = message15T_DlzExitStopAngle_deg - 360.0f;
				}

				while (message15T_DlzExitStopAngle_deg < 0.0f){
					message15T_DlzExitStopAngle_deg = message15T_DlzExitStopAngle_deg + 360.0f;
				}
			}

			// If dlzexit is less than zero than mirror the arc with respect to the aimpoint!
			else {
				message15T_DlzExitStartAngle_deg = message15T_releaseHeading_deg - DLZ_EXIT_ARC_WIDTH_DEG *0.5f;

				while (message15T_DlzExitStartAngle_deg >= 360.0f){
					message15T_DlzExitStartAngle_deg = message15T_DlzExitStartAngle_deg - 360.0f;
				}

				while (message15T_DlzExitStartAngle_deg < 0.0f){
					message15T_DlzExitStartAngle_deg = message15T_DlzExitStartAngle_deg + 360.0f;
				}

				message15T_DlzExitStopAngle_deg = message15T_releaseHeading_deg + DLZ_EXIT_ARC_WIDTH_DEG *0.5f;

				while (message15T_DlzExitStopAngle_deg >= 360.0f){
					message15T_DlzExitStopAngle_deg = message15T_DlzExitStopAngle_deg - 360.0f;
				}

				while (message15T_DlzExitStopAngle_deg < 0.0f){
					message15T_DlzExitStopAngle_deg = message15T_DlzExitStopAngle_deg + 360.0f;
				}
			}

			message15T_RangeToTarget_m = message15T_dlzPlatform_m;

			//message15T_releaseHeading_deg = message15T_releaseHeading_deg;

			// Time to parameter is defined as time to DEZ entry before reaching dez, exceeding dez entry it is reverted to remaining time to dlz entry.
			if (message15T_RangeToTarget_m > message15T_DezEntryRadius_m) {
				message15T_TimeToDezDlz = message15T_tossTime_s;

			} else if (message15T_RangeToTarget_m > message15T_DlzExitRadius_m) {
				message15T_TimeToDezDlz = message15T_releaseTime_s;

			} else if (message15T_RangeToTarget_m > message15T_DlzExitRadius_m) {
				message15T_TimeToDezDlz = 0.0f;
			}

			//message15T_TimeToImpact = message15T_tossTime_s + message15T_releaseTime_s + message15T_tof_s;
			message15T_TimeToImpact = local_releaseTime_s + message15T_tof_s;

		} else {

			message15T_DezEntryRadius_m = 10;
			message15T_DezEntryStartAngle_deg = 1;
			message15T_DezEntryStopAngle_deg = 360;

			message15T_DlzEntryRadius_m = 5;
			message15T_DlzEntryStartAngle_deg = 1;
			message15T_DlzEntryStopAngle_deg = 360;

			message15T_DlzExitRadius_m = 5;
			message15T_DlzExitStartAngle_deg = 1;
			message15T_DlzExitStopAngle_deg = 360;

			message15T_RangeToTarget_m = rangeToTarget_m;

			//message15T_releaseHeading_deg = message15T_releaseHeading_deg;

			// Time to parameter is defined as time to DEZ entry before reaching dez, exceeding dez entry it is reverted to remaining time to dlz entry.
			//	if (message15T_RangeToTarget_m > message15T_DezEntryRadius_m) {
			//	message15T_TimeToDezDlz = message15T_tossTime_s;
			//} else if (message15T_RangeToTarget_m > message15T_DlzExitRadius_m) {
			//message15T_TimeToDezDlz = message15T_releaseTime_s;
			//} else if (message15T_RangeToTarget_m > message15T_DlzExitRadius_m) {
			message15T_TimeToDezDlz = 0.0f;
			//	}

			message15T_TimeToImpact = 0;			//message15T_tossTime_s + message15T_releaseTime_s + message15T_tof_s;
		}

		//message15T_safetyRadius_m = message15T_safetyRadius_m;

				LARcontract.SetCpuTimeMs(true, time.time());
				LARcontract.SetTossRange(true, message15T_tossRange_m);
				LARcontract.SetGroundSpeedKT(true, groundSpeed_ms * MS2KT);
				LARcontract.SetAltitudeAboveTargetFeet(true, altitudeAboveTarget_ft);
				LARcontract.SetTossTime(true, message15T_tossTime_s);
				LARcontract.SetReleaseTime(true, message15T_releaseTime_s);
				LARcontract.SetTimeofFlight(true, message15T_tof_s);
				LARcontract.SetSafetyFootprintRadius(true, message15T_safetyRadius_m);

				LARcontract.SetDLZEntry(true, message15T_dlzEntry_m);
				LARcontract.SetDLZExit(true, message15T_dlzExit_m);
				LARcontract.SetDLZPlatform(true, message15T_dlzPlatform_m);
				LARcontract.SetReleaseHeadingSemicircle(true, CMathLib::FixPlusPIMinusPI(message15T_releaseHeading_deg * DEG_TO_RAD) * RAD_TO_SEMICIRCLE);
				LARcontract.SetSteeringAngleSemicircle(true, CMathLib::FixPlusPIMinusPI(message15T_steeringAngle_deg * DEG_TO_RAD) * RAD_TO_SEMICIRCLE);
				LARcontract.SetCalculationTime(true, local_releaseTime_s);
				LARcontract.SetCTSSTimeAlone(true, ctsstime_ms - time.time());

				LARcontract.SetDEZEntryStartAngleDeg(true, message15T_DezEntryStartAngle_deg);
				LARcontract.SetDEZEntryStopAngleDeg(true, message15T_DezEntryStopAngle_deg);
				LARcontract.SetDEZEntryRadiusDeg(true, message15T_DezEntryRadius_m);

				LARcontract.SetDLZEntryStartAngleDeg(true, message15T_DlzEntryStartAngle_deg);
				LARcontract.SetDLZEntryStopAngleDeg(true, message15T_DlzEntryStopAngle_deg);
				LARcontract.SetDLZEntryRadiusDeg(true, message15T_DlzEntryRadius_m);

				LARcontract.SetDLZExitStartAngleDeg(true, message15T_DlzExitStartAngle_deg);
				LARcontract.SetDLZExitStopAngleDeg(true, message15T_DlzExitStopAngle_deg);
				LARcontract.SetDLZExitRadiusDeg(true, message15T_DlzExitRadius_m);

				LARcontract.SetRangeToTargetMeters(true, message15T_RangeToTarget_m);
				LARcontract.SetReleaseHeadingDegrees(true, (uint16_t) message15T_releaseHeading_deg);
				LARcontract.SetTimeToDEZ_DLZSecs(true, message15T_TimeToDezDlz);
				LARcontract.SetTimeToImpactSecs(true, message15T_TimeToImpact);
				LARcontract.SetSafetyFPRadiusMeters(true, (uint16_t) message15T_safetyRadius_m);

				CMessage* p_mes = GetMessage(CServiceIDs::LAR_TOGAN_SERVICE_ID);
				LARcontract.Serialize(p_mes);

				PublishService(p_mes);
		*/
	}
	else
	{
		//debugging purposes
		//LARcontract.Reset();
	}

#endif
}

// Calc state derivative
void WorkItemExample::firecontrol_pointmass(FP32 *ret_val, FP32 *xhat, FP32 *airspeed_ms, FP32 airDensity_kgm3, FP32 cd,
											FP32 latitude_rad)
{

	FP32 *dxhat = ret_val;

	double g0, azimuth_rads;
	double omega1, omega2, omega3, aeroTerm;
	g0 = 9.80665f * (1 - 0.0026f * std::cos(2.0f * latitude_rad));
	azimuth_rads = (M_PI_F / 2.0f) - atan2(xhat[3], xhat[5]);
	omega1 = omegaEarth_rads * cos(latitude_rad) * cos(azimuth_rads);
	omega2 = omegaEarth_rads * sin(latitude_rad);
	omega3 = -omegaEarth_rads * cos(latitude_rad) * sin(azimuth_rads);
	//aeroTerm = ( ( -M_PI_F*airDensity_kgm3*iForm*std::pow(dia_m , 2) ) / (8 * mass_kg) ) *cd*sqrt(std::pow(xhat[3], 2) + std::pow(xhat[4], 2) + std::pow(xhat[5], 2));
	aeroTerm = ((-M_PI_F * airDensity_kgm3 * iForm * (6561.0e-6f)) / (25.0f)) * cd * sqrt(std::pow(airspeed_ms[0], 2) + std::pow(airspeed_ms[1], 2) + std::pow(airspeed_ms[2], 2));
	dxhat[0] = xhat[3];
	dxhat[1] = xhat[4];
	dxhat[2] = xhat[5];
	dxhat[3] = aeroTerm * airspeed_ms[0] - g0 * xhat[0] / rEarth_m - 2 * (omega2 * xhat[5] - omega3 * xhat[4]);
	dxhat[4] = aeroTerm * airspeed_ms[1] - g0 * (1 - 2 * xhat[1] / rEarth_m) - 2 * (omega3 * xhat[3] - omega1 * xhat[5]);
	dxhat[5] = aeroTerm * airspeed_ms[2] - g0 * xhat[2] / rEarth_m - 2 * (omega1 * xhat[4] - omega2 * xhat[3]);
}

// Run forward simulation
void WorkItemExample::firecontrol_forwardsimulation(FP32 *initstate, FP32 targetheight_m, FP32 temperatureCorrection,
													FP32 pressureCorrection, FP32 densityCorrection, FP32 *windspeed_ms, FP32 timestep_s, FP32 message2R_latitude_deg)
{

	FP32 dxhat[6];
	FP32 airspeed_ms[3];

	FP32 time_s = 0.0f;
	bool isoverground = true;
	FP32 lastheight_m;
	FP32 trueairspeed_kt;
	FP32 machnumber;
	FP32 cd;
	FP32 latitude_rad;
	FP32 aradegerorani;
	FP32 airTemperature_K, airpressure_pa, airDensity_kgm3;
	latitude_rad = message2R_latitude_deg * kDegreesToRadians;

	while (isoverground)
	{
		lastheight_m = initstate[1];

		airspeed_ms[0] = initstate[3] - windspeed_ms[0];
		airspeed_ms[1] = initstate[4];
		airspeed_ms[2] = initstate[5] - windspeed_ms[1];
		trueairspeed_kt = sqrt(std::pow((airspeed_ms[0]), 2) + std::pow(airspeed_ms[1], 2) + std::pow((airspeed_ms[2]), 2)) * kMsnToKnot;
		airTemperature_K = temperatureCorrection + WorkItemExample::firecontrol_getairtemperature(initstate[1]);
		airpressure_pa = pressureCorrection * WorkItemExample::firecontrol_getairpressure(airTemperature_K);
		airDensity_kgm3 = densityCorrection * WorkItemExample::firecontrol_getairdensity(airTemperature_K, airpressure_pa);
		machnumber = WorkItemExample::firecontrol_getmachnumber(trueairspeed_kt, airTemperature_K);
		cd = WorkItemExample::firecontrol_getcd(machnumber);
		WorkItemExample::firecontrol_pointmass(dxhat, initstate, airspeed_ms, airDensity_kgm3, cd, latitude_rad);

		for (int i = 0; i <= 5; i++)
		{
			initstate[i] = initstate[i] + (dxhat[i] * timestep_s);
		}

		windspeed_ms[0] = windspeed_ms[0] * (initstate[1] + 1219.2) / (lastheight_m + 1219.2);
		windspeed_ms[1] = windspeed_ms[1] * (initstate[1] + 1219.2) / (lastheight_m + 1219.2);

		time_s = time_s + timestep_s;

		if (initstate[1] <= targetheight_m)
		{
			isoverground = false;
		}
	}

	aradegerorani = ((initstate[1] - targetheight_m) / (dxhat[1] * timestep_s));
	initstate[0] = initstate[0] - (dxhat[0] * aradegerorani * timestep_s);
	initstate[1] = targetheight_m;
	initstate[2] = initstate[2] - (dxhat[2] * aradegerorani * timestep_s);
	initstate[3] = initstate[3] - (dxhat[3] * aradegerorani * timestep_s);
	initstate[4] = initstate[4] - (dxhat[4] * aradegerorani * timestep_s);
	initstate[5] = initstate[5] - (dxhat[5] * aradegerorani * timestep_s);
	time_s = time_s - (aradegerorani * timestep_s);

	initstate[5] = time_s;
}

// Calc standart air temperature
FP32 WorkItemExample::firecontrol_getairtemperature(FP32 altitude_m)
{
	FP32 airTemperature_K = 288.15 - 0.0065 * altitude_m;
	return airTemperature_K;
}

// Calc standart air pressure
FP32 WorkItemExample::firecontrol_getairpressure(FP32 airTemperature_K)
{
	FP32 airPressure_Pa = 101325 * pow((airTemperature_K / 288.15), 5.2559);
	return airPressure_Pa;
}

// Calc standart air density
FP32 WorkItemExample::firecontrol_getairdensity(FP32 airTemperature_K, FP32 airPressure_Pa)
{
	FP32 RGas = 287.0f;
	FP32 airDensity_kgm3 = airPressure_Pa / (RGas * airTemperature_K);
	return airDensity_kgm3;
}

// Calc mach number
FP32 WorkItemExample::firecontrol_getmachnumber(FP32 trueairspeed_kt, FP32 airTemperature_K)
{
	FP32 machnumber, speedofsound_kt;
	speedofsound_kt = (20.046796 * sqrt(airTemperature_K)) * kMsnToKnot;
	machnumber = trueairspeed_kt / speedofsound_kt;
	return machnumber;
}

// Calc mach number
FP32 WorkItemExample::firecontrol_getcd(FP32 machnumber)
{
	FP32 cd;

	if (machnumber < 6.000000E-01)
	{
		cd = 8.670000E-02;
	}
	else if ((machnumber >= 6.000000E-01) & (machnumber < 8.000000E-01))
	{
		cd = (7.950000E-02) + (1.200000E-02 * machnumber) + (-4.183676E-12 * std::pow(machnumber, 2)) + (1.818989E-12 * std::pow(machnumber, 3));
	}
	else if ((machnumber >= 8.000000E-01) & (machnumber < 10.000000E-01))
	{
		cd = (-2.378518E+01) + (8.519819E+01 * machnumber) + (-1.015009E+02 * std::pow(machnumber, 2)) + (4.038448E+01 * std::pow(machnumber, 3));
	}
	else if ((machnumber >= 10.000000E-01) & (machnumber < 10.500000E-01))
	{
		cd = (-9.160000E-01) + (1.130000E+00 * machnumber) + (8.000000E-02 * std::pow(machnumber, 2));
	}
	else if ((machnumber >= 10.500000E-01) & (machnumber < 12.000000E-01))
	{
		cd = (-3.200000E-03) + (7.366667E-01 * machnumber) + (-3.733333E-01 * std::pow(machnumber, 2));
	}
	else
	{
		cd = 0.3432000;
	}

	return cd;
}

// Calc cartesian position
void WorkItemExample::firecontrol_llh2cart(FP32 *konumenu, FP32 targetLat_deg, FP32 targetLon_deg, FP32 targetheight_m,
										   FP32 aircraftLat_deg, FP32 aircraftLon_deg, FP32 aircraftHeight_m)
{
	FP32 radiusEast_m;
	FP32 radiusNorth_m;
	FP32 hedefLat_rad;
	FP32 hedefLon_rad;
	FP32 ucakLat_rad;
	FP32 ucakLon_rad;

	hedefLat_rad = targetLat_deg * kDegreesToRadians;
	hedefLon_rad = targetLon_deg * kDegreesToRadians;
	ucakLat_rad = aircraftLat_deg * kDegreesToRadians;
	ucakLon_rad = aircraftLon_deg * kDegreesToRadians;
	radiusEast_m = 6378137.0 / std::pow((double)(1.0 - (std::pow(0.0818191908426, 2) * std::pow((std::sin(hedefLat_rad)),
																								2))),
										0.5);
	radiusNorth_m = radiusEast_m * (1 - std::pow(0.0818191908426, 2)) / (1 - (std::pow(0.0818191908426, 2) * std::pow((std::sin(hedefLat_rad)), 2)));
	// hedefe g�re do�u de�eri (metre)
	konumenu[0] = (ucakLon_rad - hedefLon_rad) * (cos(hedefLat_rad)) * (radiusEast_m + aircraftHeight_m);
	// hedefe g�re kuzey de�eri (metre)
	konumenu[1] = (ucakLat_rad - hedefLat_rad) * (radiusNorth_m + aircraftHeight_m);
	// u�a��n y�ksekli�i ayn� d�n�yor!!! hedefe g�re de�il
	konumenu[2] = aircraftHeight_m - targetheight_m;
}

// calc ranging
void WorkItemExample::firecontrol_eastnorth2downcross(FP32 *downCross, FP32 posEast_m, FP32 posNorth_m,
													  FP32 aircraftGroundTrack_der)
{
	FP32 hedefeAci_deg;
	FP32 hedefeMesafe_m;

	hedefeMesafe_m = std::pow(posEast_m, 2) + std::pow(posNorth_m, 2);
	hedefeMesafe_m = std::pow((double)hedefeMesafe_m, 0.5);
	hedefeAci_deg = 90 - (atan2(posNorth_m, posEast_m) * kRadiansToDegrees);
	hedefeAci_deg = hedefeAci_deg - aircraftGroundTrack_der;
	downCross[0] = hedefeMesafe_m * cos(hedefeAci_deg * kDegreesToRadians);
	downCross[1] = hedefeMesafe_m * sin(hedefeAci_deg * kDegreesToRadians);
}

// wander axis velocity to north-east coordinates velocity conversion
// [velocityX_ms velocityY_ms] ---> [velocityNorth_ms velocityEast_ms]
void WorkItemExample::firecontrol_wander2northeast(FP32 *velne_ms, FP32 wander_deg, FP32 velX_ms, FP32 velY_ms)
{
	FP32 wander_rad = wander_deg * kDegreesToRadians;
	velne_ms[0] = velX_ms * sin(wander_rad) + velY_ms * cos(wander_rad);
	velne_ms[1] = velX_ms * cos(wander_rad) - velY_ms * sin(wander_rad);
}

// station ned position wrt measurement reference
// momentarm + attitude -> ned position
void WorkItemExample::firecontrol_momentarm2ned(FP32 *deltaMomentArm_m, FP32 momentArmX_m, FP32 momentArmY_m,
												FP32 momentArmZ_m, FP32 roll_deg, FP32 pitch_deg, FP32 trueheading_deg)
{

	FP32 Xp, Yp, Zp;
	FP32 sroll = sin(roll_deg * kDegreesToRadians);
	FP32 croll = cos(roll_deg * kDegreesToRadians);
	FP32 spitch = sin(pitch_deg * kDegreesToRadians);
	FP32 cpitch = cos(pitch_deg * kDegreesToRadians);
	FP32 shead = sin(trueheading_deg * kDegreesToRadians);
	FP32 chead = cos(trueheading_deg * kDegreesToRadians);

	//pitch izdusumu
	Xp = momentArmX_m * cpitch + momentArmZ_m * spitch;
	Zp = -momentArmX_m * spitch + momentArmZ_m * cpitch;

	//roll izdusumu
	Yp = momentArmY_m * croll - Zp * sroll;
	Zp = momentArmY_m * sroll + Zp * croll;

	//heading izdusumu
	deltaMomentArm_m[0] = (-Yp * shead) + (Xp * chead);
	deltaMomentArm_m[1] = (Yp * chead) + (Xp * shead);
	deltaMomentArm_m[2] = Zp;
}

int WorkItemExample::task_spawn(int argc, char *argv[])
{
	WorkItemExample *instance = new WorkItemExample();

	if (instance)
	{
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init())
		{
			return PX4_OK;
		}
	}
	else
	{
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int WorkItemExample::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int WorkItemExample::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int WorkItemExample::print_usage(const char *reason)
{
	if (reason)
	{
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int work_item_example_main(int argc, char *argv[])
{
	return WorkItemExample::main(argc, argv);
}
