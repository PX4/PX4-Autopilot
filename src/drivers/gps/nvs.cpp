/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file nvs.cpp
 * 
 * NVS GNSS receiver BINR protocol implementation.
 * @(http://www.nvs-gnss.com/support/documentation/item/download/39.html)
 * 
 * LSA -Autonomous Systems Laboratory | ISEP
 *
 *  @author Pedro M. Sousa <1111519@isep.ipp.pt>
 *  @author Tiago Miranda <tasantos@inesctec.pt>
 *  @author Miguel Moreira <mmoreira@inesctec.pt>
 *  @author Joel Oliveira <hjfo@inesctec.pt>
 *
 *
 */


#include <assert.h>
#include <math.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <termios.h>


#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <drivers/drv_hrt.h>

#include "nvs.h"


#define NVSSYNC     0x10        /* nvs message sync code 1 */
#define NVSENDMSG   0x03        /* nvs message sync code 1 */
#define NVSCFG      0x06        /* nvs message cfg-??? */

#define ID_XF5RAW   0xf5        /* nvs msg id: raw measurement data */
#define ID_X4AIONO  0x4a        /* nvs msg id: gps ionospheric data */
#define ID_X4BTIME  0x4b        /* nvs msg id: GPS/GLONASS/UTC timescale data */
#define ID_XF7EPH   0xf7        /* nvs msg id: subframe buffer */
#define ID_XC2CFG	0xc2
#define ID_X88PVT	0x88
#define ID_X61DOP   0x61
#define ID_X41COG   0x41
#define ID_X60NRS	0x60

#define NVS_CONFIG_TIMEOUT		200		// ms, timeout for waiting ACK
#define NVS_PACKET_TIMEOUT		2		// ms, if now data during this delay assume that full update received
#define NVS_WAIT_BEFORE_READ	20		// ms, wait before reading to save read() calls
#define DISABLE_MSG_INTERVAL	1000000	// us, try to disable message with this interval



#define PI 								3.14159265
#define radiansToDegrees_1E7(value)		(value*10000000*180)/PI //used in latitude and longitude
#define alt_m_1E3(value)				(value*1000)			//used for altitude



/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((char *)(p)))
//static unsigned short U2(unsigned char *p) {unsigned short u; memcpy(&u,p,2); return u;}
//static unsigned int   U4(unsigned char *p) {unsigned int   u; memcpy(&u,p,4); return u;}
static inline short          I2(unsigned char *p) {short          i; memcpy(&i, p, 2); return i;}
//static int            I4(unsigned char *p) {int            i; memcpy(&i,p,4); return i;}
static inline float          R4(unsigned char *p) {float          r; memcpy(&r, p, 4); return r;}
static inline double         R8(unsigned char *p) {double         r; memcpy(&r, p, 8); return r;}
//static char           R1(unsigned char *p) {char        r; memcpy(&r,p,1); return r;}


//Acrescentado: Joel and Miguel
static char           R1(unsigned char *p) {char        r; memcpy(&r, p, 1); return r;}
gtime_t gpst2time(int week, double sec);
gtime_t epoch2time(const double *ep);


//Convert time (week and sec) in time UTC with refereced date in 1999,8,22,0,0,0
gtime_t gpst2time(int week, double sec)

{
	const static double gpst0[] = {1999, 8, 22, 0, 0, 0}; /* gps time reference */
	gtime_t t = epoch2time(gpst0);

	if (sec < -1E9 || 1E9 < sec) { sec = 0.0; }

	t.time += 86400 * 7 * week + (int)sec;
	t.sec = sec - (int)sec;
	return t;
}


//Receive referenced date and return time
gtime_t epoch2time(const double *ep)
{
	const int doy[] = {1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
	gtime_t time = {0};
	int days, sec, year = (int)ep[0], mon = (int)ep[1], day = (int)ep[2];

	if (year < 1970 || 2099 < year || mon < 1 || 12 < mon) { return time; }

	/* leap year if year%4==0 in 1901-2099 */
	days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] + day - 2 + (year % 4 == 0 && mon >= 3 ? 1 : 0);
	sec = (int)floor(ep[5]);
	time.time = (time_t)days * 86400 + (int)ep[3] * 3600 + (int)ep[4] * 60 + sec;
	time.sec = ep[5] - sec;
	return time;
}


NVS::NVS(const int &fd, struct vehicle_gps_position_s *gps_position) :
	_fd(fd),
	_gps_position(gps_position),
	_configured(false)
{
}

NVS::~NVS()
{
}


int NVS::receive(unsigned timeout)
{
	//warnx("NVS::receive");
	/* poll descriptor */
	pollfd fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;


	/* timeout additional to poll */
	uint64_t time_started = hrt_absolute_time();

	ssize_t count = 0;

	bool handled = false;

	while (true) {

		/* poll for new data, wait for only UBX_PACKET_TIMEOUT (2ms) if something already received */
		int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), handled ? NVS_PACKET_TIMEOUT : timeout);

		//warnx("%d -> poll", ret);

		if (ret < 0) {
			/* something went wrong when polling */
			warnx("nvs: poll error");
			return -1;

		} else if (ret == 0) {
			/* return success after short delay after receiving a packet or timeout after long delay */
			if (handled) {
				return 1;

			} else {
				return -1;
			}

		} else if (ret > 0) {

			/* if we have new data from GPS, go handle it */
			if (fds[0].revents & POLLIN) {
				/*
				 * We are here because poll says there is some data, so this
				 * won't block even on a blocking device. But don't read immediately
				 * by 1-2 bytes, wait for some more data to save expensive read() calls.
				 * If more bytes are available, we'll go back to poll() again.
				 */
				usleep(NVS_WAIT_BEFORE_READ * 1000);
				count = read(_fd, raw.buff, sizeof(raw.buff));

				//warnx(" to read: %d", count);
				/* pass received bytes to the packet decoder */
				for (int i = 0; i < count; i++) {

					//warnx("%c", (unsigned char )raw.buff[i]);

					if (parse_char(raw.buff[i]) > 0) {

						if (decode_nvs() > 0) {
							handled = true;
						}
					}
				}
			}
		}

		/* abort after timeout if no useful packets received */
		if (time_started + timeout * 1000 < hrt_absolute_time()) {
			warnx("nvs: timeout - no useful messages");
			return -1;
		}
	}

	return 1;
}




int NVS::configure(unsigned &baudrate)
{
	_configured = false;
	/* try different baudrates */
	const unsigned baudrates_to_try[] = {9600, 19200, 38400, 57600, 115200};
	//const unsigned baudrates_to_try[] = {115200};
	int baud_i;

	for (baud_i = 0; baud_i < 5; baud_i++) {

		baudrate = baudrates_to_try[baud_i];

		set_baudrate(_fd, baudrate);

		/*	NVS needs odd parity so... (current helper functions don't support it)*/

		struct termios uart_config;

		int termios_state;

		/* fill the struct for the new configuration */
		tcgetattr(_fd, &uart_config);

		/* clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;
		/*one stop bit */
		uart_config.c_cflag &= ~(CSTOPB);
		/*8 data bits, parity enable ODD*/
		uart_config.c_cflag |= (CS8 | PARENB | PARODD);


		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			warnx("ERROR setting NVS port settings (tcsetattr)\n");
			break;
		}

		/*Setting of Additional Operating Parameters */

		char msg[] = {0x10, 0xd7, 0x02, 0x0A, 0x10, 0x03};
		write(_fd, (const void *)msg, 6);
		usleep(50);

		/* Request  for/Setting  of  BINRProtocol   Operation   Mode */
        unsigned char msg1[] = {0x10, 0xb2, 0x04, 0x00, 0x10, 0x03};
		write(_fd, (const void *)msg1, 6);
		usleep(50);

		/* Request  for PVT Vector */
		unsigned char msg2[] = {0x10, 0x27, 0x01, 0x10, 0x03};
		write(_fd, (const void *)msg2, 5);
		usleep(50);

		/* Request  for  the DOP  and RMS for calculated PVT */
		unsigned char msg3[] = {0x10, 0x31, 0x01, 0x10, 0x03};
		write(_fd, (const void *)msg3, 5);
		usleep(50);

		/* Request for the Number of Satellites Used and DOP */
		unsigned char msg4[] = {0x10, 0x21, 0x01, 0x10, 0x03};
		write(_fd, (const void *)msg4, 5);
		usleep(50);

		/**	Request for actual speed and course over ground		*/
		unsigned char msg5[] = {0x10, 0x13, 0x01, 0x10, 0x03};
		write(_fd, (const void *)msg5, 5);
		usleep(50);

		receive(200);

		if (_configured) {		// _configured flag are changed in xc2cfg_decode in response to B2h message
			return 0;
		}

	}


	_configured = false;
	return -1;


}

int NVS::parse_char(uint8_t data)
{

	/* synchronize frame */
	if ((raw.nbyte == 0) && (data == NVSSYNC)) {

		/* Search a 0x10 */
		raw.buff[0] = data;
		raw.nbyte = 1;
		return 0;
	}

	if ((raw.nbyte == 1) && (data != NVSSYNC) && (data != NVSENDMSG)) {

		/* Discard double 0x10 and 0x10 0x03 at beginning of frame */
		raw.buff[1] = data;
		raw.nbyte = 2;
		raw.flag = 0;
		return 0;
	}

	/* This is all done to discard a double 0x10 */
	if (data == NVSSYNC) { raw.flag = (raw.flag + 1) % 2; }

	if ((data != NVSSYNC) || (raw.flag)) {

		/* Store the new byte */
		raw.buff[(raw.nbyte++)] = data;
	}

	/* Detect ending sequence */
	if ((data == NVSENDMSG) && (raw.flag)) {
		raw.len   = raw.nbyte;
		raw.nbyte = 0;

		/* Decode NVS raw message */
		return 1;
	}

	if (raw.nbyte == MAXRAWLEN) {
		raw.nbyte = 0;
		return -1;
	}

	return 0;

}

int NVS::x61dop_decode()
{

	unsigned char *p = raw.buff + 2;

	_gps_position->eph = R4(p);
	_gps_position->epv = R4(p + 4);

	return 1;
}



int NVS::x41cograd_decode()
{
	unsigned char *p = raw.buff + 2;

	_gps_position->cog_rad = (float) R4(p);
	_gps_position->vel_m_s = R4(p + 4);

	return 1;
}


int NVS::x88pvt_decode()
{

	int weeknumber = 0;
	char byte;
	char mask = 0x03;
	unsigned char *p = raw.buff + 2;

	_gps_position->lat = radiansToDegrees_1E7(R8(p));
	
	_gps_position->lon = radiansToDegrees_1E7(R8(p + 8));
	_gps_position->alt = (int32_t) alt_m_1E3(R8(p + 16));

	_gps_position->timestamp_position = hrt_absolute_time();

	_rate_count_lat_lon++;


	/* Velocity */

	_gps_position->vel_n_m_s = R8(p + 40);
	_gps_position->vel_e_m_s = R8(p + 48);
	_gps_position->vel_d_m_s = R8(p + 56);
	_gps_position->timestamp_velocity = hrt_absolute_time();
	_gps_position->vel_ned_valid	= true;


	_rate_count_vel++;



	//	Acrescentado: Joel Oliveira and Miguel Moreira

	weeknumber = I2(p + 38);
	unsigned char dTowUTC[10];
	memcpy(&dTowUTC, p + 28, 10);

	//Decoding a message of 80 bits - View page 87 of BINR NVS manual
	//unsigned int s=(dTowUTC[9] & 128) >>7 ; //not use
	unsigned int e = ((dTowUTC[9] & 127) << 8) + dTowUTC[8];
	//unsigned int i= (dTowUTC[7]&128)>>7; // not use
	uint64_t m = ((uint64_t)(dTowUTC[7] & 127) << 56) + ((uint64_t) dTowUTC[6] << 48) + ((uint64_t) dTowUTC[5] << 40) + ((
				uint64_t) dTowUTC[4] << 32) + (dTowUTC[3] << 24) + (dTowUTC[2] << 16) + (dTowUTC[1] << 8) + dTowUTC[0];
	double time_ms = (pow(2, e - 16383)) * (1 + (m / (pow(2, 63))));

	//Convert time milisec to sec
	uint64_t time_s = time_ms / 1000;
	uint64_t time_ms2 = (uint64_t)time_ms - (time_s * 1000);


	gtime_t time;

	//Convert time
	time = gpst2time(weeknumber, (double)time_s);
	time.sec = (double)time_ms2;

	//Fills the struct with time
	_gps_position->time_utc_usec = ((uint64_t)(time.time) * 1000000) + (uint64_t)(time.sec) * 1000  ; //TODO: test this

	//_gps_position->timestamp_position = _gps_position->time_utc_usec; // <<- wrong according 
	
	_gps_position->timestamp_time 		= hrt_absolute_time();
	_gps_position->timestamp_velocity 	= hrt_absolute_time();
	_gps_position->timestamp_variance 	= hrt_absolute_time();
	_gps_position->timestamp_position	= hrt_absolute_time();




	byte = R1(p + 68);

	if ((byte & mask) == 3 || (byte & mask) == 1) {
		_gps_position->fix_type = 3;

	} else if ((byte & mask) == 2) {
		_gps_position->fix_type = 2;

	} else {
		_gps_position->fix_type = 0;
	}


	return 1;

}


int NVS::xc2cfg_decode()
{
	warnx("NVS GNSS Configured");

	_configured = true;

	return 1;
}


int
NVS::x60nrs_decode()
{
	unsigned char *p = raw.buff + 2;

	_gps_position->satellites_used = U1(p) + U1(p + 1);

	return 1;

}

int NVS::decode_nvs(void)
{
	int type = U1(raw.buff + 1);

	switch (type) {

	case ID_XC2CFG: return xc2cfg_decode();

	case ID_X88PVT: return x88pvt_decode();

	case ID_X61DOP: return x61dop_decode();

	case ID_X41COG: return x41cograd_decode();

	case ID_X60NRS: return x60nrs_decode();

	default: break;
	}

	return 0;
}


