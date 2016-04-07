#include "ub280.h"

#include <px4_posix.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <poll.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/satellite_info.h>
#include <drivers/drv_hrt.h>
#include <fcntl.h>
#include <math.h>

#define UB280_DEBUG(s, ...)		{/*PX4_WARN(s, ## __VA_ARGS__);*/}

UB280::UB280(const int &fd, struct vehicle_gps_position_s *gps_position, struct satellite_info_s *satellite_info):
	_fd(fd),
	_satellite_info(satellite_info),
	_gps_position(gps_position)
{
	decode_init();
	_decode_state = NME_DECODE_UNINIT;
	_rx_buffer_bytes = 0;
}

UB280::~UB280()
{
}

/*
 * All NMEA descriptions are taken from
 * http://www.trimble.com/OEM_ReceiverHelp/V4.44/en/NMEA-0183messages_MessageOverview.html
 */

int UB280::handle_message(int len)
{
	char *endp;
	uint8_t st = 0;
	if(_rx_buffer[0] == '#'){
		while(_rx_buffer[st++]!=';'){
			if(st > len) return 0;
		}
		st--;
	}

	if (len < 7) { return 0; }

	int uiCalcComma = 0;

	for (int i = st ; i < len; i++) {
		if (_rx_buffer[i] == ',') { uiCalcComma++; }
	}
		/* decode the NMEA protocol */
	if(_rx_buffer[0] == '$'){
		char *bufptr = (char *)(_rx_buffer + 6);
		if ((memcmp(_rx_buffer + 3, "GGA,", 3) == 0) && (uiCalcComma == 14)) {
			/*
			  Time, position, and fix related data
			  An example of the GBS message string is:

			  $GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F

			  Note - The data string exceeds the UB280 standard length.
			  GGA message fields
			  Field   Meaning
			  0   Message ID $GPGGA
			  1   UTC of position fix
			  2   Latitude
			  3   Direction of latitude:
			  N: North
			  S: South
			  4   Longitude
			  5   Direction of longitude:
			  E: East
			  W: West
			  6   GPS Quality indicator:
			  0: Fix not valid
			  1: GPS fix
			  2: Differential GPS fix, OmniSTAR VBS
			  4: Real-Time Kinematic, fixed integers
			  5: Real-Time Kinematic, float integers, OmniSTAR XP/HP or Location RTK
			  7   Number of SVs in use, range from 00 through to 24+
			  8   HDOP
			  9   Orthometric height (MSL reference)
			  10  M: unit of measure for orthometric height is meters
			  11  Geoid separation
			  12  M: geoid separation measured in meters
			  13  Age of differential GPS data record, Type 1 or Type 9. Null field when DGPS is not used.
			  14  Reference station ID, range 0000-4095. A null field when any reference station ID is selected and no corrections are received1.
			  15
			  The checksum data, always begins with *
			  Note - If a user-defined geoid model, or an inclined
			*/
			double UB280_time __attribute__((unused)) = 0.0, lat = 0.0, lon = 0.0, alt = 0.0;
			int num_of_sv __attribute__((unused)) = 0, fix_quality = 0;
			double hdop __attribute__((unused)) = 99.9;
			char ns = '?', ew = '?';

			if (bufptr && *(++bufptr) != ',') { UB280_time = strtod(bufptr, &endp); bufptr = endp; }

			if (bufptr && *(++bufptr) != ',') { lat = strtod(bufptr, &endp); bufptr = endp; }

			if (bufptr && *(++bufptr) != ',') { ns = *(bufptr++); }

			if (bufptr && *(++bufptr) != ',') { lon = strtod(bufptr, &endp); bufptr = endp; }

			if (bufptr && *(++bufptr) != ',') { ew = *(bufptr++); }

			if (bufptr && *(++bufptr) != ',') { fix_quality = strtol(bufptr, &endp, 10); bufptr = endp; }

			if (bufptr && *(++bufptr) != ',') { num_of_sv = strtol(bufptr, &endp, 10); bufptr = endp; }

			if (bufptr && *(++bufptr) != ',') { hdop = strtod(bufptr, &endp); bufptr = endp; }

			if (bufptr && *(++bufptr) != ',') { alt = strtod(bufptr, &endp); bufptr = endp; }

			if (ns == 'S') {
				lat = -lat;
			}

			if (ew == 'W') {
				lon = -lon;
			}
			UB280_DEBUG("decode GGA");
			UB280_DEBUG("latitude =%.8f",lat);
			UB280_DEBUG("lontitude =%.8f", lon);
			UB280_DEBUG("altitude =%.8f", alt);
			//UB280_DEBUG("hdop =%.8f", hdop);
			/* convert from degrees, minutes and seconds to degrees * 1e7 */
			_gps_position->lat = static_cast<int>((int(lat * 0.01) + (lat * 0.01 - int(lat * 0.01)) * 100.0 / 60.0) * 10000000);
			_gps_position->lon = static_cast<int>((int(lon * 0.01) + (lon * 0.01 - int(lon * 0.01)) * 100.0 / 60.0) * 10000000);
			_gps_position->alt = static_cast<int>(alt * 1000);
			_gps_position->hdop = hdop;
			_rate_count_lat_lon++;



			if (fix_quality <= 0) {
				_gps_position->fix_type = 0;

			} else {
				/*
				 * in this NMEA message float integers (value 5) mode has higher value than fixed integers (value 4), whereas it provides lower quality,
				 * and since value 3 is not being used, I "moved" value 5 to 3 to add it to _gps_position->fix_type
				 */
				if (fix_quality == 5) { fix_quality = 3; }

				/*
				 * fix quality 1 means just a normal 3D fix, so I'm subtracting 1 here. This way we'll have 3 for auto, 4 for DGPS, 5 for floats, 6 for fixed.
				 */
				_gps_position->fix_type = 3 + fix_quality - 1;
			}

			_gps_position->timestamp_position = hrt_absolute_time();

//			_gps_position->vel_m_s = 0;                                  /**< GPS ground speed (m/s) */
//			_gps_position->vel_n_m_s = 0;                                /**< GPS ground speed in m/s */
//			_gps_position->vel_e_m_s = 0;                                /**< GPS ground speed in m/s */
//			_gps_position->vel_d_m_s = 0;                                /**< GPS ground speed in m/s */
//			_gps_position->cog_rad = 0;                                  /**< Course over ground (NOT heading, but direction of movement) in rad, -PI..PI */
//			_gps_position->vel_ned_valid = true;                         /**< Flag to indicate if NED speed is valid */
			_gps_position->c_variance_rad = 0.1f;
			_gps_position->timestamp_velocity = hrt_absolute_time();
			return 1;

		}else if ((memcmp(_rx_buffer + 3, "GSA,", 3) == 0) && (uiCalcComma == 17)) {
					/*
					  GPS DOP 值和有效卫星信息

					  $GNGSA,M,3,008,001,027,,,,,,,,,,9.3,5.8,7.3*1F

					  The Talker ID ($--) will vary depending on the satellite system used for the position solution:

					  $GP - GPS only
					  $GL - GLONASS only
					  $GN - Combined
					  0   Message ID $GPGSA
					  1   MODE MA  A = 自动  M = 手动
					  2   MODE 123 1 = 固定或不可用 2 = 2D 3 = 3D
					  3-14 PRN 在解中使用的卫星PRN编号 共12字段
					  15  pdop 位置精度因子
					  16  hdop 水平精度因子
					  17  vdop 高度精度因子
					  18  校验和
					*/
					int RPN_num[12] __attribute__((unused)) = {0};
					int mode_123 __attribute__((unused)) = 1;
					double hdop __attribute__((unused)) = 99.9, pdop __attribute__((unused)) = 99.9, vdop __attribute__((unused)) = 99.9;
					char mode_MA = '?';

					if (bufptr && *(++bufptr) != ',') { mode_MA = *(bufptr++); }

					if (bufptr && *(++bufptr) != ',') { mode_123 = strtol(bufptr, &endp , 10); bufptr = endp; }

					for(int i = 0; i < 12 ;i++)
					{
						if (bufptr && *(++bufptr) != ',') { RPN_num[i] = strtol(bufptr, &endp , 10); bufptr = endp; }
					}

					if (bufptr && *(++bufptr) != ',') { pdop = strtod(bufptr, &endp); bufptr = endp; }

					if (bufptr && *(++bufptr) != ',') { hdop = strtod(bufptr, &endp); bufptr = endp; }

					if (bufptr && *(++bufptr) != ',') { vdop = strtod(bufptr, &endp); bufptr = endp; }

					if(mode_MA){}
					_gps_position->vdop = vdop;
					_gps_position->hdop = hdop;
					//_gps_position->pdop = pdop;  //	计算不需要用到
					_gps_position->timestamp_variance = hrt_absolute_time();

					UB280_DEBUG("decode GSA");
//					UB280_DEBUG("mode123 = %d", mode_123);
//					UB280_DEBUG("hdop = %.2f", (double)hdop);
//					UB280_DEBUG("vdop = %.2f",(double)vdop);
					return 1;
		}else if ((memcmp(_rx_buffer + 3, "GST,", 3) == 0) && (uiCalcComma == 8)) {
					/*
					  Position error statistics
					  An example of the GST message string is:

					  $GPGST,172814.0,0.006,0.023,0.020,273.6,0.023,0.020,0.031*6A

					  The Talker ID ($--) will vary depending on the satellite system used for the position solution:

					  $GP - GPS only
					  $GL - GLONASS only
					  $GN - Combined
					  GST message fields
					  Field   Meaning
					  0   Message ID $GPGST
					  1   UTC of position fix
					  2   RMS value of the pseudorange residuals; includes carrier phase residuals during periods of RTK (float) and RTK (fixed) processing
					  3   Error ellipse semi-major axis 1 sigma error, in meters
					  4   Error ellipse semi-minor axis 1 sigma error, in meters
					  5   Error ellipse orientation, degrees from true north
					  6   Latitude 1 sigma error, in meters
					  7   Longitude 1 sigma error, in meters
					  8   Height 1 sigma error, in meters
					  9   The checksum data, always begins with *
					*/
					double UB280_time __attribute__((unused)) = 0.0, lat_err = 0.0, lon_err = 0.0, alt_err = 0.0;
					double min_err __attribute__((unused)) = 0.0, maj_err __attribute__((unused)) = 0.0,
					deg_from_north __attribute__((unused)) = 0.0, rms_err __attribute__((unused)) = 0.0;

					if (bufptr && *(++bufptr) != ',') { UB280_time = strtod(bufptr, &endp); bufptr = endp; }

					if (bufptr && *(++bufptr) != ',') { rms_err = strtod(bufptr, &endp); bufptr = endp; }

					if (bufptr && *(++bufptr) != ',') { maj_err = strtod(bufptr, &endp); bufptr = endp; }

					if (bufptr && *(++bufptr) != ',') { min_err = strtod(bufptr, &endp); bufptr = endp; }

					if (bufptr && *(++bufptr) != ',') { deg_from_north = strtod(bufptr, &endp); bufptr = endp; }

					if (bufptr && *(++bufptr) != ',') { lat_err = strtod(bufptr, &endp); bufptr = endp; }

					if (bufptr && *(++bufptr) != ',') { lon_err = strtod(bufptr, &endp); bufptr = endp; }

					if (bufptr && *(++bufptr) != ',') { alt_err = strtod(bufptr, &endp); bufptr = endp; }

					_gps_position->eph = sqrtf(static_cast<float>(lat_err) * static_cast<float>(lat_err)
								   + static_cast<float>(lon_err) * static_cast<float>(lon_err));
					_gps_position->epv = static_cast<float>(alt_err);

					_gps_position->s_variance_m_s = 0;
					_gps_position->timestamp_variance = hrt_absolute_time();

					UB280_DEBUG("decode GST");
					UB280_DEBUG("lat_err = %.2f", (double)lat_err);
					UB280_DEBUG("lon_err = %.2f",(double)lon_err);
					UB280_DEBUG("alt_err = %.2f", (double)alt_err);

					UB280_DEBUG("eph = %.2f", (double)_gps_position->eph);
					UB280_DEBUG("epv = %.2f",(double)_gps_position->epv);

					return 1;
		}else if ((memcmp(_rx_buffer + 3, "GSV,", 3) == 0)){
			/*
			  The GSV message string identifies the number of SVs in view, the PRN numbers, elevations, azimuths, and SNR values. An example of the GSV message string is:

			  $GPGSV,4,1,13,02,02,213,,03,-3,000,,11,00,121,,14,13,172,05*67

			  GSV message fields
			  Field   Meaning
			  0   Message ID $GPGSV
			  1   Total number of messages of this type in this cycle
			  2   Message number
			  3   Total number of SVs visible
			  4   SV PRN number
			  5   Elevation, in degrees, 90 maximum
			  6   Azimuth, degrees from True North, 000 through 359
			  7   SNR, 00 through 99 dB (null when not tracking)
			  8-11    Information about second SV, same format as fields 4 through 7
			  12-15   Information about third SV, same format as fields 4 through 7
			  16-19   Information about fourth SV, same format as fields 4 through 7
			  20  The checksum data, always begins with *
			*/
			/*
			 * currently process only gps, because do not know what
			 * Global satellite ID I should use for non GPS sats
			 */
			bool bGPS = false;

			if (memcmp(_rx_buffer, "$GP", 3) != 0) {
				return 0;

			} else {
				bGPS = true;
			}

			int all_msg_num = 0, this_msg_num = 0, tot_sv_visible = 0;
			struct gsv_sat {
				int svid;
				int elevation;
				int azimuth;
				int snr;
			} sat[4];
			memset(sat, 0, sizeof(sat));

			if (bufptr && *(++bufptr) != ',') { all_msg_num = strtol(bufptr, &endp, 10); bufptr = endp; }

			if (bufptr && *(++bufptr) != ',') { this_msg_num = strtol(bufptr, &endp, 10); bufptr = endp; }

			if (bufptr && *(++bufptr) != ',') { tot_sv_visible = strtol(bufptr, &endp, 10); bufptr = endp; }

			if ((this_msg_num < 1) || (this_msg_num > all_msg_num)) {
				return 0;
			}

			if ((this_msg_num == 0) && (bGPS == true)) {
				memset(_satellite_info->svid,     0, sizeof(_satellite_info->svid));
				memset(_satellite_info->used,     0, sizeof(_satellite_info->used));
				memset(_satellite_info->snr,      0, sizeof(_satellite_info->snr));
				memset(_satellite_info->elevation, 0, sizeof(_satellite_info->elevation));
				memset(_satellite_info->azimuth,  0, sizeof(_satellite_info->azimuth));
			}

			int end = 4;

			if (this_msg_num == all_msg_num) {
				end =  tot_sv_visible - (this_msg_num - 1) * 4;
				_gps_position->satellites_used = tot_sv_visible;
				_satellite_info->count = satellite_info_s::SAT_INFO_MAX_SATELLITES;
				_satellite_info->timestamp = hrt_absolute_time();
			}

			for (int y = 0 ; y < end ; y++) {
				if (bufptr && *(++bufptr) != ',') { sat[y].svid = strtol(bufptr, &endp, 10); bufptr = endp; }

				if (bufptr && *(++bufptr) != ',') { sat[y].elevation = strtol(bufptr, &endp, 10); bufptr = endp; }

				if (bufptr && *(++bufptr) != ',') { sat[y].azimuth = strtol(bufptr, &endp, 10); bufptr = endp; }

				if (bufptr && *(++bufptr) != ',') { sat[y].snr = strtol(bufptr, &endp, 10); bufptr = endp; }

				_satellite_info->svid[y + (this_msg_num - 1) * 4]      = sat[y].svid;
				_satellite_info->used[y + (this_msg_num - 1) * 4]      = ((sat[y].snr > 0) ? true : false);
				_satellite_info->snr[y + (this_msg_num - 1) * 4]       = sat[y].snr;
				_satellite_info->elevation[y + (this_msg_num - 1) * 4] = sat[y].elevation;
				_satellite_info->azimuth[y + (this_msg_num - 1) * 4]   = sat[y].azimuth;
			}
			return 1;
		}
	}

	/* decode the user protocol use by UB280 */
	if(_rx_buffer[0] == '#'){
		uint8_t now_pos = st ;
		double hor_spd __attribute__((unused)) = 0.0, trk_gnd = 0.0, vertical_spd = 0.0;
		char *bufptr ;
		uint8_t sol_state = 0;
		UB280_DEBUG("start pos %d", st);
		if((memcmp(_rx_buffer + 1, "BESTVEL", 7) == 0) && (uiCalcComma == 7)){
			UB280_DEBUG("decode BESTPOS");
			/* solve status */
			//UB280_DEBUG("; position = %d", now_pos);
			if(memcmp(_rx_buffer+now_pos,";SOL_COMPUTED",13) == 0){
				sol_state = 1;
			}else if(memcmp(_rx_buffer+now_pos,";INSUFFICIENT_OBS",17) == 0){

			}else if(memcmp(_rx_buffer+now_pos,";NO_CONVERGENCE",15) == 0){

			}else if(memcmp(_rx_buffer+now_pos,";COV_TRACE",10) == 0){

			}

			/* 2 速度类型 */
			while(_rx_buffer[now_pos++] != ',') if(now_pos > len ) return 0;

			/* 3 延迟值 */
			while(_rx_buffer[now_pos++] != ',') if(now_pos > len ) return 0;

			/* 4 差分龄期 */
			while(_rx_buffer[now_pos++] != ',') if(now_pos > len ) return 0;

			/* 5 对地水平速度*/
			while(_rx_buffer[now_pos++] != ',') if(now_pos > len ) return 0;
			bufptr = (char *)(_rx_buffer + now_pos);
			hor_spd = strtod(bufptr, &endp);

			/* 6 真北角度 deg*/
			while(_rx_buffer[now_pos++] != ',') if(now_pos > len ) return 0;
			bufptr = (char *)(_rx_buffer + now_pos);
			trk_gnd = strtod(bufptr, &endp);

			/* 7 垂直速度 m/s */
			while(_rx_buffer[now_pos++] != ',') if(now_pos > len ) return 0;
			bufptr = (char *)(_rx_buffer + now_pos);
			vertical_spd = strtod(bufptr, &endp);

			UB280_DEBUG("hor_spd = %.6f" , hor_spd);
			UB280_DEBUG("trk_gnd = %.6f" , trk_gnd);
			UB280_DEBUG("vertical_spd = %.6f" , vertical_spd);

			if(sol_state){
				float track_rad = static_cast<float>(trk_gnd) * M_PI_F / 180.0f;
				_gps_position->vel_n_m_s   =  static_cast<float>(hor_spd) * cosf(track_rad);
				_gps_position->vel_e_m_s   =  static_cast<float>(hor_spd) * sinf(track_rad);
				_gps_position->vel_d_m_s   =  static_cast<float>(vertical_spd);
				_gps_position->vel_ned_valid = true;
				UB280_DEBUG("vel_n_m_s = %.6f" , (double)_gps_position->vel_n_m_s );
				UB280_DEBUG("vel_e_m_s = %.6f" , (double)_gps_position->vel_e_m_s );
				UB280_DEBUG("vel_d_m_s = %.6f" , (double)_gps_position->vel_d_m_s );
			}else {
				_gps_position->vel_n_m_s   =  0;
				_gps_position->vel_e_m_s   =  0;
				_gps_position->vel_d_m_s   =  0;
				_gps_position->vel_ned_valid = false;
				UB280_DEBUG("data can not to work out sulotion !");
			}
		}
	}
	return 0;
}

int UB280::receive(unsigned timeout)
{
	{
		/* poll descriptor */
		pollfd fds[1];
		fds[0].fd = _fd;
		fds[0].events = POLLIN;

		uint8_t buf[512];

		/* timeout additional to poll */
		uint64_t time_started = hrt_absolute_time();

		int j = 0;
		ssize_t bytes_count = 0;

		while (true) {

			/* pass received bytes to the packet decoder */
			while (j < bytes_count) {
				int l = 0;

				if ((l = parse_char(buf[j])) > 0) {
					/* return to configure during configuration or to the gps driver during normal work
					 * if a packet has arrived */
					if (handle_message(l) > 0) {
						return 1;
					}
				}

				/* in case we keep trying but only get crap from GPS */
				if (time_started + timeout * 1000 * 2 < hrt_absolute_time()) {
					return -1;
				}

				j++;
			}

			/* everything is read */
			j = bytes_count = 0;

			/* then poll for new data */
			int ret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), timeout * 2);

			if (ret < 0) {
				/* something went wrong when polling */
				return -1;

			} else if (ret == 0) {
				/* Timeout */
				return -1;

			} else if (ret > 0) {
				/* if we have new data from GPS, go handle it */
				if (fds[0].revents & POLLIN) {
					/*
					 * We are here because poll says there is some data, so this
					 * won't block even on a blocking device.  If more bytes are
					 * available, we'll go back to poll() again...
					 */
					bytes_count = ::read(_fd, buf, sizeof(buf));
				}
			}
		}
	}

}

#define HEXDIGIT_CHAR(d) ((char)((d) + (((d) < 0xA) ? '0' : 'A'-0xA)))

char UB280::Hex2Char(char byTemp)
{
    if(byTemp >= 'A' && byTemp <= 'F')
    {
        byTemp = byTemp - 'A' + 10;
    }
    else if(byTemp >= 'a' && byTemp <= 'f')
    {
       byTemp = byTemp - 'a' + 10;
    }
    else        // show '0' - '9'
    {
        byTemp = byTemp - '0';
    }
    return(byTemp);
}

int UB280::parse_char(uint8_t b)
{
	int iRet = 0;
	static uint8_t type = 1; /* protocol type 1 for NMEA , 2 for user define use by UB280 */
	static uint8_t cnt = 0;
	switch (_decode_state) {
	/* First, look for sync1 */
	case NME_DECODE_UNINIT:
		if (b == '#'|| b == '$') {
			_decode_state = NME_DECODE_GOT_SYNC1;
			_rx_buffer_bytes = 0;
			_rx_buffer[_rx_buffer_bytes++] = b;
			if( b == '#' ) type = 2 ;else type = 1;
		}

		break;

	case NME_DECODE_GOT_SYNC1:
		if (b == '$' || b == '#') {
			_decode_state = NME_DECODE_GOT_SYNC1;
			_rx_buffer_bytes = 0;
			if( b == '#' ) type = 2 ;else type = 1;
		} else if (b == '*') {
			_decode_state = NME_DECODE_GOT_ASTERIKS;
		}

//		if (_rx_buffer_bytes >= (sizeof(_rx_buffer) - 5)) {
//			_decode_state = NME_DECODE_UNINIT;
//			_rx_buffer_bytes = 0;
//
//		} else {
			_rx_buffer[_rx_buffer_bytes++] = b;
//		}

		break;

	case NME_DECODE_GOT_ASTERIKS:
		_rx_buffer[_rx_buffer_bytes++] = b;
		_decode_state = NME_DECODE_GOT_FIRST_CS_BYTE;
		break;

	case NME_DECODE_GOT_FIRST_CS_BYTE:
		_rx_buffer[_rx_buffer_bytes++] = b;
		/* use fot NMEA protocol */
		if( type == 1)
		{
			uint8_t checksum = 0;
			uint8_t *buffer = _rx_buffer + 1;
			uint8_t *bufend = _rx_buffer + _rx_buffer_bytes - 3;

			for (; buffer < bufend; buffer++) { checksum ^= *buffer; }

			if ((HEXDIGIT_CHAR(checksum >> 4) == *(_rx_buffer + _rx_buffer_bytes - 2)) &&
				(HEXDIGIT_CHAR(checksum & 0x0F) == *(_rx_buffer + _rx_buffer_bytes - 1))) {
				iRet = _rx_buffer_bytes;
			}else  UB280_DEBUG("NEMA0183 CRC CHECK FAILER !");

			_decode_state = NME_DECODE_UNINIT;
			_rx_buffer_bytes = 0;
			cnt = 0;
		}else
		{
			cnt ++;
			if( cnt > 5)
			_decode_state = NME_DECODE_GOT_SEVENTH_CS_BYTE ;
		}
		break;
	case NME_DECODE_GOT_SEVENTH_CS_BYTE:
		_rx_buffer[_rx_buffer_bytes++] = b;

		for(int i = 0 ;i < _rx_buffer_bytes ; i++)printf("%c",_rx_buffer[i]);
		//UB280_DEBUG("\n");

		uint16_t start =  1;
		uint16_t end =  _rx_buffer_bytes - 10;

		uint32_t  ulCRC = 0;
		uint32_t checksum = 0;

		for(int i = _rx_buffer_bytes - 8 ; i < _rx_buffer_bytes ; i++ )
		checksum +=  Hex2Char( _rx_buffer[i] ) << ((_rx_buffer_bytes - i -1 )*4);


//		UB280_DEBUG("\n");
//		UB280_DEBUG("get check %x",checksum);
//		UB280_DEBUG("\n");
//
//		UB280_DEBUG("start = %c \n",_rx_buffer[start]);
//		UB280_DEBUG("end = %c \n",_rx_buffer[end]);

		for (int i = start; i <= end; i++) {
			ulCRC = aulCrcTable[(ulCRC ^ _rx_buffer[i]) & 0xff] ^ (ulCRC >> 8);}

//		UB280_DEBUG("cal check %x",ulCRC);
//		UB280_DEBUG("\n");

		if ( ulCRC == checksum ) {
			iRet = _rx_buffer_bytes;
		}else UB280_DEBUG("BESTVEL CRC CHECK FAILER !");

		_decode_state = NME_DECODE_UNINIT;
		_rx_buffer_bytes = 0;
		cnt = 0;
		break;
	}

	return iRet;
}

void UB280::decode_init(void)
{

}

/*
 * UB280 board configuration script
 */

const char comm[] = "$PASHS,POP,20\r\n"\
		    "$PASHS,NME,ZDA,B,ON,3\r\n"\
		    "$PASHS,NME,GGA,B,OFF\r\n"\
		    "$PASHS,NME,GST,B,ON,3\r\n"\
		    "$PASHS,NME,POS,B,ON,0.05\r\n"\
		    "$PASHS,NME,GSV,B,ON,3\r\n"\
		    "$PASHS,SPD,A,8\r\n"\
		    "$PASHS,SPD,B,9\r\n";

int UB280::configure(unsigned &baudrate)
{
	/* try different baudrates */
	//const unsigned baudrates_to_try[] = {9600, 38400, 19200, 57600, 115200};


//	for (unsigned int baud_i = 0; baud_i < sizeof(baudrates_to_try) / sizeof(baudrates_to_try[0]); baud_i++) {
//		baudrate = baudrates_to_try[baud_i];
//		set_baudrate(_fd, baudrate);
//		write(_fd, comm, sizeof(comm));
//	}

	set_baudrate(_fd, 115200);
	return 0;
}
