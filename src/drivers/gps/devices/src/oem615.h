/*
* file		: ome615.h
* author	: bdai<bdai1412@gmail.com>
* time		: Jul 18, 2016
* ref		: OEM6 Family Firmware Reference Manual.pdf
*/

#include "gps_helper.h"
#include "../../definitions.h"

#define OEM615_RECV_BUFFER_SIZE 512
#define CRC32_POLYNOMIAL   0xEDB88320L /* for CRC32 */

class GPSDriverOEM615 : public GPSHelper{
public:
	GPSDriverOEM615(GPSCallbackPtr callback, void *callback_user, struct vehicle_gps_position_s *gps_position,
			 struct satellite_info_s *satellite_info);
	virtual ~GPSDriverOEM615();

	int receive(unsigned timeout);
	int configure(unsigned &baudrate, OutputMode output_mode);

private:
	void decodeInit(void);
	int handleMessage(int len);
	int parseChar(uint8_t b);
	uint32_t		CalculateCRC32(uint32_t ulcount, uint8_t *ucBuffer);
	uint32_t        CRC32Value(int i);

	/** Read int OEM615 parameter */
	int32_t read_int();
	/** Read float OEM615 parameter */
	double read_float();
	/** Read char OEM615 parameter */
	char read_char();

	enum oem615_decode_state_t {
		OEM615_DECODE_UNINIT,
		OEM615_DECODE_GOT_SYNC1,
		NME_DECODE_GOT_SYNC1,
		OEM615_DECODE_GOT_ASTERIKS,
		NME_DECODE_GOT_ASTERIKS,
		OEM615_DECODE_GOT_FIRST_CS_BYTE,
		NME_DECODE_GOT_FIRST_CS_BYTE
	};

	struct satellite_info_s 		*_satellite_info;
	struct vehicle_gps_position_s 	*_gps_position;

	uint64_t		 				_last_timestamp_time;
	int 							oem615log_fd;

	oem615_decode_state_t 			_decode_state;
	uint8_t							_crc_pos;
	uint8_t 						_rx_buffer[OEM615_RECV_BUFFER_SIZE];
	uint16_t 						_rx_buffer_bytes;
	bool 							_parse_error; /** parse error flag */
	char 							*_parse_pos; /** parse position */


};
