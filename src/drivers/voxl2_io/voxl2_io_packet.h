/****************************************************************************
 * Copyright (c) 2017 The Linux Foundation. All rights reserved.
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
 * 3. Neither the name The Linux Foundation nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE.
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
 * In addition Supplemental Terms apply.  See the SUPPLEMENTAL file.
 *
 ****************************************************************************/

/*
 * This file contains function prototypes for Voxl2 IO UART interface
 */

#ifndef VOXL2_IO_PACKET
#define VOXL2_IO_PACKET

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "voxl2_io_crc16.h"

// Define byte values that correspond to setting red, greed, and blue LEDs
#define VOXL2_IO_LED_RED_ON   1
#define VOXL2_IO_LED_GREEN_ON 2
#define VOXL2_IO_LED_BLUE_ON  4


// Header of the packet. Each packet must start with this header
#define VOXL2_IO_PACKET_HEADER    0xAF

enum { VOXL2_IO_ERROR_BAD_LENGTH   = -3,
       VOXL2_IO_ERROR_BAD_HEADER   = -2,
       VOXL2_IO_ERROR_BAD_CHECKSUM = -1,
       VOXL2_IO_NO_PACKET          = 0
     };

// Defines for the constatnt offsets of different parts of the packet
enum { VOXL2_IO_PACKET_POS_HEADER1 = 0,
       VOXL2_IO_PACKET_POS_LENGTH,
       VOXL2_IO_PACKET_POS_TYPE,
       VOXL2_IO_PACKET_POS_DATA
     };

// Definition of the structure that holds the state of the incoming data that is being recived (i.e. incomplete packets)
typedef struct {
	uint8_t  len_received; // Number of chars received so far
	uint8_t  len_expected; // Expected number of chars based on header
	uint8_t   *bp;         // Pointer to the next write position in the buffer
	uint16_t crc;          // Accumulated CRC value so far
	uint8_t  buffer[64];   // Buffer to hold incoming data that is being parsed
} VOXL2_IOPacket;


// Definition of the response packet from ESC, containing the ESC version information
typedef struct {
	uint8_t  header;
	uint8_t  length;       // Total length of the packet
	uint8_t  type;         // This will be equal to ESC_PACKET_TYPE_VERSION_RESPONSE

	uint8_t  id;           // ID of the ESC that responded
	uint16_t sw_version;   // Software version of the ESC firmware
	uint16_t hw_version;   // HW version of the board

	uint32_t unique_id;    // Unique ID of the ESC, if available
	uint16_t crc;
}  __attribute__((__packed__)) VOXL2_IO_VERSION_INFO;

typedef struct {
	uint8_t  header;
	uint8_t  length;
	uint8_t  type;
	uint8_t  id;
	uint16_t sw_version;
	uint16_t hw_version;
	uint8_t  unique_id[12];
	char     firmware_git_version[12];
	char     bootloader_git_version[12];
	uint16_t bootloader_version;
	uint16_t crc;
}  __attribute__((__packed__)) VOXL2_IO_EXTENDED_VERSION_INFO;

// Definition of the feedback response packet from ESC
typedef struct {
	uint8_t  header;
	uint8_t  length;       // Total length of the packet
	uint8_t  type;         // This will be equal to ESC_PACKET_TYPE_FB_RESPONSE

	uint8_t  state;        // bits 0:3 = state, bits 4:7 = ID
	uint16_t rpm;          // Current RPM of the motor
	uint8_t  cmd_counter;  // Number of commands received by the ESC
	uint8_t  reserved0;
	int8_t   voltage;      // Voltage = (-28)/34.0 + 9.0 = 8.176V.  0xE4 --> 228 (-28)
	uint8_t  reserved1;

	uint16_t crc;
}  __attribute__((__packed__)) VOXL2_IO_FB_RESPONSE;

// Definition of the feedback response packet from ESC
typedef struct {
	uint8_t  header;
	uint8_t  length;       // Total length of the packet
	uint8_t  type;         // This will be equal to ESC_PACKET_TYPE_FB_RESPONSE
	uint8_t  id_state;     // bits 0:3 = state, bits 4:7 = ID

	uint16_t rpm;          // Current RPM of the motor
	uint8_t  cmd_counter;  // Number of commands received by the ESC
	uint8_t  power;        // Applied power [0..100]

	uint16_t voltage;      // Voltage measured by the ESC in mV
	int16_t  current;      // Current measured by the ESC in 8mA resolution
	int16_t  temperature;  // Temperature measured by the ESC in 0.01 degC resolution

	uint16_t crc;
}  __attribute__((__packed__)) VOXL2_IO_FB_RESPONSE_V2;


// Definition of the feedback response packet from ESC, which contains battery voltage and total current
typedef struct {
	uint8_t  header;
	uint8_t  length;
	uint8_t  type;
	uint8_t  id;       //ESC Id (could be used as system ID elsewhere)
	uint16_t voltage;  //Input voltage (Millivolts)
	int16_t  current;  //Total Current (8mA resolution)
	uint16_t crc;
}   __attribute__((__packed__)) VOXL2_IO_FB_POWER_STATUS;


//-------------------------------------------------------------------------
//Below are functions for generating packets that would be outgoing to ESCs
//-------------------------------------------------------------------------

// Create a generic packet by providing all required components
// Inputs are packet type, input data array and its size, output array and maximum size of output array
// Resulting packet will be placed in the output data array together with appropriate header and checksum
// Output value represents total length of the created packet (if positive), otherwise error code
int32_t voxl2_io_create_packet(uint8_t type, uint8_t *input_data, uint16_t input_size, uint8_t *out_data,
			       uint16_t out_data_size);

// Create a packet for requesting version information from ESC with desired id
// If an ESC with this id is connected and receives this command, it will reply with it's version information
int32_t voxl2_io_create_version_request_packet(uint8_t id, uint8_t *out, uint16_t out_size);
int32_t voxl2_io_create_extended_version_request_packet(uint8_t id, uint8_t *out, uint16_t out_size);

// Create a packet for requesting an ESC with desired id to reset
// When ESC with the particular id receives this command, and it's not spinning, ESC will reset
// This is useful for entering bootloader without removing power from the system
int32_t voxl2_io_create_reset_packet(uint8_t id, uint8_t *out, uint16_t out_size);

// Create a packet for generating a tone packet (signals are applied to motor to generate sounds)
// Inputs are relative frequency (0-255), relative duration (0-255), power (0-255) and bit mask for which ESCs should play a tone
// Bit mask definition: if bit i is set to 1, then ESC with ID=i will generate the tone
// Note that tones can only be generated when motor is not spinning
int32_t voxl2_io_create_sound_packet(uint8_t frequency, uint8_t duration, uint8_t power, uint8_t mask, uint8_t *out,
				     uint16_t out_size);

// Create a packet for standalone LED control
// Bit mask definition:
// led_byte_1 - bit0 = ESC0 Red, bit1 = ESC0, Green, bit2 = ESC0 Blue, bit3 = ESC1 Red, bit4 = ESC1 Green,
// bit5 = ESC1 Blue, bit6 = ESC2 Red, bit7 = ESC2 Green
// led_byte_2 - bit0 = ESC2 Blue, bit1 = ESC3 Red, bit2 = ESC3 Green, bit3 = ESC3 Blue, bits 4:7 = unused
// Note that control can only be sent when motor is not spinning
int32_t voxl2_io_create_led_control_packet(uint8_t led_byte_1, uint8_t led_byte_2, uint8_t *out, uint16_t out_size);

// Create a packet for setting the ID of an ESC
// Return value is the length of generated packet (if positive), otherwise error code
// Note that all ESCs that will receive this command will be set to this ID
int32_t voxl2_io_create_set_id_packet(uint8_t id, uint8_t *out, uint16_t out_size);

// Create a packet for sending open-loop command and LED command to 4 ESCs without requesting any feedback
// Return value is the length of generated packet (if positive), otherwise error code
int32_t voxl2_io_create_pwm_packet4(int16_t pwm0, int16_t pwm1, int16_t pwm2, int16_t pwm3,
				    uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3,
				    uint8_t *out, uint16_t out_size);

// Create a packet for sending open-loop command and LED command to 4 ESCs, also request feedback from one ESC (with id=fb_id)
// Return value is the length of generated packet (if positive), otherwise error code
int32_t voxl2_io_create_pwm_packet4_fb(int16_t pwm0, int16_t pwm1, int16_t pwm2, int16_t pwm3,
				       uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3,
				       int32_t fb_id, uint8_t *out, uint16_t out_size);

// Create a packet for sending closed-loop RPM command and LED command to 4 ESCs without requesting any feedback
// Return value is the length of generated packet (if positive), otherwise error code
int32_t voxl2_io_create_rpm_packet4(int16_t rpm0, int16_t rpm1, int16_t rpm2, int16_t rpm3,
				    uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3,
				    uint8_t *out, uint16_t out_size);

int32_t voxl2_io_create_hires_pwm_packet(uint32_t *pwm_val_ns, uint32_t cmd_cnt, uint8_t *out, uint16_t out_size);


// Create a packet for sending closed-loop RPM command and LED command to 4 ESCs, also request feedback from one ESC (with id=fb_id)
// Return value is the length of generated packet (if positive), otherwise error code
int32_t voxl2_io_create_rpm_packet4_fb(int16_t rpm0, int16_t rpm1, int16_t rpm2, int16_t rpm3,
				       uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3,
				       int32_t fb_id, uint8_t *out, uint16_t out_size);


//-------------------------------------------------------------------------
// Below are functions for processing incoming packets
//-------------------------------------------------------------------------


// Feed one char and see if we have accumulated a complete packet
int16_t   voxl2_io_packet_process_char(uint8_t c, VOXL2_IOPacket *packet);

// Get a pointer to the packet type from a pointer to VOXL2_IOPacket
static inline uint8_t   voxl2_io_packet_get_type(VOXL2_IOPacket *packet)        { return packet->buffer[VOXL2_IO_PACKET_POS_TYPE];    }

// Get a pointer to the packet type from a uint8_t pointer that points to the raw packet data as it comes from UART port
static inline uint8_t   voxl2_io_packet_raw_get_type(uint8_t *packet)      { return packet[VOXL2_IO_PACKET_POS_TYPE];            }

//get a pointer to the packet payload from a pointer to VOXL2_IOPacket
static inline uint8_t *voxl2_io_packet_get_data_ptr(VOXL2_IOPacket *packet)    { return &(packet->buffer[VOXL2_IO_PACKET_POS_DATA]); }

// Get a pointer to the packet payload from a uint8_t pointer that points to the raw packet data as it comes from UART port
static inline uint8_t *voxl2_io_packet_raw_get_data_ptr(uint8_t *packet)  { return &(packet[VOXL2_IO_PACKET_POS_DATA]);         }

// Get the total size (length) in bytes of the packet
static inline uint8_t   voxl2_io_packet_get_size(VOXL2_IOPacket *packet)        { return packet->buffer[VOXL2_IO_PACKET_POS_LENGTH];  }

// Get checksum of the packet from a pointer to VOXL2_IOPacket
static inline uint16_t  voxl2_io_packet_checksum_get(VOXL2_IOPacket *packet)    { return packet->crc;                            }

// Calculate the checksum of a data array. Used for packet generation / processing
static inline uint16_t  voxl2_io_packet_checksum(uint8_t *buf, uint16_t size)
{
	uint16_t crc = voxl2_io_crc16_init();
	return voxl2_io_crc16(crc, buf, size);
}

// Reset the checksum of the incoming packet. Used internally for packet reception
static inline void voxl2_io_packet_checksum_reset(VOXL2_IOPacket *packet)       { packet->crc = voxl2_io_crc16_init();                    }

// Process one character for checksum calculation while receiving a packet (used internally for packet reception)
static inline void voxl2_io_packet_checksum_process_char(VOXL2_IOPacket *packet, uint8_t c)
{
	packet->crc = voxl2_io_crc16_byte(packet->crc, c);
}


// Initialize an instance of an VOXL2_IOPacket. This should be called once before using an instance of VOXL2_IOPacket
static inline void voxl2_io_packet_init(VOXL2_IOPacket *packet)
{
	packet->len_received = 0;
	packet->len_expected = 0;
	packet->bp           = 0;

	voxl2_io_packet_checksum_reset(packet);
}

// Reset status of the packet that is being parsed. Effectively, this achieves the same thing as _packet_init
// so that _packet_init may be redundant
static inline void voxl2_io_packet_reset(VOXL2_IOPacket *packet)
{
	packet->len_received = 0;
}

#endif //VOXL2_IO_PACKET

#ifdef __cplusplus
}
#endif
