/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *       Author: Ben Dyer <ben_dyer@mac.com>
 *               Pavel Kirienko <pavel.kirienko@zubax.com>
 *               David Sidrane <david_s5@nscdg.com>
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

#pragma once

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

#include <stdint.h>
#include <stdlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UAVCAN_SERVICE_RETRIES          3
#define UAVCAN_SERVICE_TIMEOUT_MS       1000
#define UAVCAN_NODESTATUS_INTERVAL_MS   500

#define sizeof_member(t, m) sizeof(((t *)0)->m)

#define UAVCAN_STRLEN(x) sizeof((x))-1

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

typedef enum {
	CAN_OK = 0,
	CAN_BOOT_TIMEOUT,
	CAN_ERROR
} can_error_t;

/* UAVCAN message formats */
typedef enum {
	PRIORITY_HIGH = 0,
	PRIORITY_NORMAL = 1,
	PRIORITY_SERVICE = 2,
	PRIORITY_LOW = 3
} uavcan_transferpriority_t;

typedef struct packed_struct uavcan_frame_id_t {
	uint8_t transfer_id;
	uint8_t last_frame;
	uint8_t frame_index;
	uint8_t source_node_id;
	uint16_t data_type_id;
	uavcan_transferpriority_t priority;
	/* Only for priority == PRIORITY_SERVICE */
	uint8_t request_not_response;
	/* Only for priority != PRIORITY_SERVICE */
	uint8_t broadcast_not_unicast;
} uavcan_frame_id_t;

typedef struct packed_struct uavcan_nodestatus_t {
	uint32_t uptime_sec;
	uint8_t status_code;
	uint16_t vendor_specific_status_code;
	uint8_t  msb_vendor_specific_status_code;
} uavcan_nodestatus_t;

#define UAVCAN_NODESTATUS_DTID 1000u
#define UAVCAN_NODESTATUS_STATUS_INITIALIZING 1u
#define UAVCAN_NODESTATUS_STATUS_WARNING 2u
#define UAVCAN_NODESTATUS_STATUS_CRITICAL 3u
#define UAVCAN_NODESTATUS_STATUS_PACKED_SIZE 7


typedef struct packed_struct uavcan_softwareversion_t {
	uint8_t major;
	uint8_t minor;
	uint8_t optional_field_mask;
	uint32_t vcs_commit;
	uint64_t image_crc;
} uavcan_softwareversion_t;

typedef struct packed_struct uavcan_hardwareversion_t {
	uint8_t major;
	uint8_t minor;
	uint8_t unique_id[16];
	uint8_t certificate_of_authenticity_length;
	uint8_t certificate_of_authenticity[255];
} uavcan_hardwareversion_t;

typedef struct packed_struct uavcan_getnodeinfo_response_t {
	uint8_t nodestatus[UAVCAN_NODESTATUS_STATUS_PACKED_SIZE];

	uavcan_softwareversion_t software_version;
	uavcan_hardwareversion_t hardware_version;

	uint8_t name[80];
	uint8_t name_length;
} uavcan_getnodeinfo_response_t;

#define UAVCAN_GETNODEINFO_DTID 200u
#define UAVCAN_GETNODEINFO_CRC 0xcb98u


typedef struct packed_struct uavcan_allocation_t {
	uint8_t node_id; /* bottom bit is the first part flag */
	uint8_t unique_id[16];
} uavcan_allocation_t;

#define UAVCAN_DYNAMICNODEIDALLOCATION_DTID 1010u


typedef struct packed_struct uavcan_logmessage_t {
	uint8_t level;
	uint8_t message[2];
} uavcan_logmessage_t;

#define UAVCAN_LOGMESSAGE_DTID 1790u
#define UAVCAN_LOGMESSAGE_LEVEL_DEBUG 0u
#define UAVCAN_LOGMESSAGE_LEVEL_INFO 1u
#define UAVCAN_LOGMESSAGE_LEVEL_WARNING 2u
#define UAVCAN_LOGMESSAGE_LEVEL_ERROR 3u
#define UAVCAN_LOGMESSAGE_SOURCE_0 'B'
#define UAVCAN_LOGMESSAGE_SOURCE_1 'O'
#define UAVCAN_LOGMESSAGE_SOURCE_2 'O'
#define UAVCAN_LOGMESSAGE_SOURCE_3 'T'

#define UAVCAN_LOGMESSAGE_STAGE_INIT 'I'
#define UAVCAN_LOGMESSAGE_STAGE_GET_INFO 'G'
#define UAVCAN_LOGMESSAGE_STAGE_ERASE 'E'
#define UAVCAN_LOGMESSAGE_STAGE_READ 'R'
#define UAVCAN_LOGMESSAGE_STAGE_PROGRAM 'P'
#define UAVCAN_LOGMESSAGE_STAGE_VALIDATE 'V'
#define UAVCAN_LOGMESSAGE_STAGE_FINALIZE 'F'
#define UAVCAN_LOGMESSAGE_RESULT_START 's'
#define UAVCAN_LOGMESSAGE_RESULT_FAIL 'f'
#define UAVCAN_LOGMESSAGE_RESULT_OK 'o'


typedef struct packed_struct uavcan_beginfirmwareupdate_request_t {
	uint8_t source_node_id;
	uint8_t path[200];
	uint8_t path_length;
} uavcan_beginfirmwareupdate_request_t;

typedef struct packed_struct uavcan_beginfirmwareupdate_response_t {
	uint8_t error;
} uavcan_beginfirmwareupdate_response_t;

#define UAVCAN_BEGINFIRMWAREUPDATE_DTID 210u
#define UAVCAN_BEGINFIRMWAREUPDATE_CRC 0x729Eu
#define UAVCAN_BEGINFIRMWAREUPDATE_ERROR_OK 0u
#define UAVCAN_BEGINFIRMWAREUPDATE_ERROR_INVALID_MODE 1u
#define UAVCAN_BEGINFIRMWAREUPDATE_ERROR_IN_PROGRESS 2u
#define UAVCAN_BEGINFIRMWAREUPDATE_ERROR_UNKNOWN 255u


typedef struct packed_struct uavcan_getinfo_request_t {
	uint8_t path[200];
	uint8_t path_length;
} uavcan_getinfo_request_t;

typedef struct packed_struct uavcan_getinfo_response_t {
	uint32_t size;
	uint8_t sizemsb;
	uint16_t error;
	uint8_t entry_type;
} uavcan_getinfo_response_t;

#define UAVCAN_GETINFO_DTID 215u
#define UAVCAN_GETINFO_CRC 0x14b9
#define UAVCAN_GETINFO_ENTRY_TYPE_FLAG_FILE 0x01u
#define UAVCAN_GETINFO_ENTRY_TYPE_FLAG_DIRECTORY 0x02u
#define UAVCAN_GETINFO_ENTRY_TYPE_FLAG_SYMLINK 0x04u
#define UAVCAN_GETINFO_ENTRY_TYPE_FLAG_READABLE 0x08u
#define UAVCAN_GETINFO_ENTRY_TYPE_FLAG_WRITEABLE 0x10u


#define UAVCAN_FILE_READ_MAX_PATH_LEN 200

typedef struct packed_struct uavcan_read_request_t {
	uint32_t offset;
	uint8_t offsetmsb;
	uint8_t path[UAVCAN_FILE_READ_MAX_PATH_LEN];
	uint8_t path_length;
} uavcan_read_request_t;

#define UAVCAN_FILE_READ_REQUEST_FIXED_SIZE (sizeof(uavcan_read_request_t) - \
		(UAVCAN_FILE_READ_MAX_PATH_LEN + \
		 sizeof_member(uavcan_read_request_t, path_length)))

#define UAVCAN_FILE_READ_MAX_LEN 256

typedef struct packed_struct uavcan_read_response_t {
	uint16_t error;
	uint8_t data[UAVCAN_FILE_READ_MAX_LEN];
	uint16_t data_length;
} uavcan_read_response_t;

#define UAVCAN_READ_DTID 218u
#define UAVCAN_READ_CRC 0x2f12

#define UAVCAN_FILE_ERROR_OK 0u
/* Left the others out for now because we don't really care why it failed */


#define UAVCAN_ALLOCATION_CRC 0xF258u

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/****************************************************************************
 * Name: uavcan_pack_nodestatus
 *
 * Description:
 *   This function formats the data of a uavcan_nodestatus_t structure into
 *   an array of bytes.
 *
 * Input Parameters:
 *   data    - The array of bytes to populate.
 *   message - The uavcan_nodestatus_t to pack into the data
 *
 * Returned value:
 *   Number of bytes written.
 *
 ****************************************************************************/

size_t uavcan_pack_nodestatus(uint8_t *data,
			      const uavcan_nodestatus_t *message);

/****************************************************************************
 * Name: uavcan_pack_logmessage
 *
 * Description:
 *   This function formats the data of a uavcan_logmessage_t structure into
 *   an array of bytes.
 *
 * Input Parameters:
 *   data    - The array of bytes to populate.
 *   message - The uavcan_logmessage_t to pack into the data
 *
 * Returned value:
 *   Number of bytes written.
 *
 ****************************************************************************/

size_t uavcan_pack_logmessage(uint8_t *data,
			      const uavcan_logmessage_t *message);

/****************************************************************************
 * Name: uavcan_make_service_frame_id
 *
 * Description:
 *   This function formats the data of a uavcan_frame_id_t structure
 *   into a unit32.
 *
 * Input Parameters:
 *   frame_id - The uavcan_frame_id_t to pack.
 *
 * Returned value:
 *   A unit32 that is the frame id formed from packing the uavcan_frame_id_t.
 *
 ****************************************************************************/

uint32_t uavcan_make_service_frame_id(const uavcan_frame_id_t *frame_id);


/****************************************************************************
 * Name: uavcan_parse_frame_id
 *
 * Description:
 *   This function formats the data of a uavcan frame id contained in a
 *   uint32 into uavcan_frame_id_t structure.
 *
 * Input Parameters:
 *   out_frame_id     - A pointer to a uavcan_frame_id_t parse the
 *                      frame id into.
 *   frame_id         - The frame id to parse into the
 *                      uavcan_frame_id_t
 *   expected_type_id - The expected uavcan type ID that has been
 *                      parsed into out_frame_id's data_type_id field
 *
 * Returned value:
 *   The result of comparing the data_type_id to the expected_type_id
 *   Non Zero if they match otherwise zero.
 *
 ****************************************************************************/

int uavcan_parse_frame_id(uavcan_frame_id_t *out_frame_id,
			  uint32_t frame_id,
			  uint16_t expected_type_id);

/****************************************************************************
 * Name: uavcan_tx_nodestatus
 *
 * Description:
 *   This function sends a uavcan nodestatus message
 *
 * Input Parameters:
 *   node_id     - This node's node id
 *   uptime_sec  - This node's uptime in seconds.
 *   status_code - This node's current status code
 *
 * Returned value:
 *   None.
 *
 ****************************************************************************/

void uavcan_tx_nodestatus(uint8_t node_id, uint32_t uptime_sec,
			  uint8_t status_code);

/****************************************************************************
 * Name: uavcan_tx_log_message
 *
 * Description:
 *   This functions sends uavcan logmessage type data. See uavcan/protocol.h
 *   UAVCAN_LOGMESSAGE_xxx defines.
 *
 * Input Parameters:
 *   node_id - This node's node id
 *   level   - Log Level of the logmessage DEBUG, INFO, WARN, ERROR
 *   stage   - The Stage the application is at. see UAVCAN_LOGMESSAGE_STAGE_x
 *   status  - The status of that stage. Start, Fail OK
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void uavcan_tx_log_message(uint8_t node_id, uint8_t level, uint8_t stage,
			   uint8_t status);

/****************************************************************************
 * Name: uavcan_tx_allocation_message
 *
 * Description:
 *   This function sends a uavcan allocation message.
 *
 * Input Parameters:
 *   requested_node_id - This node's preferred node id 0 for no preference.
 *   unique_id_length  - This node's length of it's unique identifier.
 *   unique_id         - A pointer to the bytes that represent unique
 *                       identifier.
 *   unique_id_offset  - The offset equal 0 or the number of bytes in the
 *                       the last received message that matched the unique ID
 *                       field.
 *
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

void uavcan_tx_allocation_message(uint8_t requested_node_id,
				  size_t unique_id_length,
				  const uint8_t *unique_id,
				  uint8_t unique_id_offset);

/****************************************************************************
 * Name: uavcan_tx_getnodeinfo_response
 *
 * Description:
 *   This function sends a uavcan getnodeinfo response to a getnodeinfo
 *   request message.
 *
 * Input Parameters:
 *   node_id      - This node's node id
 *   response     - A pointer to this node's uavcan_getnodeinfo_response_t
 *                  response data.
 *   dest_node_id - The destination node id to send the message to.
 *   transfer_id  - An incrementing count used to correlate a received
 *                  message to this transmitted message.
 *
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

void uavcan_tx_getnodeinfo_response(uint8_t node_id,
				    uavcan_getnodeinfo_response_t *response,
				    uint8_t dest_node_id,
				    uint8_t transfer_id);

/****************************************************************************
 * Name: uavcan_rx_beginfirmwareupdate_request
 *
 * Description:
 *   This function attempts to receive a uavcan beginfirmwareupdate request
 *   message
 *
 * Input Parameters:
 *   node_id  - This node's node id
 *   request  - A pointer a uavcan_beginfirmwareupdate_request_t to
 *              receive the request data into.
 *   frame_id - A pointer to a uavcan_frame_id_t to return frame_id
 *              components in.
 *
 *
 * Returned value:
 *   CAN_OK on Success and CAN_ERROR otherwise.
 *
 ****************************************************************************/

can_error_t uavcan_rx_beginfirmwareupdate_request(uint8_t node_id,
		uavcan_beginfirmwareupdate_request_t *request,
		uavcan_frame_id_t *frame_id);

/****************************************************************************
 * Name: uavcan_tx_read_request
 *
 * Description:
 *   This function sends a uavcan read request message.
 *
 * Input Parameters:
 *   node_id      - This node's node id
 *   request      - A pointer a uavcan_read_request_t to
 *                  send.
 *   transfer_id  - An incrementing count used to correlate a received
 *                  message to this transmitted message.
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

void uavcan_tx_read_request(uint8_t node_id,
			    const uavcan_read_request_t *request,
			    uint8_t dest_node_id,
			    uint8_t transfer_id);

/****************************************************************************
 * Name: uavcan_rx_read_response
 *
 * Description:
 *   This function attempts to receive a uavcan read response message.
 *
 * Input Parameters:
 *   node_id      - This node's node id
 *   response     - A pointer a uavcan_read_response_t to receive the
 *                  response data into.
 *   dest_node_id - The remote node id to expect the message from.
 *   transfer_id  - An incrementing count used to correlate a received
 *                  message to this transmitted message.
 *   timeout_ms -   The number of milliseconds to wait for a response
 *
 *
 * Returned value:
 *   CAN_OK on Success and CAN_ERROR otherwise.
 *
 ****************************************************************************/

can_error_t uavcan_rx_read_response(uint8_t node_id,
				    uavcan_read_response_t *response,
				    uint8_t dest_node_id,
				    uint8_t transfer_id,
				    uint32_t timeout_ms);

/****************************************************************************
 * Name: uavcan_tx_getinfo_request
 *
 * Description:
 *   This function sends a uavcan getinfo request message.
 *
 * Input Parameters:
 *   node_id      - This node's node id
 *   request      - A pointer a uavcan_getinfo_request_t to
 *                  send.
 *   dest_node_id - The destination node id to send the message to.
 *   transfer_id  - An incrementing count used to correlate a received
 *                  message to this transmitted message.
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

void uavcan_tx_getinfo_request(uint8_t node_id,
			       const uavcan_getinfo_request_t *request,
			       uint8_t dest_node_id,
			       uint8_t transfer_id);

/****************************************************************************
 * Name: uavcan_rx_getinfo_response
 *
 * Description:
 *   This function attempts to receive a uavcan getinfo response message.
 *
 * Input Parameters:
 *   node_id      - This node's node id
 *   response     - A pointer a uavcan_getinfo_response_t to receive the
 *                  response data into.
 *   dest_node_id - The remote node id to expect the message from.
 *   transfer_id  - The expected transfer_id used to correlate this response
 *                  message to the transmitted request message.
 *   timeout_ms -   The number of milliseconds to wait for a response
 *
 *
 * Returned value:
 *   CAN_OK on Success and CAN_ERROR otherwise.
 *
 ****************************************************************************/
can_error_t uavcan_rx_getinfo_response(uint8_t node_id,
				       uavcan_getinfo_response_t *response,
				       uint8_t dest_node_id,
				       uint8_t transfer_id,
				       uint32_t timeout_ms);
