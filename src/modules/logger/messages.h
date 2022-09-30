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

#pragma once

#include <cstdint>

enum class ULogMessageType : uint8_t {
	FORMAT = 'F',
	DATA = 'D',
	INFO = 'I',
	INFO_MULTIPLE = 'M',
	PARAMETER = 'P',
	PARAMETER_DEFAULT = 'Q',
	ADD_LOGGED_MSG = 'A',
	REMOVE_LOGGED_MSG = 'R',
	SYNC = 'S',
	DROPOUT = 'O',
	LOGGING = 'L',
	LOGGING_TAGGED = 'C',
	FLAG_BITS = 'B',
};


/* declare message data structs with byte alignment (no padding) */
#pragma pack(push, 1)

/** first bytes of the file */
struct ulog_file_header_s {
	uint8_t magic[8];
	uint64_t timestamp;
};

/** first bytes of the crypto key file */
struct ulog_key_header_s {
	/* magic identifying the file content */
	uint8_t magic[7];

	/* version of this header file */
	uint8_t hdr_ver;

	/* file creation timestamp */
	uint64_t timestamp;

	/* crypto algorithm used for key exchange */
	uint8_t exchange_algorithm;

	/* encryption key index used for key exchange */
	uint8_t exchange_key;

	/* size of the key */
	uint16_t key_size;

	/* size of logfile crypto algoritm initialization data, e.g. nonce */
	uint16_t initdata_size;

	/* actual data (initdata+key) */
	uint8_t	data[0];
};


/**
 * @brief Message Header for the ULog
 *
 * This header components that is in the beginning of every ULog messages that gets written into
 * Definitions section as well as the Data section of the ULog file.
 */
struct ulog_message_header_s {
	uint16_t msg_size; ///< Size of the message excluding the header size
	uint8_t msg_type; ///< Message type, which is one of the ASCII alphabet, defined in ULogMessageType
};

#define ULOG_MSG_HEADER_LEN 3 // Length of the header in bytes: accounts for msg_size (2 bytes) and msg_type (1 byte)

/**
 * @brief Format Message
 *
 * This message describes a single ULog topic's name and it's inner fields. The inner fields can have the type
 * as defined in the uORB message file (e.g. msg/action_request.msg). Including other uORB topics, which is the
 * nested type case.
 *
 * @param format Format of the uORB topic in the format: "message_name:field0;field1;"
 * Example: "action_request:uint64_t timestamp;uint8_t action;uint8_t source;uint8_t mode;uint8_t[5] _padding0;"
 */
struct ulog_message_format_s {
	uint16_t msg_size; ///< size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::FORMAT);

	char format[1500];
};

/**
 * @brief Subscription Message
 *
 * This message describes which uORB topic the logger has subscribed to.
 *
 * Used for indicating which msg_id corresponds to which uORB topic name (message_name)
 */
struct ulog_message_add_logged_s {
	uint16_t msg_size; ///< size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::ADD_LOGGED_MSG);

	uint8_t multi_id; ///< Multi instance id, if the topic is one of a multi instance uORB topic
	uint16_t msg_id; ///< Message ID, an internally tracked id in the logger, which matches with the msg_id in ulog_message_data_s message
	char message_name[255]; ///< Name of the uORB topic
};

/**
 * @brief Unsubscription Message
 *
 * This message describes which uORB topic the logger has unsubscribed from.
 */
struct ulog_message_remove_logged_s {
	uint16_t msg_size; ///< size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::REMOVE_LOGGED_MSG);

	uint16_t msg_id;
};

/**
 * @brief Sync Message
 */
struct ulog_message_sync_s {
	uint16_t msg_size; ///< size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::SYNC);

	uint8_t sync_magic[8];
};

struct ulog_message_dropout_s {
	uint16_t msg_size = sizeof(uint16_t); ///< size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::DROPOUT);

	uint16_t duration; ///< in ms
};

/**
 * @brief Logged Data Message
 *
 * Includes the binary data of the uORB topic with the corresponding logger's internal msg_id.
 *
 * The uint8_t data[] section follows after the msg_id part, but the message struct only defines up until the msg_id,
 * since the data length varies per each uORB topic (the struct size of the uORB message).
 */
struct ulog_message_data_s {
	uint16_t msg_size; ///< size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::DATA);

	uint16_t msg_id;
};

/**
 * @brief Information Message
 *
 * Writes the dictionary type key:value relationship of any kind of information.
 *
 * Example: key_value_str[] = "char[5] sys_toolchain_ver9.4.0"
 */
struct ulog_message_info_s {
	uint16_t msg_size; ///< size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::INFO);

	uint8_t key_len; ///<  Length of the 'key'
	char key_value_str[255]; ///<  String with the key and value information
};

/**
 * @brief Multiple Information Message
 *
 * Writes the dictionary type key:value relationship of any kind of information, but
 * for the ones which has a long value that can't be contained in a single Information Message.
 */
struct ulog_message_info_multiple_s {
	uint16_t msg_size; ///< size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::INFO_MULTIPLE);

	uint8_t is_continued; ///< Can be used for arrays: set to 1, if this message is part of the previous with the same key
	uint8_t key_len;      ///< Length of the 'key'
	char key_value_str[1200]; ///< String with the key and value information
};

/**
 * @brief Logged String Message
 *
 * Logged string, either from PX4_INFO() macro calls or MAVLink information messages, etc.
 * Useful for Debugging.
 */
struct ulog_message_logging_s {
	uint16_t msg_size; ///< size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::LOGGING);

	uint8_t log_level; ///< same levels as in the linux kernel
	uint64_t timestamp;
	char message[128]; ///< defines the maximum length of a logged message string
};

struct ulog_message_logging_tagged_s {
	uint16_t msg_size; ///< size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::LOGGING_TAGGED);

	uint8_t log_level; ///< same levels as in the linux kernel
	uint16_t tag;
	uint64_t timestamp;
	char message[128]; ///< defines the maximum length of a logged message string
};

/**
 * @brief Parameter Message
 *
 * Includes a parameter value information in the format "<TYPE> <PARAMETER_NAME> <VALUE>"
 */
struct ulog_message_parameter_s {
	uint16_t msg_size;
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::PARAMETER);

	uint8_t key_len;
	char key_value_str[255]; ///<  String with the key and value information
};

/**
 * @brief ULog default parameter type flags
 *
 * These flags indicate whether the default parameter defined in the ulog_message_parameter_default_s message
 * is System's default or the User setup's default value. As user can set a custom default parameter for each
 * vehicle setup that overrides the system's default value.
 */
enum class ulog_parameter_default_type_t : uint8_t {
	system = (1 << 0), ///< System wide default parameter
	current_setup = (1 << 1) ///<  Custom setup default parameter
};

inline ulog_parameter_default_type_t operator|(ulog_parameter_default_type_t a, ulog_parameter_default_type_t b)
{
	return static_cast<ulog_parameter_default_type_t>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

/**
 * @brief Default Parameter Message
 *
 * This message defines a parameter name and it's default value. Which is useful for figuring out in ULog analyzer
 * which Parameters are non-default (User modified), which can give a quick insight into how the vehicle is configured,
 * since the default parameter values are well known & understood (e.g. PID Tuning values)
 */
struct ulog_message_parameter_default_s {
	uint16_t msg_size;
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::PARAMETER_DEFAULT);

	ulog_parameter_default_type_t default_types;
	uint8_t key_len;
	char key_value_str[255]; ///<  String with the key and value information
};


#define ULOG_INCOMPAT_FLAG0_DATA_APPENDED_MASK (1<<0)

#define ULOG_COMPAT_FLAG0_DEFAULT_PARAMETERS_MASK (1<<0)

struct ulog_message_flag_bits_s {
	uint16_t msg_size;
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::FLAG_BITS);

	uint8_t compat_flags[8];
	uint8_t incompat_flags[8]; ///< @see ULOG_INCOMPAT_FLAG_*
	uint64_t appended_offsets[3]; ///< file offset(s) for appended data if ULOG_INCOMPAT_FLAG0_DATA_APPENDED_MASK is set
};

#pragma pack(pop)
