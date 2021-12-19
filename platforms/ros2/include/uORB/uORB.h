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

#pragma once


#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>


/**
 * Object metadata.
 */
struct orb_metadata {
	const char *o_name;		/**< unique object name */
	const uint16_t o_size;		/**< object size */
	const uint16_t o_size_no_padding;	/**< object size w/o padding at the end (for logger) */
	const char *o_fields;		/**< semicolon separated list of fields (with type) */
	uint8_t o_id;			/**< ORB_ID enum */
};

typedef const struct orb_metadata *orb_id_t;

/**
 * Maximum number of multi topic instances. This must be <= 10 (because it's the last char of the node path)
 */
#if defined(CONSTRAINED_MEMORY)
# define ORB_MULTI_MAX_INSTANCES 4
#else
# define ORB_MULTI_MAX_INSTANCES 10
#endif



#define ORB_ID(_name)		&__orb_##_name

#if defined(__cplusplus)
# define ORB_DECLARE(_name)		extern "C" const struct orb_metadata __orb_##_name __EXPORT
#else
# define ORB_DECLARE(_name)		extern const struct orb_metadata __orb_##_name __EXPORT
#endif


#define ORB_DEFINE(_name, _struct, _size_no_padding, _fields, _orb_id_enum)		\
	const struct orb_metadata __orb_##_name = {	\
		#_name,					\
		sizeof(_struct),		\
		_size_no_padding,			\
		_fields,				\
		_orb_id_enum				\
	}; struct hack

/**
 * Print a topic to console. Do not call this directly, use print_message() instead.
 * @param meta orb topic metadata
 * @param data expected to be aligned to the largest member
 */
void orb_print_message_internal(const struct orb_metadata *meta, const void *data, bool print_topic_name);

#ifndef PX4_INFO_RAW
# define PX4_INFO_RAW		printf
#endif

#ifdef __cplusplus

#include <uORBTopics.hpp>

#endif // __cplusplus
