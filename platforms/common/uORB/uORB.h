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

#pragma once

/**
 * @file uORB.h
 * API for the uORB lightweight object broker.
 */

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

/**
 * Generates a pointer to the uORB metadata structure for
 * a given topic.
 *
 * The topic must have been declared previously in scope
 * with ORB_DECLARE().
 *
 * @param _name		The name of the topic.
 */
#define ORB_ID(_name)		&__orb_##_name

/**
 * Declare (prototype) the uORB metadata for a topic (used by code generators).
 *
 * @param _name		The name of the topic.
 */
#if defined(__cplusplus)
# define ORB_DECLARE(_name)		extern "C" const struct orb_metadata __orb_##_name __EXPORT
#else
# define ORB_DECLARE(_name)		extern const struct orb_metadata __orb_##_name __EXPORT
#endif

/**
 * Define (instantiate) the uORB metadata for a topic.
 *
 * The uORB metadata is used to help ensure that updates and
 * copies are accessing the right data.
 *
 * Note that there must be no more than one instance of this macro
 * for each topic.
 *
 * @param _name		The name of the topic.
 * @param _struct	The structure the topic provides.
 * @param _size_no_padding	Struct size w/o padding at the end
 * @param _fields	All fields in a semicolon separated list e.g: "float[3] position;bool armed"
 * @param _orb_id_enum	ORB ID enum e.g.: ORB_ID::vehicle_status
 */
#define ORB_DEFINE(_name, _struct, _size_no_padding, _fields, _orb_id_enum)		\
	const struct orb_metadata __orb_##_name = {	\
		#_name,					\
		sizeof(_struct),		\
		_size_no_padding,			\
		_fields,				\
		_orb_id_enum				\
	}; struct hack

__BEGIN_DECLS

int uorb_start(void);
int uorb_status(void);
int uorb_top(char **topic_filter, int num_filters);

/**
 * ORB topic advertiser handle.
 *
 * Advertiser handles are global; once obtained they can be shared freely
 * and do not need to be closed or released.
 *
 * This permits publication from interrupt context and other contexts where
 * a file-descriptor-based handle would not otherwise be in scope for the
 * publisher.
 */
typedef void 	*orb_advert_t;
typedef void 	*orb_sub_t;

static inline bool orb_advert_valid(orb_advert_t handle) {return handle != NULL;}
static const orb_advert_t ORB_ADVERT_INVALID = NULL;
static inline bool orb_sub_valid(orb_sub_t handle) {return handle != NULL;}
static const orb_sub_t ORB_SUB_INVALID = NULL;

/**
 * @see uORB::Manager::orb_advertise()
 */
extern orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data) __EXPORT;

/**
 * @see uORB::Manager::orb_advertise()
 */
extern orb_advert_t orb_advertise_queue(const struct orb_metadata *meta, const void *data,
					unsigned int queue_size) __EXPORT;

/**
 * @see uORB::Manager::orb_advertise_multi()
 */
extern orb_advert_t orb_advertise_multi(const struct orb_metadata *meta, const void *data, int *instance) __EXPORT;

/**
 * @see uORB::Manager::orb_advertise_multi()
 */
extern orb_advert_t orb_advertise_multi_queue(const struct orb_metadata *meta, const void *data, int *instance,
		unsigned int queue_size) __EXPORT;

/**
 * @see uORB::Manager::orb_unadvertise()
 */
extern int orb_unadvertise(orb_advert_t handle) __EXPORT;

/**
 * @see uORB::Manager::orb_publish()
 */
extern int	orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data) __EXPORT;

/**
 * Advertise as the publisher of a topic.
 *
 * This performs the initial advertisement of a topic; it creates the topic
 * node in /obj if required and publishes the initial data.
 *
 * @see uORB::Manager::orb_advertise_multi() for meaning of the individual parameters
 */
static inline int orb_publish_auto(const struct orb_metadata *meta, orb_advert_t *handle, const void *data,
				   int *instance)
{
	if (!*handle) {
		*handle = orb_advertise_multi(meta, data, instance);

		if (*handle) {
			return 0;
		}

	} else {
		return orb_publish(meta, *handle, data);
	}

	return -1;
}

/**
 * @see uORB::Manager::orb_subscribe()
 */
extern orb_sub_t	orb_subscribe(const struct orb_metadata *meta) __EXPORT;

/**
 * @see uORB::Manager::orb_subscribe_multi()
 */
extern orb_sub_t	orb_subscribe_multi(const struct orb_metadata *meta, unsigned instance) __EXPORT;

/**
 * @see uORB::Manager::orb_unsubscribe()
 */
extern int	orb_unsubscribe(orb_sub_t handle) __EXPORT;

/**
 * @see uORB::Manager::orb_copy()
 */
extern int	orb_copy(const struct orb_metadata *meta, orb_sub_t handle, void *buffer) __EXPORT;

/**
 * @see uORB::Manager::orb_check()
 */
extern int	orb_check(orb_sub_t handle, bool *updated) __EXPORT;

/**
 * @see uORB::Manager::orb_exists()
 */
extern int	orb_exists(const struct orb_metadata *meta, int instance) __EXPORT;

/**
 * Get the number of published instances of a topic group
 *
 * @param meta    ORB topic metadata.
 * @return    The number of published instances of this topic
 */
extern int	orb_group_count(const struct orb_metadata *meta) __EXPORT;

/**
 * @see uORB::Manager::orb_set_interval()
 */
extern int	orb_set_interval(orb_sub_t handle, unsigned interval) __EXPORT;

/**
 * @see uORB::Manager::orb_get_interval()
 */
extern int	orb_get_interval(orb_sub_t handle, unsigned *interval) __EXPORT;

/**
 * Returns the C type string from a short type in o_fields metadata, or nullptr
 * if not a short type
 */
const char *orb_get_c_type(unsigned char short_type);

/**
 * Print a topic to console. Do not call this directly, use print_message() instead.
 * @param meta orb topic metadata
 * @param data expected to be aligned to the largest member
 */
void orb_print_message_internal(const struct orb_metadata *meta, const void *data, bool print_topic_name);


__END_DECLS

/* Diverse uORB header defines */ //XXX: move to better location
typedef uint8_t arming_state_t;
typedef uint8_t main_state_t;
typedef uint8_t hil_state_t;
typedef uint8_t navigation_state_t;
typedef uint8_t switch_pos_t;

