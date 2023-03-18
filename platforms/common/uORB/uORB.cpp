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
 * @file uORB.cpp
 * A lightweight object broker.
 */

#include "uORB.h"

#include "uORBManager.hpp"
#include "uORBCommon.hpp"


#include <lib/drivers/device/Device.hpp>
#include <matrix/Quaternion.hpp>
#include <mathlib/mathlib.h>

#ifdef __PX4_NUTTX
#include <sys/boardctl.h>
#endif

static uORB::DeviceMaster *g_dev = nullptr;

int uorb_start(void)
{
	if (g_dev != nullptr) {
		PX4_WARN("already loaded");
		/* user wanted to start uorb, its already running, no error */
		return 0;
	}

	if (!uORB::Manager::initialize()) {
		PX4_ERR("uorb manager alloc failed");
		return -ENOMEM;
	}

#if !defined(__PX4_NUTTX) || defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
	/* create the driver */
	g_dev = uORB::Manager::get_instance()->get_device_master();

	if (g_dev == nullptr) {
		return -errno;
	}

#endif

	return OK;
}

int uorb_status(void)
{
#if !defined(__PX4_NUTTX) || defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)

	if (g_dev != nullptr) {
		g_dev->printStatistics();

	} else {
		PX4_INFO("uorb is not running");
	}

#else
	boardctl(ORBIOCDEVMASTERCMD, ORB_DEVMASTER_STATUS);
#endif
	return OK;
}

int uorb_top(char **topic_filter, int num_filters)
{
#if !defined(__PX4_NUTTX) || defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)

	if (g_dev != nullptr) {
		g_dev->showTop(topic_filter, num_filters);

	} else {
		PX4_INFO("uorb is not running");
	}

#else
	boardctl(ORBIOCDEVMASTERCMD, ORB_DEVMASTER_TOP);
#endif
	return OK;
}

orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data)
{
	return uORB::Manager::get_instance()->orb_advertise(meta, data);
}

orb_advert_t orb_advertise_queue(const struct orb_metadata *meta, const void *data, unsigned int queue_size)
{
	return uORB::Manager::get_instance()->orb_advertise(meta, data, queue_size);
}

orb_advert_t orb_advertise_multi(const struct orb_metadata *meta, const void *data, int *instance)
{
	return uORB::Manager::get_instance()->orb_advertise_multi(meta, data, instance);
}

orb_advert_t orb_advertise_multi_queue(const struct orb_metadata *meta, const void *data, int *instance,
				       unsigned int queue_size)
{
	return uORB::Manager::get_instance()->orb_advertise_multi(meta, data, instance, queue_size);
}

int orb_unadvertise(orb_advert_t handle)
{
	return uORB::Manager::get_instance()->orb_unadvertise(handle);
}

int orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data)
{
	return uORB::Manager::get_instance()->orb_publish(meta, handle, data);
}

int orb_subscribe(const struct orb_metadata *meta)
{
	return uORB::Manager::get_instance()->orb_subscribe(meta);
}

int orb_subscribe_multi(const struct orb_metadata *meta, unsigned instance)
{
	return uORB::Manager::get_instance()->orb_subscribe_multi(meta, instance);
}

int orb_unsubscribe(int handle)
{
	return uORB::Manager::get_instance()->orb_unsubscribe(handle);
}

int orb_copy(const struct orb_metadata *meta, int handle, void *buffer)
{
	return uORB::Manager::get_instance()->orb_copy(meta, handle, buffer);
}

int orb_check(int handle, bool *updated)
{
	return uORB::Manager::get_instance()->orb_check(handle, updated);
}

int orb_exists(const struct orb_metadata *meta, int instance)
{
	return uORB::Manager::get_instance()->orb_exists(meta, instance);
}

int orb_group_count(const struct orb_metadata *meta)
{
	unsigned instance = 0;

	while (uORB::Manager::get_instance()->orb_exists(meta, instance) == OK) {
		++instance;
	};

	return instance;
}

int orb_set_interval(int handle, unsigned interval)
{
	return uORB::Manager::get_instance()->orb_set_interval(handle, interval);
}

int orb_get_interval(int handle, unsigned *interval)
{
	return uORB::Manager::get_instance()->orb_get_interval(handle, interval);
}

const char *orb_get_c_type(unsigned char short_type)
{
	// this matches with the uorb o_fields generator
	switch (short_type) {
	case 0x82: return "int8_t";

	case 0x83: return "int16_t";

	case 0x84: return "int32_t";

	case 0x85: return "int64_t";

	case 0x86: return "uint8_t";

	case 0x87: return "uint16_t";

	case 0x88: return "uint32_t";

	case 0x89: return "uint64_t";

	case 0x8a: return "float";

	case 0x8b: return "double";

	case 0x8c: return "bool";

	case 0x8d: return "char";
	}

	return nullptr;
}


void orb_print_message_internal(const orb_metadata *meta, const void *data, bool print_topic_name)
{
	if (print_topic_name) {
		PX4_INFO_RAW(" %s\n", meta->o_name);
	}

	const hrt_abstime now = hrt_absolute_time();
	hrt_abstime topic_timestamp = 0;

	const uint8_t *data_ptr = (const uint8_t *)data;
	int data_offset = 0;

	for (int format_idx = 0; meta->o_fields[format_idx] != 0;) {
		const char *end_field = strchr(meta->o_fields + format_idx, ';');

		if (!end_field) {
			PX4_ERR("Format error in %s", meta->o_fields);
			return;
		}

		const char *c_type = orb_get_c_type(meta->o_fields[format_idx]);
		const int end_field_idx = end_field - meta->o_fields;

		int array_idx = -1;
		int field_name_idx = -1;

		for (int field_idx = format_idx; field_idx != end_field_idx; ++field_idx) {
			if (meta->o_fields[field_idx] == '[') {
				array_idx = field_idx + 1;

			} else if (meta->o_fields[field_idx] == ' ') {
				field_name_idx = field_idx + 1;
				break;
			}
		}

		int array_size = 1;

		if (array_idx >= 0) {
			array_size = strtol(meta->o_fields + array_idx, nullptr, 10);
		}

		char field_name[80];
		size_t field_name_len = end_field_idx - field_name_idx;

		if (field_name_len >= sizeof(field_name)) {
			PX4_ERR("field name too long %s (max: %u)", meta->o_fields, (unsigned)sizeof(field_name));
			return;
		}

		memcpy(field_name, meta->o_fields + field_name_idx, field_name_len);
		field_name[field_name_len] = '\0';

		if (c_type) { // built-in type
			bool dont_print = false;

			// handle special cases
			if (strncmp(field_name, "_padding", 8) == 0) {
				dont_print = true;

			} else if (strcmp(c_type, "char") == 0 && array_size > 1) { // string
				PX4_INFO_RAW("    %s: \"%.*s\"\n", field_name, array_size, (char *)(data_ptr + data_offset));
				dont_print = true;
			}

			if (!dont_print) {
				PX4_INFO_RAW("    %s: ", field_name);
			}

			if (!dont_print && array_size > 1) {
				PX4_INFO_RAW("[");
			}

			const int previous_data_offset = data_offset;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align" // the caller ensures data is aligned

			for (int i = 0; i < array_size; ++i) {
				if (strcmp(c_type, "int8_t") == 0) {
					if (!dont_print) { PX4_INFO_RAW("%" PRIi8, *(int8_t *)(data_ptr + data_offset)); }

					data_offset += sizeof(int8_t);

				} else if (strcmp(c_type, "int16_t") == 0) {
					if (!dont_print) { PX4_INFO_RAW("%" PRIi16, *(int16_t *)(data_ptr + data_offset)); }

					data_offset += sizeof(int16_t);

				} else if (strcmp(c_type, "int32_t") == 0) {
					if (!dont_print) { PX4_INFO_RAW("%" PRIi32, *(int32_t *)(data_ptr + data_offset)); }

					data_offset += sizeof(int32_t);

				} else if (strcmp(c_type, "int64_t") == 0) {
					if (!dont_print) { PX4_INFO_RAW("%" PRIi64, *(int64_t *)(data_ptr + data_offset)); }

					data_offset += sizeof(int64_t);

				} else if (strcmp(c_type, "uint8_t") == 0) {
					if (!dont_print) { PX4_INFO_RAW("%" PRIu8, *(uint8_t *)(data_ptr + data_offset)); }

					data_offset += sizeof(uint8_t);

				} else if (strcmp(c_type, "uint16_t") == 0) {
					if (!dont_print) { PX4_INFO_RAW("%" PRIu16, *(uint16_t *)(data_ptr + data_offset)); }

					data_offset += sizeof(uint16_t);

				} else if (strcmp(c_type, "uint32_t") == 0) {
					if (!dont_print) { PX4_INFO_RAW("%" PRIu32, *(uint32_t *)(data_ptr + data_offset)); }

					data_offset += sizeof(uint32_t);

				} else if (strcmp(c_type, "uint64_t") == 0) {
					if (!dont_print) { PX4_INFO_RAW("%" PRIu64, *(uint64_t *)(data_ptr + data_offset)); }

					data_offset += sizeof(uint64_t);

				} else if (strcmp(c_type, "float") == 0) {
					if (!dont_print) { PX4_INFO_RAW("%.5f", (double) * (float *)(data_ptr + data_offset)); }

					data_offset += sizeof(float);

				} else if (strcmp(c_type, "double") == 0) {
					if (!dont_print) { PX4_INFO_RAW("%.6f", *(double *)(data_ptr + data_offset)); }

					data_offset += sizeof(double);

				} else if (strcmp(c_type, "bool") == 0) {
					if (!dont_print) { PX4_INFO_RAW("%s", *(bool *)(data_ptr + data_offset) ? "True" : "False"); }

					data_offset += sizeof(bool);

				} else if (strcmp(c_type, "char") == 0) {
					if (!dont_print) { PX4_INFO_RAW("%i", (int) * (char *)(data_ptr + data_offset)); }

					data_offset += sizeof(char);

				} else {
					PX4_ERR("unknown type: %s", c_type);
					return;
				}

				if (!dont_print && i < array_size - 1) {
					PX4_INFO_RAW(", ");
				}
			}

			if (!dont_print && array_size > 1) {
				PX4_INFO_RAW("]");
			}

			// handle special cases
			if (array_size == 1) {
				if (strcmp(c_type, "uint64_t") == 0 && strcmp(field_name, "timestamp") == 0) {
					topic_timestamp = *(uint64_t *)(data_ptr + previous_data_offset);

					if (topic_timestamp != 0) {
						PX4_INFO_RAW(" (%.6f seconds ago)", (double)((now - topic_timestamp) / 1e6f));
					}

				} else if (strcmp(c_type, "uint64_t") == 0 && strcmp(field_name, "timestamp_sample") == 0) {
					hrt_abstime timestamp = *(uint64_t *)(data_ptr + previous_data_offset);

					if (topic_timestamp != 0 && timestamp != 0) {
						PX4_INFO_RAW(" (%i us before timestamp)", (int)(topic_timestamp - timestamp));
					}

				} else if (strstr(field_name, "flags") != nullptr) {
					// bitfield
					unsigned field_size = 0;
					uint64_t value = 0;

					if (strcmp(c_type, "uint8_t") == 0) {
						field_size = sizeof(uint8_t);
						value = *(uint8_t *)(data_ptr + previous_data_offset);

					} else if (strcmp(c_type, "uint16_t") == 0) {
						field_size = sizeof(uint16_t);
						value = *(uint16_t *)(data_ptr + previous_data_offset);

					} else if (strcmp(c_type, "uint32_t") == 0) {
						field_size = sizeof(uint32_t);
						value = *(uint32_t *)(data_ptr + previous_data_offset);

					} else if (strcmp(c_type, "uint64_t") == 0) {
						field_size = sizeof(uint64_t);
						value = *(uint64_t *)(data_ptr + previous_data_offset);
					}

					if (field_size > 0 && value != 0) {
						PX4_INFO_RAW(" (0b");

						bool got_set_bit = false;

						for (int i = (field_size * 8) - 1; i >= 0; i--) {
							unsigned current_bit = (value >> i) & 1;
							got_set_bit |= current_bit;

							if (got_set_bit) {
								PX4_INFO_RAW("%u%s", current_bit, ((unsigned)i < (field_size * 8) - 1 && i % 4 == 0 && i > 0) ? "'" : "");
							}
						}

						PX4_INFO_RAW(")");
					}

				} else if (strcmp(c_type, "uint32_t") == 0 && strstr(field_name, "device_id") != nullptr) {
					// Device ID
					uint32_t device_id = *(uint32_t *)(data_ptr + previous_data_offset);
					char device_id_buffer[80];
					device::Device::device_id_print_buffer(device_id_buffer, sizeof(device_id_buffer), device_id);
					PX4_INFO_RAW(" (%s)", device_id_buffer);
				}

			} else if (array_size == 4 && strcmp(c_type, "float") == 0 && (strcmp(field_name, "q") == 0
					|| strncmp(field_name, "q_", 2) == 0)) {
				// attitude
				float *attitude = (float *)(data_ptr + previous_data_offset);
				matrix::Eulerf euler{matrix::Quatf{attitude}};
				PX4_INFO_RAW(" (Roll: %.1f deg, Pitch: %.1f deg, Yaw: %.1f deg)",
					     (double)math::degrees(euler(0)), (double)math::degrees(euler(1)), (double)math::degrees(euler(2)));
			}

#pragma GCC diagnostic pop

			PX4_INFO_RAW("\n");

		} else {

			// extract the topic name
			char topic_name[80];
			const size_t topic_name_len = array_size > 1 ? array_idx - format_idx - 1 : field_name_idx - format_idx - 1;

			if (topic_name_len >= sizeof(topic_name)) {
				PX4_ERR("topic name too long in %s (max: %u)", meta->o_name, (unsigned)sizeof(topic_name));
				return;
			}

			memcpy(topic_name, meta->o_fields + format_idx, topic_name_len);
			topic_name[topic_name_len] = '\0';

			// find the metadata
			const orb_metadata *const *topics = orb_get_topics();
			const orb_metadata *found_topic = nullptr;

			for (size_t i = 0; i < orb_topics_count(); i++) {
				if (strcmp(topics[i]->o_name, topic_name) == 0) {
					found_topic = topics[i];
					break;
				}
			}

			if (!found_topic) {
				PX4_ERR("Topic %s did not match any known topics", topic_name);
				return;
			}

			// print recursively
			for (int i = 0; i < array_size; ++i) {
				PX4_INFO_RAW("  %s", field_name);

				if (array_size > 1) {
					PX4_INFO_RAW("[%i]", i);
				}

				PX4_INFO_RAW(" (%s):\n", topic_name);
				orb_print_message_internal(found_topic, data_ptr + data_offset, false);
				data_offset += found_topic->o_size;
			}
		}

		format_idx = end_field_idx + 1;
	}
}
