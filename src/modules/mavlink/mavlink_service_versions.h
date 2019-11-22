/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file mavlink_service_versions.h
 * Compile-time constants for supported MAVLink microservices.
 *
 * This file does not keep track of the status of any given microservice version handshake. Different instances of
 * MAVLink can be using different versions of a microservice, so that is done inside of the Mavlink class.
 *
 * See Mavlink::determine_service_version(...)
 *
 * @author Timothy Scott <timothy@auterion.com>
 */

#pragma once

#include <stdint.h>

namespace microservice_versions
{

// NUM_SERVICES should be one greater than the actual number of supported services, because index 0 contains
// dummy metadata for any unrecognized service.
constexpr size_t NUM_SERVICES = 4;
// service ID to represent an unknown service
constexpr uint16_t MAVLINK_SERVICE_UNKNOWN = 0;

/// This struct represents the information about microservice versions that is known to PX4 at compile time.
/// Specifically, it contains the minimum and maximum versions of a service supported by PX4, but NOT
/// the currently-selected version, which is negotiated at runtime, and is separate for each MAVLink instance.
struct service_metadata {
	const uint16_t service_id;  ///< The ID of the service
	/// The minimum version of the microservice supported by this version of PX4, or 0 if this microservice
	/// is not supported at all.
	const uint16_t min_version;
	/// The minimum version of the microservice supported by this version of PX4, or 0 if this microservice
	/// is not supported at all.
	const uint16_t max_version;
};

/**
 * This enum represents the current status of the microservice version handshake for one particular microservice.
 */
enum handshake_status {
	UNKNOWN,        ///< The microservice version handshake has not been done for this microservice
	UNSUPPORTED,    ///< The handshake has been done, and there is no common version, so this service is unsupported.
	SELECTED        ///< The handshake has been done, and the supported version has been selected.
};

/**
 * This struct represents the version information for one microservice, as known as runtime..
 */
struct service_status {
	/// See service_metadata. This is a pointer because there only needs to be one instance of a given
	/// metadata struct, because it is immutable.
	const service_metadata *metadata;
	/// See service_version_status
	handshake_status status;
	/// If status == SELECTED, then selected_version is the version of the microservice that is currently being used.
	/// If status != SELECTED, then this variable is undefined and should not be used to make any decisions.
	uint16_t selected_version;
};

/// Contains the actual data about what versions of what services are supported.
/// When adding support for a new microservice, or a new version of an existing microservice, you should
/// change the definition of this array in `mavlink_service_versions.cpp`.
/// TODO microservice version: Determine if there is a better way to declare this. If I define it in this header file,
///   it can potentially take up more flash space, as it will be redefined for every file in which it is included.
extern const service_metadata services_metadata[NUM_SERVICES];

//	/// Returns the metadata of a given microservice.
//	const service_metadata *get_metadata(uint16_t service_id);
}