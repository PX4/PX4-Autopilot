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
#include <limits.h>
#include <math.h>
#include "mavlink_main.h"
#include "mavlink_stream.h"

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

/**
 * This class manages the microservice versioning handshake.
 *
 * It inherits from MavlinkStream, so that it can support the "stream all services" request, without overloading
 * the MAVLink connection with too many messages all at once. Normally, it is idle, and streams no messages. However,
 * when the `request_service_version(...)` function is called, it becomes active, and sends one
 * `MAVLINK_SERVICE_VERSION` message per cycle.
 *
 * Once it has sent either the one service requested, or all services (if all were requested), it becomes idle again.
 */
class MavlinkServiceVersions : public MavlinkStream
{
public:
	const char *get_name() const override;

	static const char *get_name_static();

	static uint16_t get_id_static();

	uint16_t get_id() override;

	static MavlinkStream *new_instance(Mavlink *mavlink);

	unsigned get_size() override;

	/**
	 * This function returns the status of the given service, without selecting a version to use.
	 * @param service_id ID of the service in question
	 * @return Reference to a service_status which exists for the lifetime of this Mavlink object
	 */
	service_status &get_service_status(uint16_t service_id);

	/**
	 * Takes the min and max version supported by the communication partner, and determines the maximum version
	 * supported by both this version of PX4 and the communication partner. This selected version is then
	 * stored in this instance of Mavlink, and can be retrieved with Mavlink::get_service_status.
	 *
	 * @param service_id ID of the service being negotiated
	 * @param min_version Minimum version of the service supported by the other system.
	 * @param max_version Maximum version of the service supported by the other system.
	 * @return Reference to a service_status, which exists for the lifetime of this Mavlink object, and has been
	 * 			modified with the results of this service version handshake.
	 */
	service_status &determine_service_version(uint16_t service_id, uint16_t min_version,
			uint16_t max_version);

	/**
	 * See MavlinkStreamServiceVersion::determine_service_version(uint16_t, uint16_t, uint16_t).
	 *
	 * This overloaded function performs the same task as the other determine_service_version, but it does not know the
	 * min and max version supported by the other system, so it assumes that it supports every version (like a
	 * GCS would), and just selects the maximum version supported by this version of PX4.
	 *
	 * @param service_id ID of the service
	 * @return Reference to a service_status, which exists for the lifetime of this Mavlink object, and has been
	 * 			modified with the results of this service version handshake.
	 */
	service_status &determine_service_version(uint16_t service_id);

	/**
	 * This function should be called when we receive a MAV_CMD_REQUEST_SERVICE_VERSION. It performs the whole
	 * microservice version handshake: It chooses an appropriate version, and responds with a MAVLINK_SERVICE_VERSION
	 * message. After this is done, you can use MavlinkServiceVersions::get_service_status to determine the result
	 * of the handshake.
	 *
	 * @param service_id ID of the service to negotiate
	 * @param min_version Minimum supported version, sent from the other system
	 * @param max_version Maximum supported version, sent from the other system
	 */
	void request_serice_version(uint16_t service_id, uint16_t min_version, uint16_t max_version);

protected:
	explicit MavlinkServiceVersions(Mavlink *mavlink);

	~MavlinkServiceVersions();

	bool send(const hrt_abstime t) override;

	// TODO microservice versioning This should probably be removed before merging. But I am keeping it here for now
	//  just in case...
	//  (In a previous iteration of microservice versioning, it just used the generic `MAV_CMD_REQUEST_MSG`, but now
	//   it doesn't)
//	void request_message(float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0,
//						 float param5 = 0.0, float param6 = 0.0, float param7 = 0.0) override
//	{
//		request_serice_version((uint16_t) roundf(param2), (uint16_t) roundf(param3), (uint16_t) roundf(param4));
//	}

private:

	uint16_t _current_service = 0;
	uint16_t _min_version = 0;
	uint16_t _max_version = 0;
	bool _sending_all_services = false;
	int _default_interval = -1;

	service_status _microservice_versions[NUM_SERVICES];

	/* do not allow top copying this class */
	MavlinkServiceVersions(MavlinkServiceVersions &) = delete;

	MavlinkServiceVersions &operator=(const MavlinkServiceVersions &) = delete;
};
}