/****************************************************************************
 *
 *   Copyright (c) 2022-2023 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <src/modules/uxrce_dds_client/dds_topics.h>

#include <uORB/topics/message_format_request.h>
#include <uORB/topics/message_format_response.h>
#include <uORB/Subscription.hpp>

#include <lib/timesync/Timesync.hpp>

#include <lib/perf/perf_counter.h>

#if defined(CONFIG_NET) || defined(__PX4_POSIX)
# define UXRCE_DDS_CLIENT_UDP 1
#endif

#include "srv_base.h"

#define MAX_NUM_REPLIERS 5
#define STREAM_HISTORY  4
#define BUFFER_SIZE (UXR_CONFIG_SERIAL_TRANSPORT_MTU * STREAM_HISTORY) // MTU==512 by default

class UxrceddsClient : public ModuleBase<UxrceddsClient>, public ModuleParams
{
public:
	enum class Transport {
		Serial,
		Udp
	};

	UxrceddsClient(Transport transport, const char *device, int baudrate, const char *host, const char *port,
		       const char *client_namespace);

	~UxrceddsClient();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static UxrceddsClient *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/**
	 * @brief Method to add a new replyer to the replier array.
	 * @param replier pointer to the new replier.
	 * @return Returns false iff successful, otherwise false.
	 */
	bool add_replier(SrvBase *replier);

	/**
	 * @brief Method to process new incoming requests and dispatch them to the appropriate server.
	 * @param object_id replier object id
	 * @param sample_id pointer to specific request.
	 * @param time_offset_us time offset between agent and client.
	 * @param ub pointer to the received request data
	 */
	void process_requests(uxrObjectId object_id, SampleIdentity *sample_id, ucdrBuffer *ub, const int64_t time_offset_us);

	/**
	 * @brief Method to process the available replies.
	 * @return Returns false iff successful, otherwise false.
	 */
	void process_replies();

	/**
	 * @brief Method to delete all repliers.
	 * @return Returns false iff successful, otherwise false.
	 */
	void delete_repliers();

private:

	bool init();
	void deinit();

	bool setup_session(uxrSession *session);
	void delete_session(uxrSession *session);

	bool setBaudrate(int fd, unsigned baud);

	void handleMessageFormatRequest();

	uORB::Publication<message_format_response_s> _message_format_response_pub{ORB_ID(message_format_response)};
	uORB::Subscription _message_format_request_sub{ORB_ID(message_format_request)};

	/** Synchronizes the system clock if the time is off by more than 5 seconds */
	void syncSystemClock(uxrSession *session);

	Transport _transport{};

	uxrSerialTransport *_transport_serial{nullptr};
	char _device[32] {};
	int _baudrate{};

	const char *_client_namespace;

	enum class ParticipantConfig {
		Default,
		LocalHostOnly,
		Custom,
	} _participant_config{ParticipantConfig::Default};

	bool _synchronize_timestamps;

	// max port characters (5+'\0')
	static const uint8_t PORT_MAX_LENGTH = 6;

	// max agent ip characters (15+'\0')
	static const uint8_t AGENT_IP_MAX_LENGTH = 16;

#if defined(UXRCE_DDS_CLIENT_UDP)
	char _port[PORT_MAX_LENGTH] {};
	char _agent_ip[AGENT_IP_MAX_LENGTH] {};
	uxrUDPTransport *_transport_udp{nullptr};
#endif // UXRCE_DDS_CLIENT_UDP

	SendTopicsSubs *_subs{nullptr};
	RcvTopicsPubs *_pubs{nullptr};

	uxrObjectId _participant_id;

	uint8_t _output_reliable_stream_buffer[BUFFER_SIZE] {};
	uint8_t _output_data_stream_buffer[2048] {};
	uint8_t _input_reliable_stream_buffer[BUFFER_SIZE] {};

	uxrStreamId _reliable_out;
	uxrStreamId _best_effort_out;

	SrvBase *_repliers[MAX_NUM_REPLIERS];
	uint8_t _num_of_repliers{0};

	uxrCommunication *_comm{nullptr};
	int _fd{-1};

	int _last_payload_tx_rate{}; ///< in B/s
	int _last_payload_rx_rate{}; ///< in B/s

	bool _connected{false};
	bool _session_created{false};
	bool _timesync_converged{false};
	bool _subs_initialized{false};

	Timesync _timesync{timesync_status_s::SOURCE_PROTOCOL_DDS};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::UXRCE_DDS_DOM_ID>) _param_uxrce_dds_dom_id,
		(ParamInt<px4::params::UXRCE_DDS_KEY>) _param_uxrce_key,
		(ParamInt<px4::params::UXRCE_DDS_PTCFG>) _param_uxrce_dds_ptcfg,
		(ParamInt<px4::params::UXRCE_DDS_SYNCC>) _param_uxrce_dds_syncc,
		(ParamInt<px4::params::UXRCE_DDS_SYNCT>) _param_uxrce_dds_synct
	)
};
