@###############################################
@#
@# EmPy template for generating microRTPS_client.cpp file
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - msgs (List) list of all RTPS messages
@#  - multi_topics (List) list of all multi-topic names
@#  - spec (msggen.MsgSpec) Parsed specification of the .msg file
@###############################################
@{
import os

import genmsg.msgs

from px_generate_uorb_topic_files import MsgScope # this is in Tools/

topic_names = [s.short_name for s in spec]
send_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.SEND and not poll[idx]]
send_topics_poll = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.SEND and poll[idx]]
send_topics_poll_interval = [poll_interval[idx] for idx, s in enumerate(spec) if scope[idx] == MsgScope.SEND and poll[idx]]
send_base_types = [s.short_name for idx, s in enumerate(spec) if scope[idx] == MsgScope.SEND and not poll[idx]]
send_base_types_poll = [s.short_name for idx, s in enumerate(spec) if scope[idx] == MsgScope.SEND and poll[idx]]

recv_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.RECEIVE]
receive_base_types = [s.short_name for idx, s in enumerate(spec) if scope[idx] == MsgScope.RECEIVE]
}@
/****************************************************************************
 *
 * Copyright (c) 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 * Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <microRTPS_transport.h>
#include <microRTPS_client.h>

#include <inttypes.h>
#include <cstdio>
#include <ctime>
#include <pthread.h>

#include <ucdr/microcdr.h>
#include <px4_time.h>
#include <uORB/uORB.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
@[for topic in list(set(topic_names))]@
#include <uORB/topics/@(topic).h>
#include <uORB_microcdr/topics/@(topic).h>
@[end for]@

using namespace time_literals;

@[if recv_topics]@
// Publishers for received messages
struct RcvTopicsPubs {
@[    for idx, topic in enumerate(recv_topics)]@
	uORB::Publication <@(receive_base_types[idx])_s> @(topic)_pub{ORB_ID(@(topic))};
@[    end for]@
};
@[end if]@

@[if send_topics or send_topics_poll]@
@[    if send_topics]@
// Subscribers for messages to send
struct SendTopicsSubs {
@[    for idx, topic in enumerate(send_topics)]@
	uORB::Subscription @(topic)_sub{ORB_ID(@(topic))};
@[    end for]@
};
@[    end if]@

@[    if send_topics_poll]@
// Subscribers for messages to send by polling
struct SendPollTopicsSubs {
@[         for idx, topic in enumerate(send_topics_poll)]@
	int @(topic)_sub = orb_subscribe(ORB_ID(@(topic)));
@[         end for]@
};
@[    end if]@


struct SendThreadArgs {
	const uint32_t &datarate;
	uint64_t &total_sent;
	uint64_t &sent_last_sec;
	uint64_t &sent;
	int &sent_loop;
	SendThreadArgs(const uint32_t &datarate_, uint64_t &total_sent_,
                    uint64_t &sent_last_sec_, uint64_t &sent_, int &sent_loop_)
		: datarate(datarate_),
		  total_sent(total_sent_),
		  sent_last_sec(sent_last_sec_),
		  sent(sent_),
		  sent_loop(sent_loop_) {}
};

void *send(void *args)
{
	char data_buffer[BUFFER_SIZE]{};
	int read{0};
	uint32_t length{0};
	size_t header_length{0};
	uint8_t last_msg_seq{0};
	uint8_t last_remote_msg_seq{0};

	struct SendThreadArgs *data = reinterpret_cast<struct SendThreadArgs *>(args);
@[    if send_topics]@
	SendTopicsSubs *subs = new SendTopicsSubs();
@[    end if]@

@[    if send_topics_poll]@
	SendPollTopicsSubs *subs_poll = new SendPollTopicsSubs();
@[         for idx, topic in enumerate(send_topics_poll)]@
	if (subs_poll->@(topic)_sub < 0) {
		PX4_ERR("Failed to subscribe (%i)", errno);
	}
@[	        if send_topics_poll_interval[idx] > 0.0 ]@
	orb_set_interval(subs_poll->@(topic)_sub, @(send_topics_poll_interval[idx]));
@[	        end if ]@
@[         end for]@
@[    end if]@

	float bandwidth_mult{0};
	float tx_interval{1.f};
	uint64_t tx_last_sec_read{0};
	hrt_abstime last_stats_update{0};

	// ucdrBuffer to serialize using the user defined buffer
	ucdrBuffer writer;
	header_length = transport_node->get_header_length();
	ucdr_init_buffer(&writer, reinterpret_cast<uint8_t *>(&data_buffer[header_length]), BUFFER_SIZE - header_length);

	while (!_should_exit_task) {
@[    for idx, topic in enumerate(send_topics)]@
		{
			@(send_base_types[idx])_s @(topic)_data;

			if (subs->@(topic)_sub.update(&@(topic)_data))
			{
@[        if topic == 'Timesync' or topic == 'timesync']@
				if (@(topic)_data.seq != last_remote_msg_seq && @(topic)_data.tc1 == 0) {
					last_remote_msg_seq = @(topic)_data.seq;

					@(topic)_data.timestamp = hrt_absolute_time();
					@(topic)_data.seq = last_msg_seq;
					@(topic)_data.tc1 = hrt_absolute_time() * 1000ULL;
					@(topic)_data.ts1 = @(topic)_data.ts1;

					last_msg_seq++;
@[        end if]@
					// copy raw data into local buffer. Payload is shifted by header length to make room for header
					serialize_@(send_base_types[idx])(&writer, &@(topic)_data, &data_buffer[header_length], &length);

					if (0 < (read = transport_node->write(static_cast<char>(@(msgs[0].index(topic) + 1)), data_buffer, length))) {
						data->total_sent += read;
						tx_last_sec_read += read;
						++data->sent;
					}

@[        if topic == 'Timesync' or topic == 'timesync']@
				}

@[        end if]@
			}
		}
@[    end for]@

@[    if send_topics_poll]@
		{
			px4_pollfd_struct_t fds[@(len(send_topics_poll))];
@[        for idx, topic in enumerate(send_topics_poll)]@
			fds[@(idx)].fd = subs_poll->@(topic)_sub;
			fds[@(idx)].events = POLLIN;
@[        end for]@

			int pret = px4_poll(fds, @(len(send_topics_poll)), 20);

			if (pret < 0) {
				PX4_ERR("poll failed (%i)", pret);
				px4_usleep(tx_interval);
			} else if (pret != 0) {
@[        for idx, topic in enumerate(send_topics_poll)]@
				if (fds[@(idx)].revents & POLLIN) {
					@(send_base_types_poll[idx])_s @(topic)_data;
					orb_copy(ORB_ID(@(topic)), subs_poll->@(topic)_sub, &@(topic)_data);
					serialize_@(send_base_types_poll[idx])(&writer, &@(topic)_data, &data_buffer[header_length], &length);

					if (0 < (read = transport_node->write(static_cast<char>(@(msgs[0].index(topic) + 1)), data_buffer, length))) {
						data->total_sent += read;
						tx_last_sec_read += read;
						++data->sent;
					}
				}
@[        end for]@
			}
		}
@[    end if]@
		if (hrt_absolute_time() - last_stats_update >= 1_s) {
			data->sent_last_sec = tx_last_sec_read;
			if (data->datarate > 0) {
				bandwidth_mult = static_cast<float>(data->datarate) / static_cast<float>(tx_last_sec_read);
				// Apply a low-pass filter to determine the new TX interval
				tx_interval += 0.5f * (tx_interval / bandwidth_mult - tx_interval);
				// Clamp the interval between 1 and 1000 ms
				tx_interval = math::constrain(tx_interval, MIN_TX_INTERVAL_US, MAX_TX_INTERVAL_US);
			}
			tx_last_sec_read = 0;
			last_stats_update = hrt_absolute_time();
		}
@[	if not send_topics_poll]@
		px4_usleep(tx_interval);
@[	end if]@
		++data->sent_loop;
	}

	delete(data);
	delete(subs);

	return nullptr;
}

static int launch_send_thread(pthread_t &sender_thread, struct SendThreadArgs &args)
{
	pthread_attr_t sender_thread_attr;
	pthread_attr_init(&sender_thread_attr);
	pthread_attr_setstacksize(&sender_thread_attr, PX4_STACK_ADJUSTED(2250));
	struct sched_param param;
	(void)pthread_attr_getschedparam(&sender_thread_attr, &param);
	param.sched_priority = SCHED_PRIORITY_DEFAULT;
	(void)pthread_attr_setschedparam(&sender_thread_attr, &param);
	int rc = pthread_create(&sender_thread, &sender_thread_attr, &send, (void *)&args);
	if (rc != 0) {
		errno = rc;
		PX4_ERR("Could not create send thread (%d)", errno);
		return -1;
	}
	rc = pthread_setname_np(sender_thread, "urtpsclient_snd");
	if (pthread_setname_np(sender_thread, "urtpsclient_snd")) {
		errno = rc;
		PX4_ERR("Could not set pthread name for the send thread (%d)", errno);
	}
	pthread_attr_destroy(&sender_thread_attr);

	return 0;
}
@[end if]@

void micrortps_start_topics(const uint32_t &datarate, struct timespec &begin, uint64_t &total_rcvd,
			    uint64_t &total_sent, uint64_t &sent_last_sec,
			    uint64_t &rcvd_last_sec, uint64_t &received, uint64_t &sent, int &rcvd_loop, int &sent_loop)
{
	px4_clock_gettime(CLOCK_REALTIME, &begin);
	_should_exit_task = false;

@[if recv_topics]@
	char data_buffer[BUFFER_SIZE]{};
	int read{0};
	uint8_t topic_ID{255};

	uint64_t rx_last_sec_read{0};
	hrt_abstime last_stats_update{0};

	RcvTopicsPubs *pubs = new RcvTopicsPubs();

	// Set the main task name to 'urtpsclient_rcv' in case there is
	// data to receive
	px4_prctl(PR_SET_NAME, "urtpsclient_rcv", px4_getpid());

	// ucdrBuffer to deserialize using the user defined buffer
	ucdrBuffer reader;
	ucdr_init_buffer(&reader, reinterpret_cast<uint8_t *>(data_buffer), BUFFER_SIZE);
@[end if]@

@[if send_topics]@
	// var struct to be updated on the thread
	SendThreadArgs *sender_thread_args = new SendThreadArgs(datarate, total_sent, sent_last_sec, sent, sent_loop);

	// create a thread for sending data
	pthread_t sender_thread;
	launch_send_thread(sender_thread, (*sender_thread_args));
@[end if]@

	while (!_should_exit_task) {
@[if recv_topics]@
		while (0 < (read = transport_node->read(&topic_ID, data_buffer, BUFFER_SIZE))) {
			total_rcvd += read;
			rx_last_sec_read += read;

			uint64_t read_time = hrt_absolute_time();

			switch (topic_ID) {
@[    for idx, topic in enumerate(recv_topics)]@
			case @(msgs[0].index(topic) + 1): {
				@(receive_base_types[idx])_s @(topic)_data;
				deserialize_@(receive_base_types[idx])(&reader, &@(topic)_data, data_buffer);

				if (@(topic)_data.timestamp > read_time) {
					// don't allow timestamps from the future
					@(topic)_data.timestamp = read_time;
				}

				pubs->@(topic)_pub.publish(@(topic)_data);
				++received;
			}
			break;
@[    end for]@
			default:
				PX4_WARN("Unexpected topic ID '%hhu' to getMsg. Please make sure the client is capable of parsing the message associated to the topic ID '%hhu'",
					 topic_ID, topic_ID);
				break;
			}
		}
@[end if]@

		if (hrt_absolute_time() - last_stats_update >= 1_s) {
			rcvd_last_sec = rx_last_sec_read;
			rx_last_sec_read = 0;
			last_stats_update = hrt_absolute_time();
		}

		// loop forever if informed loop number is negative
		if (_options.loops >= 0 && rcvd_loop >= _options.loops) { break; }

		px4_usleep(_options.sleep_us);
		++rcvd_loop;
	}

@[if send_topics]@
	_should_exit_task = true;
	pthread_join(sender_thread, nullptr);
@[end if]@
@[if recv_topics]@
	delete(pubs);
@[end if]@
}
