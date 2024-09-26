@###############################################
@#
@# EmPy template
@#
@###############################################
@# Generates publications and subscriptions for XRCE
@#
@# Context:
@#  - msgs (List) list of all RTPS messages
@#  - topics (List) list of all topic names
@#  - spec (msggen.MsgSpec) Parsed specification of the .msg file
@###############################################
@{
import os


}@

#include <utilities.hpp>

#include <uxr/client/client.h>
#include <ucdr/microcdr.h>

#include <mathlib/mathlib.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/uORB.h>
@[for include in type_includes]@
#include <uORB/ucdr/@(include).h>
#include <uORB/topics/@(include).h>
@[end for]@

#define UXRCE_DEFAULT_POLL_RATE 10

typedef bool (*UcdrSerializeMethod)(const void* data, ucdrBuffer& buf, int64_t time_offset);

static constexpr int max_topic_size = 512;
@[    for pub in publications]@
static_assert(sizeof(@(pub['simple_base_type'])_s) <= max_topic_size, "topic too large, increase max_topic_size");
@[    end for]@

struct SendSubscription {
	const struct orb_metadata *orb_meta;
	uxrObjectId data_writer;
	const char* dds_type_name;
	const char* topic;
	uint32_t topic_size;
	UcdrSerializeMethod ucdr_serialize_method;
};

// Subscribers for messages to send
struct SendTopicsSubs {
	SendSubscription send_subscriptions[@(len(publications))] = {
@[    for pub in publications]@
			{ ORB_ID(@(pub['topic_simple'])),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "@(pub['dds_type'])",
			  "@(pub['topic'])",
			  ucdr_topic_size_@(pub['simple_base_type'])(),
			  &ucdr_serialize_@(pub['simple_base_type']),
			},
@[    end for]@
	};

	px4_pollfd_struct_t fds[@(len(publications))] {};

	uint32_t num_payload_sent{};

	void init();
	void update(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId best_effort_stream_id, uxrObjectId participant_id, const char *client_namespace);
	void reset();
};

void SendTopicsSubs::init() {
	for (unsigned idx = 0; idx < sizeof(send_subscriptions)/sizeof(send_subscriptions[0]); ++idx) {
		fds[idx].fd = orb_subscribe(send_subscriptions[idx].orb_meta);
		fds[idx].events = POLLIN;
		orb_set_interval(fds[idx].fd, UXRCE_DEFAULT_POLL_RATE);
	}
}

void SendTopicsSubs::reset() {
	num_payload_sent = 0;
	for (unsigned idx = 0; idx < sizeof(send_subscriptions)/sizeof(send_subscriptions[0]); ++idx) {
		send_subscriptions[idx].data_writer = uxr_object_id(0, UXR_INVALID_ID);
		orb_unsubscribe(fds[idx].fd);
		fds[idx].fd = -1;
	}
};

void SendTopicsSubs::update(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId best_effort_stream_id, uxrObjectId participant_id, const char *client_namespace)
{
	int64_t time_offset_us = session->time_offset / 1000; // ns -> us

	alignas(sizeof(uint64_t)) char topic_data[max_topic_size];

	for (unsigned idx = 0; idx < sizeof(send_subscriptions)/sizeof(send_subscriptions[0]); ++idx) {
		if (fds[idx].revents & POLLIN) {
			// Topic updated, copy data and send
			orb_copy(send_subscriptions[idx].orb_meta, fds[idx].fd, &topic_data);
			if (send_subscriptions[idx].data_writer.id == UXR_INVALID_ID) {
				// data writer not created yet
				create_data_writer(session, reliable_out_stream_id, participant_id, static_cast<ORB_ID>(send_subscriptions[idx].orb_meta->o_id), client_namespace, send_subscriptions[idx].topic,
								   send_subscriptions[idx].dds_type_name, send_subscriptions[idx].data_writer);
			}

			if (send_subscriptions[idx].data_writer.id != UXR_INVALID_ID) {

				ucdrBuffer ub;
				uint32_t topic_size = send_subscriptions[idx].topic_size;
				if (uxr_prepare_output_stream(session, best_effort_stream_id, send_subscriptions[idx].data_writer, &ub, topic_size) != UXR_INVALID_REQUEST_ID) {
					send_subscriptions[idx].ucdr_serialize_method(&topic_data, ub, time_offset_us);
					// TODO: fill up the MTU and then flush, which reduces the packet overhead
					uxr_flash_output_streams(session);
					num_payload_sent += topic_size;

				} else {
					//PX4_ERR("Error uxr_prepare_output_stream UXR_INVALID_REQUEST_ID %s", send_subscriptions[idx].subscription.get_topic()->o_name);
				}

			} else {
				//PX4_ERR("Error UXR_INVALID_ID %s", send_subscriptions[idx].subscription.get_topic()->o_name);
			}

		}
	}
}

// Publishers for received messages
struct RcvTopicsPubs {
@[    for sub in subscriptions]@
	uORB::Publication<@(sub['simple_base_type'])_s> @(sub['topic_simple'])_pub{ORB_ID(@(sub['topic_simple']))};
@[    end for]@

@[    for sub in subscriptions_multi]@
	uORB::PublicationMulti<@(sub['simple_base_type'])_s> @(sub['topic_simple'])_pub{ORB_ID(@(sub['topic_simple']))};
@[    end for]@

	uint32_t num_payload_received{};

	bool init(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId reliable_in_stream_id, uxrStreamId best_effort_in_stream_id, uxrObjectId participant_id, const char *client_namespace);
};

static void on_topic_update(uxrSession *session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id,
		     struct ucdrBuffer *ub, uint16_t length, void *args)
{
	RcvTopicsPubs *pubs = (RcvTopicsPubs *)args;
	const int64_t time_offset_us = session->time_offset / 1000; // ns -> us
	pubs->num_payload_received += length;

	switch (object_id.id) {
@[    for idx, sub in enumerate(subscriptions + subscriptions_multi)]@
	case @(idx)+ (65535U / 32U) + 1: {
			@(sub['simple_base_type'])_s data;

			if (ucdr_deserialize_@(sub['simple_base_type'])(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(@(sub['simple_base_type'])), data);
				pubs->@(sub['topic_simple'])_pub.publish(data);
			}
		}
		break;

@[    end for]@

	default:
		PX4_ERR("unknown object id: %i", object_id.id);
		break;
	}
}

bool RcvTopicsPubs::init(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId reliable_in_stream_id, uxrStreamId best_effort_in_stream_id, uxrObjectId participant_id, const char *client_namespace)
{
@[    for idx, sub in enumerate(subscriptions + subscriptions_multi)]@
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(@(sub['simple_base_type']))) * 2; // use a bit larger queue size than internal
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, @(idx), client_namespace, "@(sub['topic'])", "@(sub['dds_type'])", queue_depth);
	}
@[    end for]@

	uxr_set_topic_callback(session, on_topic_update, this);

	return true;
}
