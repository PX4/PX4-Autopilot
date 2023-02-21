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

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
@[for include in type_includes]@
#include <uORB/ucdr/@(include).h>
@[end for]@

// Subscribers for messages to send
struct SendTopicsSubs {
@[    for pub in publications]@
	uORB::Subscription @(pub['topic_simple'])_sub{ORB_ID(@(pub['topic_simple']))};
	uxrObjectId @(pub['topic_simple'])_data_writer{};
@[    end for]@

	uint32_t num_payload_sent{};

	void update(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId best_effort_stream_id, uxrObjectId participant_id, const char *client_namespace);
	void reset();
};

void SendTopicsSubs::reset() {
	num_payload_sent = 0;
@[    for idx, pub in enumerate(publications)]@
	@(pub['topic_simple'])_data_writer = uxr_object_id(0, UXR_INVALID_ID);
@[    end for]@
};

void SendTopicsSubs::update(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId best_effort_stream_id, uxrObjectId participant_id, const char *client_namespace)
{
	int64_t time_offset_us = session->time_offset / 1000; // ns -> us
@[    for idx, pub in enumerate(publications)]@

	{
		@(pub['simple_base_type'])_s data;

		if (@(pub['topic_simple'])_sub.update(&data)) {

			if (@(pub['topic_simple'])_data_writer.id == UXR_INVALID_ID) {
				// data writer not created yet
				create_data_writer(session, reliable_out_stream_id, participant_id, ORB_ID::@(pub['topic_simple']), client_namespace, "@(pub['topic_simple'])", "@(pub['dds_type'])", @(pub['topic_simple'])_data_writer);
			}

			if (@(pub['topic_simple'])_data_writer.id != UXR_INVALID_ID) {

				ucdrBuffer ub;
				uint32_t topic_size = ucdr_topic_size_@(pub['simple_base_type'])();
				if (uxr_prepare_output_stream(session, best_effort_stream_id, @(pub['topic_simple'])_data_writer, &ub, topic_size) != UXR_INVALID_REQUEST_ID) {
					ucdr_serialize_@(pub['simple_base_type'])(data, ub, time_offset_us);
					// TODO: fill up the MTU and then flush, which reduces the packet overhead
					uxr_flash_output_streams(session);
					num_payload_sent += topic_size;

				} else {
					//PX4_ERR("Error uxr_prepare_output_stream UXR_INVALID_REQUEST_ID @(pub['topic_simple'])");
				}

			} else {
				//PX4_ERR("Error UXR_INVALID_ID @(pub['topic_simple'])");
			}

		}
	}
@[    end for]@
}

// Publishers for received messages
struct RcvTopicsPubs {
@[    for sub in subscriptions]@
	uORB::Publication<@(sub['simple_base_type'])_s> @(sub['topic_simple'])_pub{ORB_ID(@(sub['topic_simple']))};
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
@[    for idx, sub in enumerate(subscriptions)]@
	case @(idx)+1000: {
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
@[    for idx, sub in enumerate(subscriptions)]@
	{
			uint16_t queue_depth = uORB::DefaultQueueSize<@(sub['simple_base_type'])_s>::value * 2; // use a bit larger queue size than internal
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, @(idx), client_namespace, "@(sub['topic_simple'])", "@(sub['dds_type'])", queue_depth);
	}
@[    end for]@

	uxr_set_topic_callback(session, on_topic_update, this);

	return true;
}
