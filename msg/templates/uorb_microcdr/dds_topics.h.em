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

import genmsg.msgs

from px_generate_uorb_topic_files import MsgScope # this is in Tools/

topic_names = [s.short_name for s in spec]
send_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.SEND]
send_base_types = [s.short_name for idx, s in enumerate(spec) if scope[idx] == MsgScope.SEND]
recv_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.RECEIVE]
receive_base_types = [s.short_name for idx, s in enumerate(spec) if scope[idx] == MsgScope.RECEIVE]
}@


#include <uxr/client/client.h>
#include <ucdr/microcdr.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
@[for idx, topic in enumerate(send_topics)]@
#include <uORB/ucdr/@(send_base_types[idx]).h>
@[end for]@
@[for idx, topic in enumerate(recv_topics)]@
#include <uORB/ucdr/@(receive_base_types[idx]).h>
@[end for]@

// Subscribers for messages to send
struct SendTopicsSubs {
@[    for idx, topic in enumerate(send_topics)]@
	uORB::Subscription @(topic)_sub{ORB_ID(@(topic))};
	uxrObjectId @(topic)_data_writer;
@[    end for]@

	uxrSession* session;

	uint32_t num_payload_sent{};

	bool init(uxrSession* session_, uxrStreamId stream_id, uxrObjectId participant_id);
	void update(uxrStreamId stream_id);
};

bool SendTopicsSubs::init(uxrSession* session_, uxrStreamId stream_id, uxrObjectId participant_id)
{
	session = session_;

@[    for idx, topic in enumerate(send_topics)]@
@{
topic_pascal = topic.replace("_", " ").title().replace(" ", "")
}@
	{

		uxrObjectId topic_id = uxr_object_id(@(idx)+1, UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/out/@(topic_pascal)</name>"
				"<dataType>px4_msgs::msg::dds_::@(topic_pascal)_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml,
						UXR_REPLACE);

		uxrObjectId publisher_id = uxr_object_id(@(idx)+1, UXR_PUBLISHER_ID);
		const char* publisher_xml = "";
		uint16_t publisher_req = uxr_buffer_create_publisher_xml(session, stream_id, publisher_id, participant_id,
						publisher_xml, UXR_REPLACE);

		uxrObjectId datawriter_id = uxr_object_id(@(idx)+1, UXR_DATAWRITER_ID);
		@(topic)_data_writer = datawriter_id;
		const char* datawriter_xml = "<dds>"
				"<data_writer>"
				"<topic>"
				"<kind>NO_KEY</kind>"
				"<name>rt/fmu/out/@(topic_pascal)</name>"
				"<dataType>px4_msgs::msg::dds_::@(topic_pascal)_</dataType>"
				"</topic>"
				"</data_writer>"
				"</dds>";
		uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(session, stream_id, datawriter_id, publisher_id,
						datawriter_xml, UXR_REPLACE);

		// Send create entities message and wait its status
		uint8_t status[3];
		uint16_t requests[3] = {
			topic_req, publisher_req, datawriter_req
		};
		if (!uxr_run_session_until_all_status(session, 1000, requests, status, 3)) {
			PX4_ERR("create entities failed: %s, topic: %i publisher: %i datawriter: %i", "@(topic_pascal)", status[0], status[1], status[2]);
			return false;
		}
	}

@[    end for]@

	return true;
}

void SendTopicsSubs::update(uxrStreamId stream_id)
{
@[    for idx, topic in enumerate(send_topics)]@
	{
		@(send_base_types[idx])_s data;

		if (@(topic)_sub.update(&data)) {
			ucdrBuffer ub{};
			uint32_t topic_size = ucdr_topic_size_@(send_base_types[idx])();
			uxr_prepare_output_stream(session, stream_id, @(topic)_data_writer, &ub, topic_size);
			ucdr_serialize_@(send_base_types[idx])(data, ub);
			// TODO: fill up the MTU and then flush, which reduces the packet overhead
			uxr_flash_output_streams(session);
			num_payload_sent += topic_size;
		}
	}
@[    end for]@

}

static void on_topic_update(uxrSession* session, uxrObjectId object_id,
	uint16_t request_id, uxrStreamId stream_id, struct ucdrBuffer* ub, uint16_t
	length, void* args);

// Publishers for received messages
struct RcvTopicsPubs {
@[    for idx, topic in enumerate(recv_topics)]@
	uORB::Publication <@(receive_base_types[idx])_s> @(topic)_pub{ORB_ID(@(topic))};
@[    end for]@

	uxrSession* session;

	uint32_t num_payload_received{};

	bool init(uxrSession* session_, uxrStreamId stream_id, uxrStreamId input_stream, uxrObjectId participant_id);
};

bool RcvTopicsPubs::init(uxrSession* session_, uxrStreamId stream_id, uxrStreamId input_stream, uxrObjectId participant_id)
{
	session = session_;
    uxr_set_topic_callback(session, on_topic_update, this);


@[    for idx, topic in enumerate(recv_topics)]@
@{
topic_pascal = topic.replace("_", " ").title().replace(" ", "")
}@
	{

		uxrObjectId subscriber_id = uxr_object_id(@(idx)+1, UXR_SUBSCRIBER_ID);
		const char* subscriber_xml = "";
		uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, stream_id, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

		uxrObjectId topic_id = uxr_object_id(1000+@(idx), UXR_TOPIC_ID);
		const char* topic_xml = "<dds>"
				"<topic>"
				"<name>rt/fmu/in/@(topic_pascal)</name>"
				"<dataType>px4_msgs::msg::dds_::@(topic_pascal)_</dataType>"
				"</topic>"
				"</dds>";
		uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml, UXR_REPLACE);

		uxrObjectId datareader_id = uxr_object_id(@(idx)+1, UXR_DATAREADER_ID);
		const char* datareader_xml = "<dds>"
										 "<data_reader>"
											 "<topic>"
												 "<kind>NO_KEY</kind>"
												 "<name>rt/fmu/in/@(topic_pascal)</name>"
												 "<dataType>px4_msgs::msg::dds_::@(topic_pascal)_</dataType>"
											 "</topic>"
										 "</data_reader>"
									 "</dds>";
		uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, stream_id, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

		uint8_t status[3];
		uint16_t requests[3] = {topic_req, subscriber_req, datareader_req };
		if(!uxr_run_session_until_all_status(session, 1000, requests, status, 3))
		{
			PX4_ERR("create entities failed: %s %i %i %i", "@(topic)", status[0], status[1], status[2]);
			return false;
		}

		uxrDeliveryControl delivery_control = {0};
		delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
		uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

	}

@[    end for]@

	return true;
}

void on_topic_update(uxrSession* session, uxrObjectId object_id,
	uint16_t request_id, uxrStreamId stream_id, struct ucdrBuffer* ub, uint16_t
	length, void* args)
{
	RcvTopicsPubs* pubs = (RcvTopicsPubs*)args;
	pubs->num_payload_received += length;

	switch (object_id.id) {
@[    for idx, topic in enumerate(recv_topics)]@
		case @(idx)+1: {
			@(receive_base_types[idx])_s topic;
			if (ucdr_deserialize_@(receive_base_types[idx])(*ub, topic)) {
				pubs->@(topic)_pub.publish(topic);
			}
		}
		break;
@[    end for]@

		default:
		PX4_ERR("unknown object id: %i", object_id.id);
		break;
	}
}
