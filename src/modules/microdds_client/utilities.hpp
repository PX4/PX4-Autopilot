
#pragma once

#include <uxr/client/client.h>
#include <ucdr/microcdr.h>

#include <uORB/topics/uORBTopics.hpp>

#define TOPIC_NAME_SIZE 128

uxrObjectId topic_id_from_orb(ORB_ID orb_id, uint8_t instance = 0)
{
	if (orb_id != ORB_ID::INVALID) {
		uint16_t id = static_cast<uint8_t>(orb_id) + (instance * UINT8_MAX);
		uxrObjectId topic_id = uxr_object_id(id, UXR_TOPIC_ID);
		return topic_id;
	}

	return uxrObjectId{};
}

static bool generate_topic_name(char *topic, const char *client_namespace, const char *direction, const char *name)
{
	if (client_namespace != nullptr) {
		int ret = snprintf(topic, TOPIC_NAME_SIZE, "rt/%s/fmu/%s/%s", client_namespace, direction, name);
		return (ret > 0 && ret < TOPIC_NAME_SIZE);
	}

	int ret = snprintf(topic, TOPIC_NAME_SIZE, "rt/fmu/%s/%s", direction, name);
	return (ret > 0 && ret < TOPIC_NAME_SIZE);
}

static bool create_data_writer(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrObjectId participant_id,
			       ORB_ID orb_id, const char *client_namespace, const char *topic_name_simple, const char *type_name,
			       uxrObjectId &datawriter_id)
{
	// topic
	char topic_name[TOPIC_NAME_SIZE];

	if (!generate_topic_name(topic_name, client_namespace, "out", topic_name_simple)) {
		PX4_ERR("topic path too long");
		return false;
	}

	uxrObjectId topic_id = topic_id_from_orb(orb_id);
	uint16_t topic_req = uxr_buffer_create_topic_bin(session, reliable_out_stream_id, topic_id, participant_id, topic_name,
			     type_name, UXR_REPLACE);


	// publisher
	uxrObjectId publisher_id = uxr_object_id(topic_id.id, UXR_PUBLISHER_ID);
	uint16_t publisher_req = uxr_buffer_create_publisher_bin(session, reliable_out_stream_id, publisher_id, participant_id,
				 UXR_REPLACE);


	// data writer
	datawriter_id = uxr_object_id(topic_id.id, UXR_DATAWRITER_ID);

	uxrQoS_t qos = {
		.durability = UXR_DURABILITY_TRANSIENT_LOCAL,
		.reliability = UXR_RELIABILITY_BEST_EFFORT,
		.history = UXR_HISTORY_KEEP_LAST,
		.depth = 0,
	};

	uint16_t datawriter_req = uxr_buffer_create_datawriter_bin(session, reliable_out_stream_id, datawriter_id, publisher_id,
				  topic_id, qos, UXR_REPLACE);

	// Send create entities message and wait its status
	uint16_t requests[3] {topic_req, publisher_req, datawriter_req};
	uint8_t status[3];

	if (!uxr_run_session_until_all_status(session, 1000, requests, status, 3)) {
		PX4_ERR("create entities failed: %s, topic: %i publisher: %i datawriter: %i",
			topic_name, status[0], status[1], status[2]);
		return false;

	} else {
		PX4_INFO("successfully created %s data writer, topic id: %d", topic_name, topic_id.id);
	}

	return true;
}

static bool create_data_reader(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId input_stream_id,
			       uxrObjectId participant_id, uint16_t index, const char *client_namespace, const char *topic_name_simple,
			       const char *type_name, uint16_t queue_depth)
{
	// topic
	char topic_name[TOPIC_NAME_SIZE];

	if (!generate_topic_name(topic_name, client_namespace, "in", topic_name_simple)) {
		PX4_ERR("topic path too long");
		return false;
	}

	uint16_t id = index + 1000;


	uxrObjectId topic_id = uxr_object_id(id, UXR_TOPIC_ID);
	uint16_t topic_req = uxr_buffer_create_topic_bin(session, reliable_out_stream_id, topic_id, participant_id, topic_name,
			     type_name, UXR_REPLACE);


	// subscriber
	uxrObjectId subscriber_id = uxr_object_id(id, UXR_SUBSCRIBER_ID);
	uint16_t subscriber_req = uxr_buffer_create_subscriber_bin(session, reliable_out_stream_id, subscriber_id,
				  participant_id, UXR_REPLACE);


	// data reader
	uxrObjectId datareader_id = uxr_object_id(id, UXR_DATAREADER_ID);

	uxrQoS_t qos = {
		.durability = UXR_DURABILITY_VOLATILE,
		.reliability = UXR_RELIABILITY_BEST_EFFORT,
		.history = UXR_HISTORY_KEEP_LAST,
		.depth = queue_depth,
	};

	uint16_t datareader_req = uxr_buffer_create_datareader_bin(session, reliable_out_stream_id, datareader_id,
				  subscriber_id, topic_id, qos, UXR_REPLACE);

	uint16_t requests[3] {topic_req, subscriber_req, datareader_req};
	uint8_t status[3];

	if (!uxr_run_session_until_all_status(session, 1000, requests, status, 3)) {
		PX4_ERR("create entities failed: %s %i %i %i", topic_name,
			status[0], status[1], status[2]);
		return false;
	}

	uxrDeliveryControl delivery_control{};
	delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
	uxr_buffer_request_data(session, reliable_out_stream_id, datareader_id, input_stream_id, &delivery_control);

	return true;
}
