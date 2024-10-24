
#pragma once

#include <uxr/client/client.h>
#include <ucdr/microcdr.h>

#include <uORB/topics/uORBTopics.hpp>

#define TOPIC_NAME_SIZE 128

uxrObjectId topic_id_from_orb(ORB_ID orb_id, uint8_t instance = 0)
{
	// Note that the uxrObjectId.id is a uint16_t so we need to cap the ID,
	// and urx does not allow us to use the upper 4 bits.
	const unsigned max_id = 65535U / 32U;
	const unsigned id = static_cast<unsigned>(orb_id) + (instance * ORB_TOPICS_COUNT);

	if (orb_id != ORB_ID::INVALID && id < max_id) {
		uxrObjectId topic_id = uxr_object_id(static_cast<uint16_t>(id), UXR_TOPIC_ID);
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
	char datawriter_xml[1024];
	datawriter_id = uxr_object_id(topic_id.id, UXR_DATAWRITER_ID);
	const char *datawriter_xml_format = "<dds>"
					    "<data_writer>"
					    "<topic>"
					    "<kind>NO_KEY</kind>"
					    "<name>%s</name>"
					    "<dataType>px4_msgs::msg::dds_::%s_</dataType>"
					    "</topic>"
					    "<qos>"
					    "<publishMode>"
					    "<kind>ASYNCHRONOUS</kind>"
					    "</publishMode>"
					    "</qos>"
					    "<historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>"
					    "</data_writer>"
					    "</dds>";

	int ret = snprintf(datawriter_xml, 1024, datawriter_xml_format, topic_name, topic_name_simple);

	if (ret < 0) {
		PX4_ERR("Can't create datawriter_xml string");
		return false;
	}

	uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(session, reliable_out_stream_id, datawriter_id, publisher_id,
				  datawriter_xml, UXR_REPLACE);

	// Send create entities message and wait its status
	uint16_t requests[3] {topic_req, publisher_req, datawriter_req};
	uint8_t status[3];

	if (!uxr_run_session_until_all_status(session, 5000, requests, status, 3)) {
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

	// Use the second half of the available ID space.
	// Add 1 so that we get a nice hex starting number: 0x800 instead of 0x7ff.
	uint16_t id = index + (65535U / 32U) + 1;

	uxrObjectId topic_id = uxr_object_id(id, UXR_TOPIC_ID);
	uint16_t topic_req = uxr_buffer_create_topic_bin(session, reliable_out_stream_id, topic_id, participant_id, topic_name,
			     type_name, UXR_REPLACE);


	// subscriber
	uxrObjectId subscriber_id = uxr_object_id(id, UXR_SUBSCRIBER_ID);
	uint16_t subscriber_req = uxr_buffer_create_subscriber_bin(session, reliable_out_stream_id, subscriber_id,
				  participant_id, UXR_REPLACE);


	// data reader
	char datareader_xml[1024];
	uxrObjectId datareader_id = uxr_object_id(id, UXR_DATAREADER_ID);
	const char *datareader_xml_format = "<dds>"
					    "<data_reader>"
					    "<topic>"
					    "<kind>NO_KEY</kind>"
					    "<name>%s</name>"
					    "<dataType>px4_msgs::msg::dds_::%s_</dataType>"
					    "</topic>"
					    "<historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>"
					    "</data_reader>"
					    "</dds>";

	int ret = snprintf(datareader_xml, 1024, datareader_xml_format, topic_name, topic_name_simple);

	if (ret < 0) {
		PX4_ERR("Can't create datareader_xml string");
		return false;
	}

	uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, reliable_out_stream_id, datareader_id,
				  subscriber_id, datareader_xml, UXR_REPLACE);

	uint16_t requests[3] {topic_req, subscriber_req, datareader_req};
	uint8_t status[3];

	if (!uxr_run_session_until_all_status(session, 5000, requests, status, 3)) {
		PX4_ERR("create entities failed: %s %i %i %i", topic_name,
			status[0], status[1], status[2]);
		return false;
	}

	uxrDeliveryControl delivery_control{};
	delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
	uxr_buffer_request_data(session, reliable_out_stream_id, datareader_id, input_stream_id, &delivery_control);

	return true;
}
