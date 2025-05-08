/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include "translations.h"
#include "util.h"
#include "template_util.h"

#include <rosidl_typesupport_cpp/message_type_support_dispatch.hpp>
#include <rosidl_typesupport_fastrtps_cpp/message_type_support.h>

class RegisteredTranslations {
public:

	RegisteredTranslations(RegisteredTranslations const&) = delete;
	void operator=(RegisteredTranslations const&) = delete;


	static RegisteredTranslations& instance() {
		static RegisteredTranslations instance;
		return instance;
	}

	/**
	 * @brief Register a translation class with 1 input and 1 output message.
	 *
	 * The translation class has the form:
	 *
	 * ```
	 * class MyTranslation {
	 * public:
	 * 	using MessageOlder = px4_msgs_old::msg::VehicleAttitudeV2;
	 *
	 * 	using MessageNewer = px4_msgs::msg::VehicleAttitude;
	 *
	 * 	static constexpr const char* kTopic = "fmu/out/vehicle_attitude";
	 *
	 * 	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer) {
	 * 	    // set msg_newer from msg_older
	 * 	}
	 *
	 * 	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older) {
	 * 	    // set msg_older from msg_newer
	 * 	}
	 * };
	 * ```
	 */
	template<class T>
	void registerDirectTranslation() {
		const std::string topic_name = T::kTopic;
		_topic_translations.addTopic(getTopicForMessageType<typename T::MessageOlder>(topic_name));
		_topic_translations.addTopic(getTopicForMessageType<typename T::MessageNewer>(topic_name));

		// Translation callbacks
		auto translation_cb_from_older = [](const std::vector<MessageBuffer>& older_msg, std::vector<MessageBuffer>& newer_msg) {
			T::fromOlder(*(const typename T::MessageOlder*)older_msg[0].get(), *(typename T::MessageNewer*)newer_msg[0].get());
		};
		auto translation_cb_to_older = [](const std::vector<MessageBuffer>& newer_msg, std::vector<MessageBuffer>& older_msg) {
			T::toOlder(*(const typename T::MessageNewer*)newer_msg[0].get(), *(typename T::MessageOlder*)older_msg[0].get());
		};
		_topic_translations.addTranslation({translation_cb_from_older,
									  {MessageIdentifier{topic_name, T::MessageOlder::MESSAGE_VERSION}},
									  {MessageIdentifier{topic_name, T::MessageNewer::MESSAGE_VERSION}}});
		_topic_translations.addTranslation({translation_cb_to_older,
									  {MessageIdentifier{topic_name, T::MessageNewer::MESSAGE_VERSION}},
									  {MessageIdentifier{topic_name, T::MessageOlder::MESSAGE_VERSION}}});
	}

	/**
	 * @brief Register a translation class for a service.
	 *
	 * The translation class has the form:
	 *
	 * ```
	 * class MyServiceTranslation {
	 * public:
	 * 	using MessageOlder = px4_msgs_old::srv::VehicleCommandV0;
	 * 	using MessageNewer = px4_msgs::srv::VehicleCommand;
	 *
	 * 	static constexpr const char* kTopic = "fmu/vehicle_command";
	 *
	 * 	static void fromOlder(const MessageOlder::Request &msg_older, MessageNewer::Request &msg_newer) {
	 * 		// set msg_newer from msg_older
	 * 	}
	 *
	 * 	static void toOlder(const MessageNewer::Request &msg_newer, MessageOlder::Request &msg_older) {
	 * 		// set msg_older from msg_newer
	 * 	}
	 *
	 * 	static void fromOlder(const MessageOlder::Response &msg_older, MessageNewer::Response &msg_newer) {
	 * 		// set msg_newer from msg_older
	 * 	}
	 *
	 * 	static void toOlder(const MessageNewer::Response &msg_newer, MessageOlder::Response &msg_older) {
	 * 		// set msg_older from msg_newer
	 * 	}
	 * };
	 * ```
	 */
	template<class T>
	void registerServiceDirectTranslation() {
		const std::string topic_name = T::kTopic;
		_service_translations.addNode(getServiceForMessageType<typename T::MessageOlder>(topic_name));
		_service_translations.addNode(getServiceForMessageType<typename T::MessageNewer>(topic_name));
		// Add translations
		{ // Request
			auto translation_cb_from_older = [](const std::vector<MessageBuffer> &older_msg,
												std::vector<MessageBuffer> &newer_msg) {
				T::fromOlder(*(const typename T::MessageOlder::Request *) older_msg[0].get(),
							 *(typename T::MessageNewer::Request *) newer_msg[0].get());
			};
			auto translation_cb_to_older = [](const std::vector<MessageBuffer> &newer_msg,
											  std::vector<MessageBuffer> &older_msg) {
				T::toOlder(*(const typename T::MessageNewer::Request *) newer_msg[0].get(),
						   *(typename T::MessageOlder::Request *) older_msg[0].get());
			};
			_service_translations.addRequestTranslation({translation_cb_from_older,
														 {MessageIdentifier{topic_name, T::MessageOlder::Request::MESSAGE_VERSION}},
														 {MessageIdentifier{topic_name, T::MessageNewer::Request::MESSAGE_VERSION}}});
			_service_translations.addRequestTranslation({translation_cb_to_older,
														 {MessageIdentifier{topic_name, T::MessageNewer::Request::MESSAGE_VERSION}},
														 {MessageIdentifier{topic_name, T::MessageOlder::Request::MESSAGE_VERSION}}});
		}
		{ // Response
			auto translation_cb_from_older = [](const std::vector<MessageBuffer> &older_msg,
												std::vector<MessageBuffer> &newer_msg) {
				T::fromOlder(*(const typename T::MessageOlder::Response *) older_msg[0].get(),
							 *(typename T::MessageNewer::Response *) newer_msg[0].get());
			};
			auto translation_cb_to_older = [](const std::vector<MessageBuffer> &newer_msg,
											  std::vector<MessageBuffer> &older_msg) {
				T::toOlder(*(const typename T::MessageNewer::Response *) newer_msg[0].get(),
						   *(typename T::MessageOlder::Response *) older_msg[0].get());
			};
			_service_translations.addResponseTranslation({translation_cb_from_older,
														 {MessageIdentifier{topic_name, T::MessageOlder::Request::MESSAGE_VERSION}},
														 {MessageIdentifier{topic_name, T::MessageNewer::Request::MESSAGE_VERSION}}});
			_service_translations.addResponseTranslation({translation_cb_to_older,
														 {MessageIdentifier{topic_name, T::MessageNewer::Request::MESSAGE_VERSION}},
														 {MessageIdentifier{topic_name, T::MessageOlder::Request::MESSAGE_VERSION}}});
		}

	}

	/**
	 * @brief Register a translation class with N input and M output messages.
	 *
	 * The translation class has the form:
	 * ```
	 * class MyTranslation {
	 * public:
	 * 	using MessagesOlder = TypesArray<ROS_MSG_OLDER_1, ROS_MSG_OLDER_2, ...>;
	 * 	static constexpr const char* kTopicsOlder[] = {
	 * 			"fmu/out/vehicle_global_position",
	 * 			"fmu/out/vehicle_local_position",
	 * 			...
	 * 	};
	 *
	 * 	using MessagesNewer = TypesArray<ROS_MSG_NEWER_1, ROS_MSG_NEWER_2, ...>;
	 * 	static constexpr const char* kTopicsNewer[] = {
	 * 			"fmu/out/vehicle_global_position",
	 * 			"fmu/out/vehicle_local_position",
	 * 			...
	 * 	};
	 *
	 * 	static void fromOlder(const MessagesOlder::Type1 &msg_older1, const MessagesOlder::Type2 &msg_older2, ...
	 * 						  MessagesNewer::Type1 &msg_newer1, MessagesNewer::Type2 &msg_newer2, ...) {
	 * 		// Set msg_newerX from msg_olderX
	 * 	}
	 *
	 * 	static void toOlder(const MessagesNewer::Type1 &msg_newer1, const MessagesNewer::Type2 &msg_newer2, ...
	 * 						MessagesOlder::Type1 &msg_older1, MessagesOlder::Type2 &msg_older2, ...) {
	 * 		// Set msg_olderX from msg_newerX
	 * 	}
	 * };
	 * ```
	 */
	template<class T>
	void registerTranslation() {
		const auto topics_older = getTopicsForMessageType(typename T::MessagesOlder::args(), T::kTopicsOlder);
		std::vector<MessageIdentifier> topics_older_identifiers;
		for (const auto& topic : topics_older) {
			_topic_translations.addTopic(topic);
			topics_older_identifiers.emplace_back(topic.id);
		}
		const auto topics_newer = getTopicsForMessageType(typename T::MessagesNewer::args(),T::kTopicsNewer);
		std::vector<MessageIdentifier> topics_newer_identifiers;
		for (const auto& topic : topics_newer) {
			_topic_translations.addTopic(topic);
			topics_newer_identifiers.emplace_back(topic.id);
		}

		// Translation callbacks
		const auto translation_cb_from_older = [](const std::vector<MessageBuffer>& older_msgs, std::vector<MessageBuffer>& newer_msgs) {
			call_translation_function(&T::fromOlder, typename T::MessagesOlder::args(), typename T::MessagesNewer::args(), older_msgs, newer_msgs);
		};
		const auto translation_cb_to_older = [](const std::vector<MessageBuffer>& newer_msgs, std::vector<MessageBuffer>& older_msgs) {
			call_translation_function(&T::toOlder, typename T::MessagesNewer::args(), typename T::MessagesOlder::args(), newer_msgs, older_msgs);
		};
		{
			// Older -> Newer
			Translation translation;
			translation.cb = translation_cb_from_older;
			translation.inputs = topics_older_identifiers;
			translation.outputs = topics_newer_identifiers;
			_topic_translations.addTranslation(std::move(translation));
		}
		{
			// Newer -> Older
			Translation translation;
			translation.cb = translation_cb_to_older;
			translation.inputs = topics_newer_identifiers;
			translation.outputs = topics_older_identifiers;
			_topic_translations.addTranslation(std::move(translation));
		}
	}

	const TopicTranslations& topicTranslations() const { return _topic_translations; }
	const ServiceTranslations& serviceTranslations() const { return _service_translations; }

protected:
	RegisteredTranslations() = default;
private:
	template<typename RosMessageType>
	static size_t getMaxSerializedMessageSize() {
		const auto type_handle = rclcpp::get_message_type_support_handle<RosMessageType>();
		const auto fastrtps_handle = rosidl_typesupport_cpp::get_message_typesupport_handle_function(&type_handle, "rosidl_typesupport_fastrtps_cpp");
		if (fastrtps_handle) {
			const auto *callbacks = static_cast<const message_type_support_callbacks_t *>(fastrtps_handle->data);
			char bound_info;
			return callbacks->max_serialized_size(bound_info);
		}
		return 0;
	}

	template<typename RosMessageType>
	static Topic getTopicForMessageType(const std::string& topic_name) {
		Topic ret{};
		ret.id.topic_name = topic_name;
		ret.id.version = RosMessageType::MESSAGE_VERSION;
		auto message_buffer = std::make_shared<RosMessageType>();
		ret.message_buffer = std::static_pointer_cast<void>(message_buffer);

		// Subscription/Publication factory methods
		const std::string topic_name_versioned = getVersionedTopicName(topic_name, ret.id.version);
		ret.subscription_factory = [topic_name_versioned, message_buffer](rclcpp::Node& node,
																		  const std::function<void()>& on_topic_cb) -> rclcpp::SubscriptionBase::SharedPtr {
			return std::dynamic_pointer_cast<rclcpp::SubscriptionBase>(
					// Note: template instantiation of subscriptions slows down compilation considerably, see
					// https://github.com/ros2/rclcpp/issues/1949
					node.create_subscription<RosMessageType>(topic_name_versioned, rclcpp::QoS(1).best_effort(),
															 [on_topic_cb=on_topic_cb, message_buffer](typename RosMessageType::UniquePtr msg) -> void {
																 *message_buffer = *msg;
																 on_topic_cb();
															 }));
		};
		ret.publication_factory = [topic_name_versioned](rclcpp::Node& node) -> rclcpp::PublisherBase::SharedPtr {
			return std::dynamic_pointer_cast<rclcpp::PublisherBase>(
					node.create_publisher<RosMessageType>(topic_name_versioned, rclcpp::QoS(1).best_effort()));
		};

		ret.max_serialized_message_size = getMaxSerializedMessageSize<RosMessageType>();

		return ret;
	}

	template<typename RosMessageType>
	static Service getServiceForMessageType(const std::string& topic_name) {
		Service ret{};
		ret.id.topic_name = topic_name;
		ret.id.version = RosMessageType::Request::MESSAGE_VERSION;
		auto message_buffer_request = std::make_shared<typename RosMessageType::Request>();
		ret.message_buffer_request = std::static_pointer_cast<void>(message_buffer_request);
		auto message_buffer_response = std::make_shared<typename RosMessageType::Response>();
		ret.message_buffer_response = std::static_pointer_cast<void>(message_buffer_response);

		// Service/client factory methods
		const std::string topic_name_versioned = getVersionedTopicName(topic_name, ret.id.version);
		ret.service_factory = [topic_name_versioned, message_buffer_request](rclcpp::Node& node,
																	 const std::function<void(std::shared_ptr<rmw_request_id_t> req_id)>& on_request_cb) -> rclcpp::ServiceBase::SharedPtr {
			return std::dynamic_pointer_cast<rclcpp::ServiceBase>(
					node.create_service<RosMessageType>(topic_name_versioned,
															 [on_request_cb=on_request_cb, message_buffer_request](
																	   typename rclcpp::Service<RosMessageType>::SharedPtr service,
																	   std::shared_ptr<rmw_request_id_t> req_id,
																	   const std::shared_ptr<typename RosMessageType::Request> request
															 ) -> void {
																 *message_buffer_request = *request;
																 on_request_cb(std::move(req_id));
															 }));
		};
		ret.client_factory = [topic_name_versioned, message_buffer_response](rclcpp::Node& node,
													const std::function<void(rmw_request_id_t&)>& on_response_cb) {
			auto client = node.create_client<RosMessageType>(topic_name_versioned);
			client->set_on_new_response_callback([client, message_buffer_response, on_response_cb](size_t num) {
				for (size_t i = 0; i < num; i++) {
					rmw_request_id_t request_id{};
					if (client->take_response(*message_buffer_response, request_id)) {
						on_response_cb(request_id);
					}
				}
			});
			const auto send_request = [client](MessageBuffer request) {
				auto result = client->async_send_request(std::static_pointer_cast<typename RosMessageType::Request>(request));
				// We don't need the client to keep track of ongoing requests, so we remove it right away
				// to prevent leaks
				client->remove_pending_request(result.request_id);
				return result.request_id;
			};
			return std::make_tuple(std::dynamic_pointer_cast<rclcpp::ClientBase>(client), send_request);
		};

		ret.publication_factory_request = [](rclcpp::Node& node, const std::string& topic_name) -> rclcpp::PublisherBase::SharedPtr {
			return std::dynamic_pointer_cast<rclcpp::PublisherBase>(
					node.create_publisher<typename RosMessageType::Request>(
							topic_name,rclcpp::QoS(1).best_effort().avoid_ros_namespace_conventions(true)));
		};
		ret.publication_factory_response = [](rclcpp::Node& node, const std::string& topic_name) -> rclcpp::PublisherBase::SharedPtr {
			return std::dynamic_pointer_cast<rclcpp::PublisherBase>(
					node.create_publisher<typename RosMessageType::Response>(
							topic_name,rclcpp::QoS(1).best_effort().avoid_ros_namespace_conventions(true)));
		};

		ret.max_serialized_message_size_request = getMaxSerializedMessageSize<typename RosMessageType::Request>();
		ret.max_serialized_message_size_response = getMaxSerializedMessageSize<typename RosMessageType::Response>();

		return ret;
	}

	template<typename... RosMessageTypes, size_t... Is>
	static std::vector<Topic> getTopicsForMessageTypeImpl(const char* const topics[], std::integer_sequence<size_t, Is...>) {
		std::vector<Topic> ret {
			getTopicForMessageType<RosMessageTypes>(topics[Is])...
		};
		return ret;
	}

	template<typename... RosMessageTypes, size_t N>
	static std::vector<Topic> getTopicsForMessageType(Pack<RosMessageTypes...>, const char* const (&topics)[N]) {
		static_assert(N == sizeof...(RosMessageTypes), "Number of topics does not match number of message types");
		return getTopicsForMessageTypeImpl<RosMessageTypes...>(topics, std::index_sequence_for<RosMessageTypes...>{});
	}

	TopicTranslations _topic_translations;
	ServiceTranslations _service_translations;
};

template<class T>
class RegistrationHelperDirect {
public:
	explicit RegistrationHelperDirect(const char* dummy) {
		// There's something strange: when there is no argument passed, the
		// compiler removes the static object completely. I don't know
		// why but this dummy variable prevents that.
		(void)dummy;
		RegisteredTranslations::instance().registerDirectTranslation<T>();
	}
	explicit RegistrationHelperDirect(const char* dummy, bool for_service) {
		(void)dummy;
		RegisteredTranslations::instance().registerServiceDirectTranslation<T>();
	}
	RegistrationHelperDirect(RegistrationHelperDirect const&) = delete;
	void operator=(RegistrationHelperDirect const&) = delete;
};

#define REGISTER_TOPIC_TRANSLATION_DIRECT(class_name) \
	RegistrationHelperDirect<class_name> class_name##_registration_direct("dummy");

#define REGISTER_SERVICE_TRANSLATION_DIRECT(class_name) \
	RegistrationHelperDirect<class_name> class_name##_service_registration_direct("dummy", true);

template<class T>
class TopicRegistrationHelperGeneric {
public:
	explicit TopicRegistrationHelperGeneric(const char* dummy) {
		(void)dummy;
		RegisteredTranslations::instance().registerTranslation<T>();
	}
	TopicRegistrationHelperGeneric(TopicRegistrationHelperGeneric const&) = delete;
	void operator=(TopicRegistrationHelperGeneric const&) = delete;
};

#define REGISTER_TOPIC_TRANSLATION(class_name) \
	TopicRegistrationHelperGeneric<class_name> class_name##_registration_generic("dummy");
