/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

#include <sys/types.h>
#include <sys/stat.h>
#include <stdarg.h>
#include <fcntl.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>

#if defined(__PX4_NUTTX) && !defined(CONFIG_BUILD_FLAT) && defined(__KERNEL__)
#include <px4_platform/board_ctrl.h>
#endif

#include "uORBDeviceNode.hpp"
#include "uORBUtils.hpp"
#include "uORBManager.hpp"

#ifdef CONFIG_ORB_COMMUNICATOR
pthread_mutex_t uORB::Manager::_communicator_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif

uORB::Manager *uORB::Manager::_Instance = nullptr;

bool uORB::Manager::initialize()
{
	if (_Instance == nullptr) {
		_Instance = new uORB::Manager();
	}

#if defined(__PX4_NUTTX) && !defined(CONFIG_BUILD_FLAT) && defined(__KERNEL__)
	px4_register_boardct_ioctl(_ORBIOCDEVBASE, orb_ioctl);
#endif
	return _Instance != nullptr;
}

bool uORB::Manager::terminate()
{
	if (_Instance != nullptr) {
		delete _Instance;
		_Instance = nullptr;
		return true;
	}

	return false;
}

uORB::Manager::Manager()
{
#ifdef ORB_USE_PUBLISHER_RULES
	const char *file_name = PX4_STORAGEDIR"/orb_publisher.rules";
	int ret = readPublisherRulesFromFile(file_name, _publisher_rule);

	if (ret == PX4_OK) {
		_has_publisher_rules = true;
		PX4_INFO("Using orb rules from %s", file_name);

	} else {
		PX4_ERR("Failed to read publisher rules file %s (%s)", file_name, strerror(-ret));
	}

#endif /* ORB_USE_PUBLISHER_RULES */

}

uORB::Manager::~Manager()
{
	delete _device_master;
}

uORB::DeviceMaster *uORB::Manager::get_device_master()
{
	if (!_device_master) {
		_device_master = new DeviceMaster();

		if (_device_master == nullptr) {
			PX4_ERR("Failed to allocate DeviceMaster");
			errno = ENOMEM;
		}
	}

	return _device_master;
}

#if defined(__PX4_NUTTX) && !defined(CONFIG_BUILD_FLAT) && defined(__KERNEL__)
int	uORB::Manager::orb_ioctl(unsigned int cmd, unsigned long arg)
{
	int ret = PX4_OK;

	switch (cmd) {
	case ORBIOCDEVEXISTS: {
			orbiocdevexists_t *data = (orbiocdevexists_t *)arg;

			if (data->check_advertised) {
				if (uORB::Manager::get_instance()) {
					data->ret = uORB::Manager::get_instance()->orb_exists(get_orb_meta(data->orb_id), data->instance);

				} else {
					data->ret = PX4_ERROR;
				}

			} else {
				data->ret = uORB::Manager::orb_device_node_exists(data->orb_id, data->instance) ? PX4_OK : PX4_ERROR;
			}
		}
		break;

	case ORBIOCDEVADVERTISE: {
			orbiocdevadvertise_t *data = (orbiocdevadvertise_t *)arg;
			uORB::DeviceMaster *dev = uORB::Manager::get_instance()->get_device_master();

			if (dev) {
				data->ret = dev->advertise(data->meta, data->is_advertiser, data->instance);

			} else {
				data->ret = PX4_ERROR;
			}
		}
		break;

	case ORBIOCDEVUNADVERTISE: {
			orbiocdevunadvertise_t *data = (orbiocdevunadvertise_t *)arg;
			data->ret = uORB::Manager::orb_unadvertise(data->handle);
		}
		break;

	case ORBIOCDEVPUBLISH: {
			orbiocdevpublish_t *data = (orbiocdevpublish_t *)arg;
			data->ret = uORB::Manager::orb_publish(data->meta, data->handle, data->data);
		}
		break;

	case ORBIOCDEVADDSUBSCRIBER: {
			orbiocdevaddsubscriber_t *data = (orbiocdevaddsubscriber_t *)arg;
			data->handle = uORB::Manager::orb_add_internal_subscriber(data->orb_id, data->instance, data->initial_generation);
		}
		break;

	case ORBIOCDEVREMSUBSCRIBER: {
			uORB::Manager::orb_remove_internal_subscriber(reinterpret_cast<void *>(arg));
		}
		break;

	case ORBIOCDEVQUEUESIZE: {
			orbiocdevqueuesize_t *data = (orbiocdevqueuesize_t *)arg;
			data->size = uORB::Manager::orb_get_queue_size(data->handle);
		}
		break;

	case ORBIOCDEVDATACOPY: {
			orbiocdevdatacopy_t *data = (orbiocdevdatacopy_t *)arg;
			data->ret = uORB::Manager::orb_data_copy(data->handle, data->dst, data->generation, data->only_if_updated);
		}
		break;

	case ORBIOCDEVREGCALLBACK: {
			orbiocdevregcallback_t *data = (orbiocdevregcallback_t *)arg;
			data->registered = uORB::Manager::register_callback(data->handle, data->callback_sub);
		}
		break;

	case ORBIOCDEVUNREGCALLBACK: {
			orbiocdevunregcallback_t *data = (orbiocdevunregcallback_t *)arg;
			uORB::Manager::unregister_callback(data->handle, data->callback_sub);
		}
		break;

	case ORBIOCDEVGETINSTANCE: {
			orbiocdevgetinstance_t *data = (orbiocdevgetinstance_t *)arg;
			data->instance = uORB::Manager::orb_get_instance(data->handle);
		}
		break;

	case ORBIOCDEVMASTERCMD: {
			uORB::DeviceMaster *dev = uORB::Manager::get_instance()->get_device_master();

			if (dev) {
				if (arg == ORB_DEVMASTER_TOP) {
					dev->showTop(nullptr, 0);

				} else {
					dev->printStatistics();
				}
			}
		}
		break;

	case ORBIOCDEVUPDATESAVAIL: {
			orbiocdevupdatesavail_t *data = (orbiocdevupdatesavail_t *)arg;
			data->ret = updates_available(data->handle, data->last_generation);
		}
		break;

	case ORBIOCDEVISADVERTISED: {
			orbiocdevisadvertised_t *data = (orbiocdevisadvertised_t *)arg;
			data->ret = is_advertised(data->handle);
		}
		break;

	default:
		ret = -ENOTTY;
	}

	return ret;
}
#endif

int uORB::Manager::orb_exists(const struct orb_metadata *meta, int instance)
{
	if (meta == nullptr) {
		return PX4_ERROR;
	}

	int ret = PX4_ERROR;

	// instance valid range: [0, ORB_MULTI_MAX_INSTANCES)
	if ((instance < 0) || (instance > (ORB_MULTI_MAX_INSTANCES - 1))) {
		return ret;
	}

	uORB::DeviceMaster *dev = uORB::Manager::get_instance()->get_device_master();

	if (dev) {
		uORB::DeviceNode *node = dev->getDeviceNode(meta, instance);

		if (node != nullptr) {
			if (node->is_advertised()) {
				return PX4_OK;
			}
		}
	}

	return ret;
}

orb_advert_t uORB::Manager::orb_advertise_multi(const struct orb_metadata *meta, const void *data, int *instance)
{
#ifdef ORB_USE_PUBLISHER_RULES

	// check publisher rule
	if (_has_publisher_rules) {
		const char *prog_name = px4_get_taskname();

		if (strcmp(_publisher_rule.module_name, prog_name) == 0) {
			if (_publisher_rule.ignore_other_topics) {
				if (!findTopic(_publisher_rule, meta->o_name)) {
					PX4_DEBUG("not allowing %s to publish topic %s", prog_name, meta->o_name);
					return (orb_advert_t)_Instance;
				}
			}

		} else {
			if (findTopic(_publisher_rule, meta->o_name)) {
				PX4_DEBUG("not allowing %s to publish topic %s", prog_name, meta->o_name);
				return (orb_advert_t)_Instance;
			}
		}
	}

#endif /* ORB_USE_PUBLISHER_RULES */

	/* open the node as an advertiser */
	int fd = node_open(meta, true, instance);

	if (fd == PX4_ERROR) {
		PX4_ERR("%s advertise failed (%i)", meta->o_name, errno);
		return nullptr;
	}

	/* get the advertiser handle and close the node */
	orb_advert_t advertiser;

	int result = px4_ioctl(fd, ORBIOCGADVERTISER, (unsigned long)&advertiser);
	px4_close(fd);

	if (result == PX4_ERROR) {
		PX4_WARN("px4_ioctl ORBIOCGADVERTISER failed. fd = %d", fd);
		return nullptr;
	}

#ifdef CONFIG_ORB_COMMUNICATOR

	// Advertise to the remote side, but only if it is a local topic. Otherwise
	// we will generate an advertisement loop.
	if (_remote_topics.find(meta->o_name) == false) {
		uORB::DeviceNode::topic_advertised(meta);
	}

#endif /* CONFIG_ORB_COMMUNICATOR */

	/* the advertiser may perform an initial publish to initialise the object */
	if (data != nullptr) {
		result = orb_publish(meta, advertiser, data);

		if (result == PX4_ERROR) {
			PX4_ERR("orb_publish failed %s", meta->o_name);
			return nullptr;
		}
	}

	return advertiser;
}

int uORB::Manager::orb_unadvertise(orb_advert_t handle)
{
#ifdef ORB_USE_PUBLISHER_RULES

	if (handle == _Instance) {
		return PX4_OK; //pretend success
	}

#endif /* ORB_USE_PUBLISHER_RULES */

	return uORB::DeviceNode::unadvertise(handle);
}

int uORB::Manager::orb_subscribe(const struct orb_metadata *meta)
{
	return node_open(meta, false);
}

int uORB::Manager::orb_subscribe_multi(const struct orb_metadata *meta, unsigned instance)
{
	int inst = instance;
	return node_open(meta, false, &inst);
}

int uORB::Manager::orb_unsubscribe(int fd)
{
	return px4_close(fd);
}

int uORB::Manager::orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data)
{
#ifdef ORB_USE_PUBLISHER_RULES

	if (handle == _Instance) {
		return PX4_OK; //pretend success
	}

#endif /* ORB_USE_PUBLISHER_RULES */

	return uORB::DeviceNode::publish(meta, handle, data);
}

int uORB::Manager::orb_copy(const struct orb_metadata *meta, int handle, void *buffer)
{
	int ret;

	ret = px4_read(handle, buffer, meta->o_size);

	if (ret < 0) {
		return PX4_ERROR;
	}

	if (ret != (int)meta->o_size) {
		errno = EIO;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int uORB::Manager::orb_check(int handle, bool *updated)
{
	/* Set to false here so that if `px4_ioctl` fails to false. */
	*updated = false;
	return px4_ioctl(handle, ORBIOCUPDATED, (unsigned long)(uintptr_t)updated);
}

int uORB::Manager::orb_set_interval(int handle, unsigned interval)
{
	return px4_ioctl(handle, ORBIOCSETINTERVAL, interval * 1000);
}

int uORB::Manager::orb_get_interval(int handle, unsigned *interval)
{
	int ret = px4_ioctl(handle, ORBIOCGETINTERVAL, (unsigned long)interval);
	*interval /= 1000;
	return ret;
}


bool uORB::Manager::orb_device_node_exists(ORB_ID orb_id, uint8_t instance)
{
	DeviceMaster *device_master = uORB::Manager::get_instance()->get_device_master();

	return device_master != nullptr &&
	       device_master->deviceNodeExists(orb_id, instance);
}

void *uORB::Manager::orb_add_internal_subscriber(ORB_ID orb_id, uint8_t instance, unsigned *initial_generation)
{
	uORB::DeviceNode *node = nullptr;
	DeviceMaster *device_master = uORB::Manager::get_instance()->get_device_master();

	if (device_master != nullptr) {
		node = device_master->getDeviceNode(get_orb_meta(orb_id), instance);

		if (node) {
			node->add_internal_subscriber();
			*initial_generation = node->get_initial_generation();
		}
	}

	return node;
}

void uORB::Manager::orb_remove_internal_subscriber(void *node_handle)
{
	static_cast<DeviceNode *>(node_handle)->remove_internal_subscriber();
}

uint8_t uORB::Manager::orb_get_queue_size(const void *node_handle) { return static_cast<const DeviceNode *>(node_handle)->get_queue_size(); }

int8_t uORB::Manager::orb_get_subscriber_count(const void *node_handle)
{
	return static_cast<const DeviceNode *>(node_handle)->subscriber_count();
}

bool uORB::Manager::orb_data_copy(void *node_handle, void *dst, unsigned &generation, bool only_if_updated)
{
	if (!is_advertised(node_handle)) {
		return false;
	}

	if (only_if_updated && !static_cast<const uORB::DeviceNode *>(node_handle)->updates_available(generation)) {
		return false;
	}

	return static_cast<DeviceNode *>(node_handle)->copy(dst, generation);
}

// add item to list of work items to schedule on node update
bool uORB::Manager::register_callback(void *node_handle, SubscriptionCallback *callback_sub)
{
	return static_cast<DeviceNode *>(node_handle)->register_callback(callback_sub);
}

// remove item from list of work items
void uORB::Manager::unregister_callback(void *node_handle, SubscriptionCallback *callback_sub)
{
	static_cast<DeviceNode *>(node_handle)->unregister_callback(callback_sub);
}

uint8_t uORB::Manager::orb_get_instance(const void *node_handle)
{
	if (node_handle) {
		return static_cast<const uORB::DeviceNode *>(node_handle)->get_instance();
	}

	return -1;
}

/* These are optimized by inlining in NuttX Flat build */
#if !defined(CONFIG_BUILD_FLAT)
unsigned uORB::Manager::updates_available(const void *node_handle, unsigned last_generation)
{
	return is_advertised(node_handle) ? static_cast<const uORB::DeviceNode *>(node_handle)->updates_available(
		       last_generation) : 0;
}

bool uORB::Manager::is_advertised(const void *node_handle)
{
	return static_cast<const uORB::DeviceNode *>(node_handle)->is_advertised();
}
#endif

int uORB::Manager::node_open(const struct orb_metadata *meta, bool advertiser, int *instance)
{
	char path[orb_maxpath];
	int fd = -1;
	int ret = PX4_ERROR;

	/*
	 * If meta is null, the object was not defined, i.e. it is not
	 * known to the system.  We can't advertise/subscribe such a thing.
	 */
	if (nullptr == meta) {
		errno = ENOENT;
		return PX4_ERROR;
	}

	/* if we have an instance and are an advertiser, we will generate a new node and set the instance,
	 * so we do not need to open here */
	if (!instance || !advertiser) {
		/*
		 * Generate the path to the node and try to open it.
		 */
		ret = uORB::Utils::node_mkpath(path, meta, instance);

		if (ret != OK) {
			errno = -ret;
			return PX4_ERROR;
		}

		/* open the path as either the advertiser or the subscriber */
		fd = px4_open(path, advertiser ? PX4_F_WRONLY : PX4_F_RDONLY);

	} else {
		*instance = 0;
	}

	/* we may need to advertise the node... */
	if (fd < 0) {

		ret = PX4_ERROR;

		if (get_device_master()) {
			ret = _device_master->advertise(meta, advertiser, instance);
		}

		/* it's OK if it already exists */
		if ((ret != PX4_OK) && (EEXIST == errno)) {
			ret = PX4_OK;
		}

		if (ret == PX4_OK) {
			/* update the path, as it might have been updated during the node advertise call */
			ret = uORB::Utils::node_mkpath(path, meta, instance);

			/* on success, try to open again */
			if (ret == PX4_OK) {
				fd = px4_open(path, (advertiser) ? PX4_F_WRONLY : PX4_F_RDONLY);

			} else {
				errno = -ret;
				return PX4_ERROR;
			}
		}
	}

	if (fd < 0) {
		errno = EIO;
		return PX4_ERROR;
	}

	/* everything has been OK, we can return the handle now */
	return fd;
}

#ifdef CONFIG_ORB_COMMUNICATOR
void uORB::Manager::set_uorb_communicator(uORBCommunicator::IChannel *channel)
{
	pthread_mutex_lock(&_communicator_mutex);

	if (channel != nullptr) {
		channel->register_handler(this);
		_comm_channel = channel;
	}

	pthread_mutex_unlock(&_communicator_mutex);
}

uORBCommunicator::IChannel *uORB::Manager::get_uorb_communicator()
{
	pthread_mutex_lock(&_communicator_mutex);
	uORBCommunicator::IChannel *temp = _comm_channel;
	pthread_mutex_unlock(&_communicator_mutex);

	return temp;
}

int16_t uORB::Manager::process_remote_topic(const char *topic_name)
{
	PX4_DEBUG("entering process_remote_topic: name: %s", topic_name);

	// First make sure this is a valid topic
	const struct orb_metadata *const *topic_list = orb_get_topics();
	orb_id_t topic_ptr = nullptr;

	for (size_t i = 0; i < orb_topics_count(); i++) {
		if (strcmp(topic_list[i]->o_name, topic_name) == 0) {
			topic_ptr = topic_list[i];
			break;
		}
	}

	if (! topic_ptr) {
		PX4_ERR("process_remote_topic meta not found for %s\n", topic_name);
		return -1;
	}

	// Look to see if we already have a node for this topic
	char nodepath[orb_maxpath];
	int ret = uORB::Utils::node_mkpath(nodepath, topic_name);

	if (ret == OK) {
		DeviceMaster *device_master = get_device_master();

		if (device_master) {
			uORB::DeviceNode *node = device_master->getDeviceNode(nodepath);

			if (node) {
				PX4_DEBUG("Marking DeviceNode(%s) as advertised in process_remote_topic", topic_name);
				node->mark_as_advertised();
				_remote_topics.insert(topic_name);
				return 0;
			}
		}
	}

	// We didn't find a node so we need to create it via an advertisement
	PX4_DEBUG("Advertising remote topic %s", topic_name);
	_remote_topics.insert(topic_name);
	orb_advertise(topic_ptr, nullptr);

	return 0;
}

int16_t uORB::Manager::process_add_subscription(const char *messageName)
{
	PX4_DEBUG("entering Manager_process_add_subscription: name: %s", messageName);

	int16_t rc = 0;
	char nodepath[orb_maxpath];
	int ret = uORB::Utils::node_mkpath(nodepath, messageName);
	DeviceMaster *device_master = get_device_master();

	if (ret == OK && device_master) {
		uORB::DeviceNode *node = device_master->getDeviceNode(nodepath);

		if (node == nullptr) {
			PX4_DEBUG("DeviceNode(%s) not created yet", messageName);

		} else {
			// node is present. But don't send any data to it if it
			// is a node advertised by the remote side
			if (_remote_topics.find(messageName) == false) {
				node->process_add_subscription();
			}
		}

	} else {
		rc = -1;
	}

	return rc;
}

int16_t uORB::Manager::process_remove_subscription(const char *messageName)
{
	int16_t rc = -1;
	char nodepath[orb_maxpath];
	int ret = uORB::Utils::node_mkpath(nodepath, messageName);
	DeviceMaster *device_master = get_device_master();

	if (ret == OK && device_master) {
		uORB::DeviceNode *node = device_master->getDeviceNode(nodepath);

		// get the node name.
		if (node == nullptr) {
			PX4_DEBUG("[posix-uORB::Manager::process_remove_subscription(%d)]Error No existing subscriber found for message: [%s]",
				  __LINE__, messageName);

		} else {
			// node is present.
			node->process_remove_subscription();
			rc = 0;
		}
	}

	return rc;
}

int16_t uORB::Manager::process_received_message(const char *messageName, int32_t length, uint8_t *data)
{
	int16_t rc = -1;
	char nodepath[orb_maxpath];
	int ret = uORB::Utils::node_mkpath(nodepath, messageName);
	DeviceMaster *device_master = get_device_master();

	if (ret == OK && device_master) {
		uORB::DeviceNode *node = device_master->getDeviceNode(nodepath);

		// get the node name.
		if (node == nullptr) {
			PX4_DEBUG("No existing subscriber found for message: [%s] nodepath:[%s]", messageName, nodepath);

		} else {
			// node is present.
			node->process_received_message(length, data);
			rc = 0;
		}
	}

	return rc;
}

#endif /* CONFIG_ORB_COMMUNICATOR */

#ifdef ORB_USE_PUBLISHER_RULES

bool uORB::Manager::startsWith(const char *pre, const char *str)
{
	size_t lenpre = strlen(pre),
	       lenstr = strlen(str);
	return lenstr < lenpre ? false : strncmp(pre, str, lenpre) == 0;
}

bool uORB::Manager::findTopic(const PublisherRule &rule, const char *topic_name)
{
	const char **topics_ptr = rule.topics;

	while (*topics_ptr) {
		if (strcmp(*topics_ptr, topic_name) == 0) {
			return true;
		}

		++topics_ptr;
	}

	return false;
}

void uORB::Manager::strTrim(const char **str)
{
	while (**str == ' ' || **str == '\t') { ++(*str); }
}

int uORB::Manager::readPublisherRulesFromFile(const char *file_name, PublisherRule &rule)
{
	FILE *fp;
	static const int line_len = 1024;
	int ret = PX4_OK;
	char *line = new char[line_len];

	if (!line) {
		return -ENOMEM;
	}

	fp = fopen(file_name, "r");

	if (fp == NULL) {
		delete[](line);
		return -errno;
	}

	const char *restrict_topics_str = "restrict_topics:";
	const char *module_str = "module:";
	const char *ignore_others = "ignore_others:";

	rule.ignore_other_topics = false;
	rule.module_name = nullptr;
	rule.topics = nullptr;

	while (fgets(line, line_len, fp) && ret == PX4_OK) {

		if (strlen(line) < 2 || line[0] == '#') {
			continue;
		}

		if (startsWith(restrict_topics_str, line)) {
			//read topics list
			char *start = line + strlen(restrict_topics_str);
			strTrim((const char **)&start);
			char *topics = strdup(start);
			int topic_len = 0, num_topics = 0;

			for (int i = 0; topics[i]; ++i) {
				if (topics[i] == ',' || topics[i] == '\n') {
					if (topic_len > 0) {
						topics[i] = 0;
						++num_topics;
					}

					topic_len = 0;

				} else {
					++topic_len;
				}
			}

			if (num_topics > 0) {
				rule.topics = new const char *[num_topics + 1];
				int topic = 0;
				strTrim((const char **)&topics);
				rule.topics[topic++] = topics;

				while (topic < num_topics) {
					if (*topics == 0) {
						++topics;
						strTrim((const char **)&topics);
						rule.topics[topic++] = topics;

					} else {
						++topics;
					}
				}

				rule.topics[num_topics] = nullptr;
			}

		} else if (startsWith(module_str, line)) {
			//read module name
			char *start = line + strlen(module_str);
			strTrim((const char **)&start);
			int len = strlen(start);

			if (len > 0 && start[len - 1] == '\n') {
				start[len - 1] = 0;
			}

			rule.module_name = strdup(start);

		} else if (startsWith(ignore_others, line)) {
			const char *start = line + strlen(ignore_others);
			strTrim(&start);

			if (startsWith("true", start)) {
				rule.ignore_other_topics = true;
			}

		} else {
			PX4_ERR("orb rules file: wrong format: %s", line);
			ret = -EINVAL;
		}
	}

	if (ret == PX4_OK && (!rule.module_name || !rule.topics)) {
		PX4_ERR("Wrong format in orb publisher rules file");
		ret = -EINVAL;
	}

	delete[](line);
	fclose(fp);
	return ret;
}
#endif /* ORB_USE_PUBLISHER_RULES */
