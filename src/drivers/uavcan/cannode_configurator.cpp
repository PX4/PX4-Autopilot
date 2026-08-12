/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file cannode_configurator.cpp
 *
 * Applies airframe-style parameter defaults to UAVCAN peripheral nodes.
 * Config files live at /etc/init.d/cannode_airframes/<node_name> in ROMFS.
 * Format: PARAM_NAME=value per line; type is inferred via a GetSet GET before each SET.
 * Lines starting with # are comments.
 */

#include "cannode_configurator.hpp"
#include "uavcan_main.hpp"

#include <cstdio>
#include <dirent.h>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <unistd.h>

#include <px4_platform_common/log.h>

using namespace time_literals;

static constexpr char CANNODE_AIRFRAMES_DIR[] = "/fs/microsd/ext_autostart/cannode_airframes/";

// Scans CANNODE_AIRFRAMES_DIR for a file whose @name tag matches node_name.
static bool findConfigForNode(const char *node_name, char *out_path, size_t path_size)
{
	DIR *dir = opendir(CANNODE_AIRFRAMES_DIR);

	if (!dir) { return false; }

	bool found = false;
	struct dirent *de;

	while (!found && (de = readdir(dir)) != nullptr) {
		if (de->d_name[0] == '.') { continue; }

		const int n = snprintf(out_path, path_size, "%s%s", CANNODE_AIRFRAMES_DIR, de->d_name);

		if (n <= 0 || static_cast<size_t>(n) >= path_size) { continue; }

		FILE *fp = fopen(out_path, "r");

		if (!fp) { continue; }

		char line[128];

		while (fgets(line, sizeof(line), fp) != nullptr) {
			line[strcspn(line, "\r\n")] = '\0';
			const char *p = line;

			while (*p == ' ' || *p == '\t') { ++p; }

			if (strncmp(p, "@name ", 6) == 0) {
				found = (strcmp(p + 6, node_name) == 0);
				break;
			}
		}

		fclose(fp);
	}

	closedir(dir);
	return found;
}

CanNodeConfigurator::CanNodeConfigurator(uavcan::NodeInfoRetriever &retriever)
	: _retriever(retriever)
{
	pthread_mutex_init(&_queue_mutex, nullptr);
	_retriever.addListener(this);
}

void CanNodeConfigurator::init()
{
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setstacksize(&attr, 4096);
	pthread_create(&_thread, &attr, threadEntry, this);
	pthread_attr_destroy(&attr);
}

CanNodeConfigurator::~CanNodeConfigurator()
{
	_retriever.removeListener(this);
	_should_exit.store(true);
	pthread_join(_thread, nullptr);
	_queue.clear();
	pthread_mutex_destroy(&_queue_mutex);
}

void CanNodeConfigurator::handleNodeInfoRetrieved(uavcan::NodeID node_id,
		const uavcan::protocol::GetNodeInfo_::Response &node_info)
{
	using NS = uavcan::protocol::NodeStatus;

	PX4_INFO("cannode_cfg: node %u '%s' mode=%u health=%u",
		 node_id.get(), node_info.name.c_str(),
		 (unsigned)node_info.status.mode,
		 (unsigned)node_info.status.health);

	if (node_info.status.mode != NS::MODE_OPERATIONAL ||
	    node_info.status.health >= NS::HEALTH_ERROR) {
		pthread_mutex_lock(&_queue_mutex);

		for (QueueEntry *e = _queue.getHead(); e != nullptr; e = e->getSibling()) {
			if (e->node_id == node_id.get()) {
				_queue.deleteNode(e);
				PX4_DEBUG("cannode_cfg: dequeued node %u (mode=%u health=%u)",
					  node_id.get(), (unsigned)node_info.status.mode,
					  (unsigned)node_info.status.health);
				break;
			}
		}

		pthread_mutex_unlock(&_queue_mutex);
		return;
	}

	QueueEntry *entry = new QueueEntry();

	if (entry == nullptr) {
		PX4_ERR("cannode_cfg: alloc failed for node %u", node_id.get());
		return;
	}

	entry->node_id   = node_id.get();
	entry->queued_at = hrt_absolute_time();
	strncpy(entry->node_name, node_info.name.c_str(), MaxNodeNameLen - 1);
	entry->node_name[MaxNodeNameLen - 1] = '\0';

	pthread_mutex_lock(&_queue_mutex);

	for (QueueEntry *e = _queue.getHead(); e != nullptr; e = e->getSibling()) {
		if (e->node_id == node_id.get()) {
			pthread_mutex_unlock(&_queue_mutex);
			delete entry;
			return;
		}
	}

	_queue.add(entry);
	pthread_mutex_unlock(&_queue_mutex);
}

void *CanNodeConfigurator::threadEntry(void *arg)
{
	static_cast<CanNodeConfigurator *>(arg)->threadMain();
	return nullptr;
}

void CanNodeConfigurator::threadMain()
{
	while (!_should_exit.load()) {
		uavcan_firmware_update_s fw_update{};

		if (_fw_update_sub.copy(&fw_update)) {
			_fw_update_pending = fw_update.pending_updates;
		}

		if (_fw_update_pending) {
			sleep(1);
			continue;
		}

		pthread_mutex_lock(&_queue_mutex);
		QueueEntry *entry = _queue.getHead();
		if (entry != nullptr) {
			if(hrt_elapsed_time(&entry->queued_at) < 3_s){
				entry = nullptr;
			}else{
				_queue.remove(entry);
			}
		}
		pthread_mutex_unlock(&_queue_mutex);

		if (entry != nullptr) {
			char path[MaxPathLen];

			if (findConfigForNode(entry->node_name, path, sizeof(path))) {
				applyConfig(entry->node_id, path);
			}

			delete entry;
		} else {
			sleep(3);
		}
	}
}

// Reads from fp, skipping blank lines and comments.
// Strips trailing \r\n. Returns false on EOF.
static bool readNextLine(FILE *fp, char *line, size_t len)
{
	while (fgets(line, static_cast<int>(len), fp) != nullptr) {
		line[strcspn(line, "\r\n")] = '\0';

		if (line[0] != '\0' && line[0] != '#') {
			return true;
		}
	}

	return false;
}

// Parses a "NAME=value" line. Returns false if malformed.
static bool parseNameValue(const char *line, char *out_name, size_t name_size, char *out_value, size_t value_size)
{
	const char *eq = strchr(line, '=');

	if (!eq) { return false; }

	const size_t name_len = static_cast<size_t>(eq - line);

	if (name_len == 0 || name_len >= name_size) { return false; }

	const size_t value_len = strlen(eq + 1);

	if (value_len == 0 || value_len >= value_size) { return false; }

	memcpy(out_name, line, name_len);
	out_name[name_len] = '\0';
	memcpy(out_value, eq + 1, value_len + 1);
	return true;
}

void CanNodeConfigurator::applyConfig(uint8_t node_id, const char *path)
{
	FILE *fp = fopen(path, "r");

	if (!fp) {
		PX4_ERR("cannode_cfg: cannot open '%s': %d", path, errno);
		return;
	}

	PX4_INFO("cannode_cfg: configuring node %u from '%s'", node_id, path);

	static constexpr size_t kNameSize  = 93;
	static constexpr size_t kValueSize = 32;
	char line[kNameSize + kValueSize + 2];
	char name[kNameSize];
	char value[kValueSize];

	while (readNextLine(fp, line, sizeof(line))) {
		if (!parseNameValue(line, name, sizeof(name), value, sizeof(value))) { continue; }

		PX4_INFO("cannode_cfg: node %u  SET %s = %s", node_id, name, value);
		UavcanNode::instance()->set_param(static_cast<int>(node_id), name, value);
	}

	fclose(fp);
	PX4_INFO("cannode_cfg: configuration complete for node %u", node_id);
}

