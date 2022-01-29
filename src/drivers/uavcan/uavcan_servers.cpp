/****************************************************************************
 *
 *   Copyright (c) 2014-2021 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/tasks.h>
#include <drivers/drv_hrt.h>

#include <nuttx/config.h>

#include <cstdlib>
#include <cstring>
#include <cctype>
#include <fcntl.h>
#include <dirent.h>
#include <pthread.h>
#include <mathlib/mathlib.h>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <version/version.h>

#include <arch/chip/chip.h>

#include "uavcan_main.hpp"
#include "uavcan_servers.hpp"

#include <uavcan_posix/dynamic_node_id_server/file_event_tracer.hpp>
#include <uavcan_posix/dynamic_node_id_server/file_storage_backend.hpp>
#include <uavcan_posix/firmware_version_checker.hpp>

/**
 * @file uavcan_servers.cpp
 *
 * Implements basic functionality of UAVCAN node.
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 *         David Sidrane <david_s5@nscdg.com>
 */

UavcanServers::UavcanServers(uavcan::INode &node, uavcan::NodeInfoRetriever &node_info_retriever) :
	_server_instance(node, _storage_backend, _tracer),
	_fileserver_backend(node),
	_fw_upgrade_trigger(node, _fw_version_checker),
	_fw_server(node, _fileserver_backend),
	_node_info_retriever(node_info_retriever)
{
}

int UavcanServers::init()
{
	/*
	 * Initialize the fw version checker.
	 * giving it its path
	 */
	int ret = _fw_version_checker.createFwPaths(UAVCAN_FIRMWARE_PATH, UAVCAN_ROMFS_FW_PATH);

	if (ret < 0) {
		PX4_ERR("FirmwareVersionChecker init: %d, errno: %d", ret, errno);
		return ret;
	}

	/* Start fw file server back */
	ret = _fw_server.start(UAVCAN_FIRMWARE_PATH, UAVCAN_ROMFS_FW_PATH);

	if (ret < 0) {
		PX4_ERR("BasicFileServer init: %d, errno: %d", ret, errno);
		return ret;
	}

	/* Initialize storage back end for the node allocator using UAVCAN_NODE_DB_PATH directory */
	ret = _storage_backend.init(UAVCAN_NODE_DB_PATH);

	if (ret < 0) {
		PX4_ERR("FileStorageBackend init: %d, errno: %d", ret, errno);
		return ret;
	}

	/* Initialize trace in the UAVCAN_NODE_DB_PATH directory */
	ret = _tracer.init(UAVCAN_LOG_FILE);

	if (ret < 0) {
		PX4_ERR("FileEventTracer init: %d, errno: %d", ret, errno);
		return ret;
	}

	/* hardware version */
	uavcan::protocol::HardwareVersion hwver;
	UavcanNode::getHardwareVersion(hwver);

	/* Initialize the dynamic node id server  */
	ret = _server_instance.init(hwver.unique_id);

	if (ret < 0) {
		PX4_ERR("CentralizedServer init: %d", ret);
		return ret;
	}

	/* Start the fw version checker   */
	ret = _fw_upgrade_trigger.start(_node_info_retriever);

	if (ret < 0) {
		PX4_ERR("FirmwareUpdateTrigger init: %d", ret);
		return ret;
	}

	/*
	Check for firmware in the root directory, move it to appropriate location on
	the SD card, as defined by the APDesc.
	*/
	migrateFWFromRoot(UAVCAN_FIRMWARE_PATH, UAVCAN_SD_ROOT_PATH);

	/*  Start the Node   */
	return 0;
}

void UavcanServers::migrateFWFromRoot(const char *sd_path, const char *sd_root_path)
{
	/*
	Copy Any bin files with APDes into appropriate location on SD card
	overriding any firmware the user has already loaded there.

	The SD firmware directory structure is along the lines of:

	  /fs/microsd/ufw
	     nnnnn.bin - where n is the board_id
	*/

	const size_t maxlen = UAVCAN_MAX_PATH_LENGTH;
	const size_t sd_root_path_len = strlen(sd_root_path);
	struct stat sb;
	int rv;
	char dstpath[maxlen + 1];
	char srcpath[maxlen + 1];

	DIR *const sd_root_dir = opendir(sd_root_path);

	if (!sd_root_dir) {
		return;
	}

	if (stat(sd_path, &sb) != 0 || !S_ISDIR(sb.st_mode)) {
		rv = mkdir(sd_path, S_IRWXU | S_IRWXG | S_IRWXO);

		if (rv != 0) {
			PX4_ERR("dev: couldn't create '%s'", sd_path);
			return;
		}
	}

	// Iterate over all bin files in root directory
	struct dirent *dev_dirent = NULL;

	while ((dev_dirent = readdir(sd_root_dir)) != nullptr) {

		uavcan_posix::FirmwareVersionChecker::AppDescriptor descriptor;

		// Looking for all uavcan.bin files.

		if (DIRENT_ISFILE(dev_dirent->d_type) && strstr(dev_dirent->d_name, ".bin") != nullptr) {

			// Make sure the path fits

			size_t filename_len = strlen(dev_dirent->d_name);
			size_t srcpath_len = sd_root_path_len + 1 + filename_len;

			if (srcpath_len > maxlen) {
				PX4_WARN("file: srcpath '%s%s' too long", sd_root_path, dev_dirent->d_name);
				continue;
			}

			snprintf(srcpath, sizeof(srcpath), "%s%s", sd_root_path, dev_dirent->d_name);

			if (uavcan_posix::FirmwareVersionChecker::getFileInfo(srcpath, descriptor, 1024) != 0) {
				continue;
			}

			if (descriptor.image_crc == 0) {
				continue;
			}

			snprintf(dstpath, sizeof(dstpath), "%s/%d.bin", sd_path, descriptor.board_id);

			if (copyFw(dstpath, srcpath) >= 0) {
				unlink(srcpath);
			}
		}
	}

	if (dev_dirent != nullptr) {
		(void)closedir(dev_dirent);
	}
}

int UavcanServers::copyFw(const char *dst, const char *src)
{
	int rv = 0;
	uint8_t buffer[512] {};

	int dfd = open(dst, O_WRONLY | O_CREAT, 0666);

	if (dfd < 0) {
		PX4_ERR("copyFw: couldn't open dst");
		return -errno;
	}

	int sfd = open(src, O_RDONLY, 0);

	if (sfd < 0) {
		(void)close(dfd);
		PX4_ERR("copyFw: couldn't open src");
		return -errno;
	}

	ssize_t size = 0;

	do {
		size = read(sfd, buffer, sizeof(buffer));

		if (size < 0) {
			PX4_ERR("copyFw: couldn't read");
			rv = -errno;

		} else if (size > 0) {
			rv = 0;
			ssize_t remaining = size;
			ssize_t total_written = 0;
			ssize_t written = 0;

			do {
				written = write(dfd, &buffer[total_written], remaining);

				if (written < 0) {
					PX4_ERR("copyFw: couldn't write");
					rv = -errno;

				} else {
					total_written += written;
					remaining -=  written;
				}
			} while (written > 0 && remaining > 0);
		}
	} while (rv == 0 && size != 0);

	(void)close(dfd);
	(void)close(sfd);

	return rv;
}
