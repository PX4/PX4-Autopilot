/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#include <lib/tinybson/tinybson.h>
#include <sys/stat.h>
#include <libgen.h>

#include <fcntl.h>

static void print_usage(const char *reason = nullptr, const char *command = nullptr)
{
	if (reason) {
		PX4_ERR("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION("Utility to read BSON from a file and print or output document size.");

	if (command == nullptr || strcmp(command, "docsize") == 0) {
		PRINT_MODULE_USAGE_NAME_SIMPLE("bsondump docsize", "command");
		PRINT_MODULE_USAGE_ARG("<file>", "The BSON file to decode for document size.", false);
	}

	if (command == nullptr) {
		PRINT_MODULE_USAGE_NAME_SIMPLE("bsondump", "command");
		PRINT_MODULE_USAGE_ARG("<file>", "The BSON file to decode and print.", false);
	}
}

static int bson_print_callback(bson_decoder_t decoder, bson_node_t node)
{
	switch (node->type) {
	case BSON_EOO:
		PX4_INFO_RAW("BSON_EOO\n");
		return 0;

	case BSON_DOUBLE:
		PX4_INFO_RAW("BSON_DOUBLE: %s = %.6f\n", node->name, node->d);
		return 1;

	case BSON_BOOL:
		PX4_INFO_RAW("BSON_BOOL:   %s = %d\n", node->name, node->b);
		return 1;

	case BSON_INT32:
		PX4_INFO_RAW("BSON_INT32:  %s = %" PRIi32 "\n", node->name, node->i32);
		return 1;

	case BSON_INT64:
		PX4_INFO_RAW("BSON_INT64:  %s = %" PRIi64 "\n", node->name, node->i64);
		return 1;

	default:
		PX4_INFO_RAW("ERROR %s unhandled bson type %d\n", node->name, node->type);
		return 1; // just skip this entry
	}

	return -1;
}

static int
bson_docsize_callback(bson_decoder_t decoder, bson_node_t node)
{

	if (node->type == BSON_EOO) {
		PX4_DEBUG("end of parameters");
		return 0;
	}

	return 1;
}

int copy_file(const char *src_path, const char *dst_path)
{
	int src_fd = open(src_path, O_RDONLY);
	int dst_fd = open(dst_path, O_WRONLY | O_TRUNC);

	if (src_fd == -1 || dst_fd == -1) {
		perror("Failed to open files for copying");

		if (src_fd != -1) { close(src_fd); }

		if (dst_fd != -1) { close(dst_fd); }

		return -1;
	}

	char buffer[1024];
	ssize_t bytes_read;

	while ((bytes_read = read(src_fd, buffer, sizeof(buffer))) > 0) {
		if (write(dst_fd, buffer, bytes_read) != bytes_read) {
			perror("Failed to copy file");
			close(src_fd);
			close(dst_fd);
			return -1;
		}
	}

	close(src_fd);
	close(dst_fd);

	if (bytes_read == -1) {
		perror("Error reading from source file");
		return -1;
	}

	return 0;
}

extern "C" __EXPORT int bsondump_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage("Invalid number of arguments.");
		return -1;
	}

	char *command = argv[1];
	char *file_name = argv[3];

	if (strcmp(command, "docsize") == 0) {
		if (argc != 3) {
			print_usage("Usage: bsondump docsize <file>", "docsize");
			return -1;
		}

		file_name = argv[2];
		int source_fd = open(file_name, O_RDONLY);

		if (source_fd < 0) {
			PX4_ERR("open '%s' failed (%i)", file_name, errno);
			return 1;
		}

		bson_decoder_s decoder{};
		int result = -1;

		if (bson_decoder_init_file(&decoder, source_fd, bson_docsize_callback) == 0) {

			do {
				result = bson_decoder_next(&decoder);

			} while (result > 0);

			PX4_INFO("DECODED_SIZE:%" PRId32 " SAVED_SIZE:%" PRId32 "\n",
				 decoder.total_decoded_size, decoder.total_document_size);
		}

		close(source_fd);

		if (decoder.total_decoded_size != decoder.total_document_size && decoder.total_document_size == 0
		    && strstr(file_name, "mtd_caldata") != NULL) {

			PX4_WARN("Mismatch in BSON sizes and saved size is zero. Setting document size to decoded size.");

			const char *source_path = "/fs/mtd_caldata";
			const char *temp_path = "/fs/microsd/tmp/mtd_caldata";

			source_fd = open(source_path, O_RDONLY);

			if (source_fd == -1) {
				perror("Failed to re-open source file for reading");
				return -1;
			}

			struct stat st;

			if (fstat(source_fd, &st) != 0) {
				perror("Failed to stat source file");
				close(source_fd);
				return -1;
			}

			char *buffer = new char[st.st_size];

			if (read(source_fd, buffer, st.st_size) != st.st_size) {
				perror("Failed to read source file");
				delete[] buffer;
				close(source_fd);
				return -1;
			}

			// Modify the first 4 bytes with the correct decoded size
			uint32_t corrected_size = decoder.total_decoded_size;
			memcpy(buffer, &corrected_size, sizeof(corrected_size));

			char *dir_path = strdup(temp_path);
			char *dir = dirname(dir_path);

			struct stat st2 = {0};

			if (stat(dir, &st2) == -1) {
				mkdir(dir, 0777);
			}

			free(dir_path);

			int temp_fd = open(temp_path, O_WRONLY | O_CREAT | O_TRUNC, 0666);

			if (temp_fd == -1) {
				perror("Failed to open temp file");
				delete[] buffer;
				return -1;
			}

			if (write(temp_fd, buffer, st.st_size) != st.st_size) {
				perror("Failed to write to temp file");
				delete[] buffer;
				close(temp_fd);
				return -1;
			}

			close(source_fd);
			close(temp_fd);
			delete[] buffer;

			if (copy_file(temp_path, source_path) != 0) {
				perror("Failed to copy temp file to source file");
				return -1;
			}

			if (remove(temp_path) != 0) {
				perror("Failed to remove temp file");
				return -1;
			}

			return 1;
		}

		return 0;


	} else {

		file_name = argv[1];
		int fd = open(file_name, O_RDONLY);

		if (fd < 0) {
			PX4_ERR("open '%s' failed (%i)", file_name, errno);
			return 1;

		} else {
			PX4_INFO_RAW("[bsondump] reading from %s\n", file_name);

			bson_decoder_s decoder{};

			if (bson_decoder_init_file(&decoder, fd, bson_print_callback) == 0) {
				PX4_INFO_RAW("BSON document size %" PRId32 "\n", decoder.total_document_size);

				int result = -1;

				do {
					result = bson_decoder_next(&decoder);

				} while (result > 0);

				close(fd);

				if (result == 0) {
					PX4_INFO_RAW("BSON decoded %" PRId32 " bytes (double:%" PRIu16 ", string:%" PRIu16 ", bin:%" PRIu16 ", bool:%" PRIu16
						     ", int32:%" PRIu16 ", int64:%" PRIu16 ")\n",
						     decoder.total_decoded_size,
						     decoder.count_node_double, decoder.count_node_string, decoder.count_node_bindata, decoder.count_node_bool,
						     decoder.count_node_int32, decoder.count_node_int64);

					return 0;

				} else if (result == -ENODATA) {
					PX4_WARN("no data");
					return -1;

				} else {
					PX4_ERR("failed (%d)", result);
					return -1;
				}
			}
		}

	}

	return -1;
}
