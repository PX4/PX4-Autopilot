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

#include <fcntl.h>

static void print_usage(const char *reason = nullptr)
{
	if (reason) {
		PX4_ERR("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION("read BSON from a file and print in human form");

	PRINT_MODULE_USAGE_NAME_SIMPLE("bsondump", "command");
	PRINT_MODULE_USAGE_ARG("<file>", "File name", false);
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

extern "C" __EXPORT int bsondump_main(int argc, char *argv[])
{
	if (argc != 2) {
		print_usage();

	} else {
		char *file_name = argv[1];

		if (file_name) {
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
	}

	return -1;
}
