#include "sdcard_check.h"
#include "sdcard_checksums.h"

#include <crc32.h>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <px4_platform_common/defines.h>

namespace sdcard_check
{

Result verify_all()
{
	Result result{};

#ifdef PX4_STORAGEDIR

	for (unsigned i = 0; i < num_files; i++) {
		result.num_checked++;

		char path[256];
		snprintf(path, sizeof(path), "%s/%s", PX4_STORAGEDIR, files[i].relative_path);

		int fd = open(path, O_RDONLY);

		if (fd < 0) {
			result.num_missing++;
			continue;
		}

		uint32_t crc = 0;
		uint8_t buf[256];
		ssize_t n;

		while ((n = read(fd, buf, sizeof(buf))) > 0) {
			crc = crc32part(buf, n, crc);
		}

		close(fd);

		if (crc != files[i].expected_crc) {
			result.num_mismatch++;
		}
	}

#endif /* PX4_STORAGEDIR */

	return result;
}

} // namespace sdcard_check
