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

#include <px4_platform_common/defines.h>
#include <px4_daemon/server_io.h>

#include <sys/boardctl.h>
#include <sys/statfs.h>
#include <sys/termios.h>

#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>

#if defined(CONFIG_NET_SOCKETS)
#include <arpa/inet.h>
#endif

__BEGIN_DECLS

long PX4_TICKS_PER_SEC = CONFIG_SYS_CLOCK_TICKS_PER_SEC;

int boardctl(unsigned int cmd, uintptr_t arg)
{
	if (cmd == BOARDIOC_RESET) {
		sys_reboot(SYS_REBOOT_COLD);
	}

	return -ENOTSUP;
}

// only the existence of the file is checked
int access(const char *pathname, int mode)
{
	struct stat st;
	return stat(pathname, &st);
}

int truncate(const char *path, off_t length)
{
	const int fd = open(path, O_WRONLY);

	if (fd < 0) {
		return -1;
	}

	const int ret = ftruncate(fd, length);
	close(fd);
	return ret;
}

#if defined(CONFIG_NET_SOCKETS)
int inet_aton(const char *cp, struct in_addr *inp)
{
	return inet_pton(AF_INET, cp, inp) == 1;
}
#endif

int statfs(const char *path, struct statfs *buf)
{
	struct fs_statvfs stat;
	const int ret = fs_statvfs(path, &stat);

	if (ret != 0) {
		errno = -ret;
		return -1;
	}

	buf->f_bsize = stat.f_frsize;
	buf->f_blocks = stat.f_blocks;
	buf->f_bfree = stat.f_bfree;
	buf->f_bavail = stat.f_bfree;
	return 0;
}

speed_t cfgetispeed(const struct termios *termios_p)
{
	return termios_p->c_ispeed;
}

speed_t cfgetospeed(const struct termios *termios_p)
{
	return termios_p->c_ospeed;
}

int cfsetispeed(struct termios *termios_p, speed_t speed)
{
	termios_p->c_ispeed = speed;
	return 0;
}

int cfsetospeed(struct termios *termios_p, speed_t speed)
{
	termios_p->c_ospeed = speed;
	return 0;
}

int tcdrain(int fd)
{
	errno = ENOTTY;
	return -1;
}

int tcflush(int fd, int queue_selector)
{
	errno = ENOTTY;
	return -1;
}

int tcgetattr(int fd, struct termios *termios_p)
{
	errno = ENOTTY;
	return -1;
}

int tcsetattr(int fd, int optional_actions, const struct termios *termios_p)
{
	errno = ENOTTY;
	return -1;
}

int dprintf(int fd, const char *format, ...)
{
	va_list args;
	va_start(args, format);
	const int len = vsnprintf(nullptr, 0, format, args);
	va_end(args);

	if (len <= 0) {
		return len;
	}

	char *buf = (char *)malloc(len + 1);

	if (buf == nullptr) {
		errno = ENOMEM;
		return -1;
	}

	va_start(args, format);
	vsnprintf(buf, len + 1, format, args);
	va_end(args);

	const int ret = write(fd, buf, len);
	free(buf);
	return ret;
}

__END_DECLS

FILE *get_stdout(bool *isatty_)
{
	if (isatty_) {
		*isatty_ = false;
	}

	return stdout;
}
