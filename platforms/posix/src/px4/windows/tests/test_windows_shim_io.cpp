/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
 * @file test_windows_shim_io.cpp
 *
 * Unit tests for the inline I/O helpers in unistd.h and sys/stat.h:
 *   pipe(), pipe2(), fsync(), fdatasync(), dprintf(), vdprintf(),
 *   px4_mkdir_shim() / mkdir(), and lstat() (the latter is implemented
 *   as a stat() pass-through).
 *
 * These all live in the shim headers and so are unit-testable without
 * pulling in the PX4 platform stack.
 */

#include <gtest/gtest.h>

#ifdef _WIN32

#include <windows.h>
#include <io.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>
#include <sys/stat.h>

TEST(WindowsShimIo, PipeCreatesUsableFds)
{
	int p[2] = { -1, -1 };
	ASSERT_EQ(pipe(p), 0);
	EXPECT_GE(p[0], 0);
	EXPECT_GE(p[1], 0);
	const char msg[] = "round";
	ASSERT_EQ(_write(p[1], msg, (unsigned)sizeof(msg)), (int)sizeof(msg));
	char buf[16] = {};
	ASSERT_EQ(_read(p[0], buf, sizeof(buf)), (int)sizeof(msg));
	EXPECT_STREQ(buf, "round");
	_close(p[0]);
	_close(p[1]);
}

TEST(WindowsShimIo, Pipe2IgnoresFlags)
{
	int p[2] = { -1, -1 };
	/* Flags are documented as ignored on Windows. */
	ASSERT_EQ(pipe2(p, 0xDEADBEEF), 0);
	EXPECT_GE(p[0], 0);
	EXPECT_GE(p[1], 0);
	_close(p[0]);
	_close(p[1]);
}

TEST(WindowsShimIo, FsyncOnRegularFile)
{
	char path[MAX_PATH];
	GetTempPathA(sizeof(path), path);
	strcat_s(path, sizeof(path), "px4_fsync.dat");
	int fd = _open(path, _O_CREAT | _O_RDWR | _O_BINARY, _S_IREAD | _S_IWRITE);
	ASSERT_GE(fd, 0);
	const char data[] = "hello";
	ASSERT_EQ(_write(fd, data, (unsigned)sizeof(data)), (int)sizeof(data));
	EXPECT_EQ(fsync(fd), 0);
	EXPECT_EQ(fdatasync(fd), 0);
	_close(fd);
	DeleteFileA(path);
}

TEST(WindowsShimIo, FsyncBadFdReturnsError)
{
	errno = 0;
	EXPECT_NE(fsync(-1), 0);
}

TEST(WindowsShimIo, DprintfWritesFormatted)
{
	char path[MAX_PATH];
	GetTempPathA(sizeof(path), path);
	strcat_s(path, sizeof(path), "px4_dprintf.txt");
	int fd = _open(path, _O_CREAT | _O_RDWR | _O_BINARY | _O_TRUNC,
		       _S_IREAD | _S_IWRITE);
	ASSERT_GE(fd, 0);
	int n = dprintf(fd, "v=%d s=%s", 42, "ok");
	EXPECT_GT(n, 0);
	_lseek(fd, 0, SEEK_SET);
	char buf[64] = {};
	int r = _read(fd, buf, sizeof(buf) - 1);
	ASSERT_GT(r, 0);
	buf[r] = 0;
	EXPECT_STREQ(buf, "v=42 s=ok");
	_close(fd);
	DeleteFileA(path);
}

static int call_vdprintf(int fd, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	int n = vdprintf(fd, fmt, ap);
	va_end(ap);
	return n;
}

TEST(WindowsShimIo, VdprintfWritesFormatted)
{
	char path[MAX_PATH];
	GetTempPathA(sizeof(path), path);
	strcat_s(path, sizeof(path), "px4_vdprintf.txt");
	int fd = _open(path, _O_CREAT | _O_RDWR | _O_BINARY | _O_TRUNC,
		       _S_IREAD | _S_IWRITE);
	ASSERT_GE(fd, 0);
	int n = call_vdprintf(fd, "x=%d", 7);
	EXPECT_GT(n, 0);
	_lseek(fd, 0, SEEK_SET);
	char buf[16] = {};
	int r = _read(fd, buf, sizeof(buf) - 1);
	ASSERT_GT(r, 0);
	buf[r] = 0;
	EXPECT_STREQ(buf, "x=7");
	_close(fd);
	DeleteFileA(path);
}

TEST(WindowsShimIo, MkdirCreatesDirectory)
{
	char path[MAX_PATH];
	GetTempPathA(sizeof(path), path);
	strcat_s(path, sizeof(path), "px4_mkdir_shim_test");
	RemoveDirectoryA(path); /* tidy from previous run */
	EXPECT_EQ(mkdir(path, 0755), 0);
	struct stat st = {};
	ASSERT_EQ(stat(path, &st), 0);
	EXPECT_TRUE(S_ISDIR(st.st_mode));
	RemoveDirectoryA(path);
}

TEST(WindowsShimIo, MkdirExistingFails)
{
	char path[MAX_PATH];
	GetTempPathA(sizeof(path), path);
	strcat_s(path, sizeof(path), "px4_mkdir_shim_test2");
	RemoveDirectoryA(path);
	ASSERT_EQ(mkdir(path, 0755), 0);
	EXPECT_NE(mkdir(path, 0755), 0);
	RemoveDirectoryA(path);
}

TEST(WindowsShimIo, LstatBehavesLikeStatForFile)
{
	char path[MAX_PATH];
	GetTempPathA(sizeof(path), path);
	strcat_s(path, sizeof(path), "px4_lstat.dat");
	int fd = _open(path, _O_CREAT | _O_RDWR | _O_BINARY, _S_IREAD | _S_IWRITE);
	ASSERT_GE(fd, 0);
	_close(fd);

	struct stat a = {};
	struct stat b = {};
	ASSERT_EQ(lstat(path, &a), 0);
	ASSERT_EQ(stat(path, &b), 0);
	EXPECT_EQ(a.st_size, b.st_size);
	EXPECT_TRUE(S_ISREG(a.st_mode));
	/* lstat() never reports a symlink on Windows. */
	EXPECT_FALSE(S_ISLNK(a.st_mode));
	DeleteFileA(path);
}

TEST(WindowsShimIo, LstatNonexistentFails)
{
	struct stat st = {};
	EXPECT_NE(lstat("Z:\\definitely\\not\\here\\px4test", &st), 0);
}

TEST(WindowsShimIo, StModeBitsExpose)
{
	/* Smoke test the POSIX permission constants the shim exports. */
	EXPECT_EQ(S_IRWXU, 0700);
	EXPECT_EQ(S_IRUSR, 0400);
	EXPECT_EQ(S_IWUSR, 0200);
	EXPECT_EQ(S_IXUSR, 0100);
	EXPECT_EQ(S_IRWXG, 0070);
	EXPECT_EQ(S_IRWXO, 0007);
	EXPECT_EQ(S_IFDIR, 0040000);
	EXPECT_EQ(S_IFREG, 0100000);
}

#endif // _WIN32

TEST(WindowsShimIo, BuildSentinel)
{
	SUCCEED();
}
