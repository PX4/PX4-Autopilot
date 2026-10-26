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
#include "mavlink_ftp_path.h"

#include <gtest/gtest.h>

#include <string>

#include <px4_platform_common/defines.h>

namespace
{
std::string storage(const char *relative)
{
	return std::string(PX4_STORAGEDIR) + relative;
}

bool protected_(const std::string &path)
{
	return mavlink_ftp_path_is_write_protected(path.c_str());
}
} // namespace

TEST(MavlinkFtpPath, blocksBootConsumedPaths)
{
	EXPECT_TRUE(protected_(storage("/etc/config.txt")));
	EXPECT_TRUE(protected_(storage("/etc/extras.txt")));
	EXPECT_TRUE(protected_(storage("/etc/rc.txt")));
	EXPECT_TRUE(protected_(storage("/ext_autostart")));
	EXPECT_TRUE(protected_(storage("/ext_autostart/rc.autostart")));
	EXPECT_TRUE(protected_(storage("/ext_autostart_new")));
	EXPECT_TRUE(protected_(storage("/ext_autostart_new/rc.autostart")));
}

TEST(MavlinkFtpPath, foldsEquivalentSpellings)
{
	EXPECT_TRUE(protected_(storage("//etc//extras.txt")));
	EXPECT_TRUE(protected_(storage("/etc/./rc.txt")));
	EXPECT_TRUE(protected_(storage("/etc/extras.txt/")));
	EXPECT_TRUE(protected_(storage("/ETC/Extras.TXT")));   // storage is FAT
	EXPECT_TRUE(protected_(storage("/etc/extras.txt.")));  // FAT drops the trailing dot
	EXPECT_TRUE(protected_(storage("/etc/rc.txt ")));      // ... and the trailing space
}

TEST(MavlinkFtpPath, refusesSpellingsThatCannotBeCompared)
{
	EXPECT_TRUE(protected_(storage("/etc/../etc/rc.txt")));
	EXPECT_TRUE(protected_(storage("/etc/logging/../rc.txt")));
	EXPECT_TRUE(protected_(storage("/..")));
	EXPECT_TRUE(protected_(storage("/EXT_AU~1/rc.autostart"))); // FAT 8.3 alias shape
	EXPECT_TRUE(protected_(storage("/log/A~1.TXT")));
}

TEST(MavlinkFtpPath, keepsDocumentedWritesWorking)
{
	EXPECT_FALSE(protected_(storage("/etc/logging/logger_topics.txt"))); // docs/en/dev_log/logging.md
	EXPECT_FALSE(protected_(storage("/log/2026-10-26.ulg")));
	EXPECT_FALSE(protected_(storage("/etc/extras.txt.bak")));
	EXPECT_FALSE(protected_(storage("/ext_autostart_backup/rc.autostart")));
	EXPECT_FALSE(protected_(storage("/parameters_backup.bson")));
	EXPECT_FALSE(protected_(storage("/other/etc/rc.txt")));
	EXPECT_FALSE(protected_(storage("//log//a.txt"))); // same folding, harmless target
	EXPECT_FALSE(protected_(storage("/log/./a.txt")));
}
