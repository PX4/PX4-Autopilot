/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "util_parse.h"
#include <cstdio>

namespace px4
{
namespace logger
{
namespace util
{

bool parse_sess_dir_name(const char *name, int &sess_idx)
{
	return sscanf(name, "sess%d", &sess_idx) == 1;
}

bool parse_date_dir_name(const char *name, int &year, int &month, int &day)
{
	return sscanf(name, "%d-%d-%d", &year, &month, &day) == 3;
}

bool is_date_older(int y1, int m1, int d1, int y2, int m2, int d2)
{
	return y1 < y2 ||
	       (y1 == y2 && m1 < m2) ||
	       (y1 == y2 && m1 == m2 && d1 < d2);
}

void process_dir_entry(const char *name, LogDirInfo &info)
{
	int sess_idx;
	int year, month, day;

	if (parse_sess_dir_name(name, sess_idx)) {
		info.num_sess++;

		if (sess_idx > info.sess_idx_max) {
			info.sess_idx_max = sess_idx;
		}

		if (sess_idx < info.sess_idx_min) {
			info.sess_idx_min = sess_idx;
		}

	} else if (parse_date_dir_name(name, year, month, day)) {
		info.num_dates++;

		if (is_date_older(year, month, day, info.oldest_year, info.oldest_month, info.oldest_day)) {
			info.oldest_year = year;
			info.oldest_month = month;
			info.oldest_day = day;
		}
	}
}

} //namespace util
} //namespace logger
} //namespace px4
