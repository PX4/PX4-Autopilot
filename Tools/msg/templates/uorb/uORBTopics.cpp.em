@###############################################
@#
@# EmPy template for generating uORBTopics.cpp file
@# for logging purposes
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - msgs (List) list of all msg files
@#  - multi_topics (List) list of all multi-topic names
@###############################################
/****************************************************************************
 *
 *   Copyright (C) 2013-2022 PX4 Development Team. All rights reserved.
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

#include <uORB/topics/uORBTopics.hpp>
#include <uORB/uORB.h>
@{
msg_names = list(set([mn.replace(".msg", "") for mn in msgs])) # set() filters duplicates
msg_names.sort()
msgs_count = len(msg_names)

topic_names = list(set(topics)) # set() filters duplicates
topic_names.sort()
topics_count = len(topics)

}@
@[for msg_name in msg_names]@
#include <uORB/topics/@(msg_name).h>
@[end for]

const constexpr struct orb_metadata *const uorb_topics_list[ORB_TOPICS_COUNT] = {
@[for idx, topic_name in enumerate(topic_names, 1)]@
	ORB_ID(@(topic_name))@[if idx != topic_names], @[end if]
@[end for]
};

const struct orb_metadata *const *orb_get_topics()
{
	return uorb_topics_list;
}

const struct orb_metadata *get_orb_meta(ORB_ID id)
{
	if (id == ORB_ID::INVALID) {
		return nullptr;
	}

	return uorb_topics_list[static_cast<orb_id_size_t>(id)];
}
