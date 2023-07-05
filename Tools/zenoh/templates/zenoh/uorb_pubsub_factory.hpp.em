@###############################################
@#
@# EmPy template for generating u.hpp file
@# for logging purposes
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - topics (List) list of all topic names
@###############################################
@{

topic_dict = dict(zip(datatypes, full_base_names))

topics_count = len(topics)
topic_names_all = list(set(topics)) # set() filters duplicates
topic_names_all.sort()

datatypes = list(set(datatypes)) # set() filters duplicates
datatypes.sort()

full_base_names = list(set(full_base_names)) # set() filters duplicates
full_base_names.sort()


}@
/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * uorb_pubsub_factory.hpp
 *
 * Defines generic, templatized uORB over Zenoh / ROS2
 */

#pragma once

#include <publishers/uorb_publisher.hpp>
#include <uORB/topics/uORBTopics.hpp>
@[for idx, topic_name in enumerate(full_base_names)]@
#include <px4/msg/@(topic_name).h>
@[end for]

@[for idx, topic_name in enumerate(datatypes)]@
@{
type_topic_count = len([e for e in topic_names_all if e.startswith(topic_name)])
}@
#ifdef CONFIG_ZENOH_PUBSUB_@(topic_name.upper())
#  define CONFIG_ZENOH_PUBSUB_@(topic_name.upper())_COUNT @(type_topic_count)
#else
#  define CONFIG_ZENOH_PUBSUB_@(topic_name.upper())_COUNT 0
#endif
@[end for]

#define ZENOH_PUBSUB_COUNT \
@[for idx, topic_name in enumerate(datatypes)]@
        CONFIG_ZENOH_PUBSUB_@(topic_name.upper())_COUNT + \
@[end for]        0

typedef struct {
	const uint32_t *ops;
	const orb_metadata* orb_meta;
} UorbPubSubTopicBinder;

const UorbPubSubTopicBinder _topics[ZENOH_PUBSUB_COUNT] {
@{
uorb_id_idx = 0
}@
@[for idx, topic_name in enumerate(datatypes)]@
#ifdef CONFIG_ZENOH_PUBSUB_@(topic_name.upper())
@{
topic_names = [e for e in topic_names_all if e.startswith(topic_name)]
}@
@[for topic_name_inst in topic_names]@
		{
		  px4_msg_@(topic_dict[topic_name])_cdrstream_desc.ops.ops,
		  ORB_ID(@(topic_name_inst))
		},
@{
uorb_id_idx += 1
}@
@[end for]#endif
@[end for]
};

uORB_Zenoh_Publisher* genPublisher(const orb_metadata *meta) {
    for (auto &pub : _topics) {
        if(pub.orb_meta->o_id == meta->o_id) {
            return new uORB_Zenoh_Publisher(meta, pub.ops);
        }
    }
    return NULL;
}


uORB_Zenoh_Publisher* genPublisher(const char *name) {
    for (auto &pub : _topics) {
        if(strcmp(pub.orb_meta->o_name, name) == 0) {
            return new uORB_Zenoh_Publisher(pub.orb_meta, pub.ops);
        }
    }
    return NULL;
}


Zenoh_Subscriber* genSubscriber(const orb_metadata *meta) {
    for (auto &sub : _topics) {
        if(sub.orb_meta->o_id == meta->o_id) {
            return new uORB_Zenoh_Subscriber(meta, sub.ops);
        }
    }
    return NULL;
}


Zenoh_Subscriber* genSubscriber(const char *name) {
    for (auto &sub : _topics) {
        if(strcmp(sub.orb_meta->o_name, name) == 0) {
            return new uORB_Zenoh_Subscriber(sub.orb_meta, sub.ops);
        }
    }
    return NULL;
}
