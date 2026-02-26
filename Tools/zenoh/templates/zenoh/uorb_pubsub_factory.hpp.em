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
type_topic_count = len(datatypes_with_topics[topic_name])
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

@[for topic_name, rihs01_hash in rihs01_hashes.items()]@
const uint8_t @(topic_name)_hash[32] = @(rihs01_hash)
@[end for]

@[for idx, topic_name in enumerate(datatypes)]@
#ifdef CONFIG_ZENOH_PUBSUB_@(topic_name.upper())
@{
topic_names = datatypes_with_topics[topic_name]
}@
const orb_metadata* @(topic_name)_topic_meta[@(len(topic_names))] = {
@[for topic_name_inst in topic_names]@
	ORB_ID(@(topic_name_inst)),
@[end for]};
#endif
@[end for]

typedef struct {
	const char *data_type_name;
	const uint32_t *ops;
	const uint8_t *hash;
	const orb_metadata** orb_topic;
	const uint8_t orb_topics_size;
} UorbPubSubTopicBinder;

const UorbPubSubTopicBinder _topics[ZENOH_PUBSUB_COUNT] {
@{
uorb_id_idx = 0
}@
@[for idx, topic_name in enumerate(datatypes)]@
#ifdef CONFIG_ZENOH_PUBSUB_@(topic_name.upper())
@{
topic_names = datatypes_with_topics[topic_name]
}@
		{
		  "@(topic_name)",
		  px4_msgs_msg_@(topic_dict[topic_name])_cdrstream_desc.ops.ops,
		  @(topic_dict[topic_name])_hash,
		  @(topic_name)_topic_meta,
		  @(len(topic_names)),
		},
#endif
@[end for]
};

uORB_Zenoh_Publisher* genPublisher(const orb_metadata *meta, int instance) {
    for (auto &pub : _topics) {
        for(int i = 0; i < pub.orb_topics_size; i++) {
            if(pub.orb_topic[i]->o_id == meta->o_id) {
                return new uORB_Zenoh_Publisher(meta, pub.ops, instance);
            }
        }
    }
    return NULL;
}


uORB_Zenoh_Publisher* genPublisher(const char *name, int instance) {
    for (auto &pub : _topics) {
        for(int i = 0; i < pub.orb_topics_size; i++) {
            if(strcmp(pub.orb_topic[i]->o_name, name) == 0) {
                return new uORB_Zenoh_Publisher(pub.orb_topic[i], pub.ops, instance);
            }
        }
    }
    return NULL;
}


Zenoh_Subscriber* genSubscriber(const orb_metadata *meta, int instance) {
    for (auto &sub : _topics) {
        for(int i = 0; i < sub.orb_topics_size; i++) {
            if(sub.orb_topic[i]->o_id == meta->o_id) {
                return new uORB_Zenoh_Subscriber(meta, sub.ops, instance);
            }
        }
    }
    return NULL;
}


Zenoh_Subscriber* genSubscriber(const char *name, int instance) {
    for (auto &sub : _topics) {
        for(int i = 0; i < sub.orb_topics_size; i++) {
            if(strcmp(sub.orb_topic[i]->o_name, name) == 0) {
                return new uORB_Zenoh_Subscriber(sub.orb_topic[i], sub.ops, instance);
            }
        }
    }
    return NULL;
}

const char* getTypeName(const char *name) {
    for (auto &sub : _topics) {
        for(int i = 0; i < sub.orb_topics_size; i++) {
            if(strcmp(sub.orb_topic[i]->o_name, name) == 0) {
                return sub.data_type_name;
            }
        }
    }
    return NULL;
}


const uint8_t* getRIHS01_Hash(const orb_metadata *meta) {
    for (auto &sub : _topics) {
        for(int i = 0; i < sub.orb_topics_size; i++) {
            if(sub.orb_topic[i]->o_id == meta->o_id) {
                return sub.hash;
            }
        }
    }
    return NULL;
}

const uint8_t* getRIHS01_Hash(const char *name) {
    for (auto &sub : _topics) {
        for(int i = 0; i < sub.orb_topics_size; i++) {
            if(strcmp(sub.orb_topic[i]->o_name, name) == 0) {
                return sub.hash;
            }
        }
    }
    return NULL;
}
