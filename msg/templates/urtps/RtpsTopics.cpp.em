@###############################################
@#
@# EmPy template for generating RtpsTopics.cpp file
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - msgs (List) list of all msg files
@#  - ids (List) list of all RTPS msg ids
@###############################################
@{
import os

import genmsg.msgs
import gencpp
from px_generate_uorb_topic_helper import * # this is in Tools/
from px_generate_uorb_topic_files import MsgScope # this is in Tools/

send_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.SEND]
recv_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.RECEIVE]
package = package[0]
fastrtpsgen_version = fastrtpsgen_version[0]
ros2_distro = ros2_distro[0].decode("utf-8")
}@
/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 * Copyright (C) 2018-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "RtpsTopics.h"

bool RtpsTopics::init()
{
@[if recv_topics]@
    // Initialise subscribers
@[for topic in recv_topics]@
    if (_@(topic)_sub.init()) {
        std::cout << "@(topic) subscriber started" << std::endl;
    } else {
        std::cout << "ERROR starting @(topic) subscriber" << std::endl;
        return false;
    }

@[end for]@
@[end if]@
@[if send_topics]@
    // Initialise publishers
@[for topic in send_topics]@
    if (_@(topic)_pub.init()) {
        std::cout << "@(topic) publisher started" << std::endl;
    } else {
        std::cout << "ERROR starting @(topic) publisher" << std::endl;
        return false;
    }

@[end for]@
@[end if]@
    return true;
}

@[if send_topics]@
void RtpsTopics::publish(uint8_t topic_ID, char data_buffer[], size_t len)
{
    switch (topic_ID)
    {
@[for topic in send_topics]@
        case @(rtps_message_id(ids, topic)): // @(topic)
        {
@[    if 1.5 <= fastrtpsgen_version <= 1.7]@
@[        if ros2_distro]@
            @(package)::msg::dds_::@(topic)_ st;
@[        else]@
            @(topic)_ st;
@[        end if]@
@[    else]@
@[        if ros2_distro]@
            @(package)::msg::@(topic) st;
@[        else]@
            @(topic) st;
@[        end if]@
@[    end if]@
            eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
            eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
            st.deserialize(cdr_des);
            _@(topic)_pub.publish(&st);
        }
        break;
@[end for]@
        default:
            printf("Unexpected topic ID to publish\n");
        break;
    }
}
@[end if]@
@[if recv_topics]@

bool RtpsTopics::hasMsg(uint8_t *topic_ID)
{
    if (nullptr == topic_ID) return false;

    *topic_ID = 0;
    while (_next_sub_idx < @(len(recv_topics)) && 0 == *topic_ID)
    {
        switch (_sub_topics[_next_sub_idx])
        {
@[for topic in recv_topics]@
            case @(rtps_message_id(ids, topic)): if (_@(topic)_sub.hasMsg()) *topic_ID = @(rtps_message_id(ids, topic)); break;
@[end for]@
            default:
                printf("Unexpected topic ID to check hasMsg\n");
            break;
        }
        _next_sub_idx++;
    }

    if (0 == *topic_ID)
    {
        _next_sub_idx = 0;
        return false;
    }

    return true;
}

bool RtpsTopics::getMsg(const uint8_t topic_ID, eprosima::fastcdr::Cdr &scdr)
{
    bool ret = false;
    switch (topic_ID)
    {
@[for topic in recv_topics]@
        case @(rtps_message_id(ids, topic)): // @(topic)
            if (_@(topic)_sub.hasMsg())
            {
@[    if 1.5 <= fastrtpsgen_version <= 1.7]@
@[        if ros2_distro]@
                @(package)::msg::dds_::@(topic)_ msg = _@(topic)_sub.getMsg();
@[        else]@
                @(topic)_ msg = _@(topic)_sub.getMsg();
@[        end if]@
@[    else]@
@[        if ros2_distro]@
                @(package)::msg::@(topic) msg = _@(topic)_sub.getMsg();
@[        else]@
                @(topic) msg = _@(topic)_sub.getMsg();
@[        end if]@
@[    end if]@
                msg.serialize(scdr);
                ret = true;
            }
        break;
@[end for]@
        default:
            printf("Unexpected topic ID '%hhu' to getMsg\n", topic_ID);
        break;
    }

    return ret;
}
@[end if]@
