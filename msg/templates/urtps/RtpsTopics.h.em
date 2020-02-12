@###############################################
@#
@# EmPy template for generating RtpsTopics.h file
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

from px_generate_uorb_topic_helper import * # this is in Tools/
from px_generate_uorb_topic_files import MsgScope # this is in Tools/

send_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.SEND]
recv_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.RECEIVE]
}@
/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 * Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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

#include <fastcdr/Cdr.h>
#include <condition_variable>
#include <queue>

@[for topic in send_topics]@
#include "@(topic)_Publisher.h"
@[end for]@
@[for topic in recv_topics]@
#include "@(topic)_Subscriber.h"
@[end for]@

class RtpsTopics {
public:
    bool init(std::condition_variable* t_send_queue_cv, std::mutex* t_send_queue_mutex, std::queue<uint8_t>* t_send_queue);
@[if send_topics]@
    void publish(uint8_t topic_ID, char data_buffer[], size_t len);
@[end if]@
@[if recv_topics]@
    bool getMsg(const uint8_t topic_ID, eprosima::fastcdr::Cdr &scdr);
@[end if]@

private:
@[if send_topics]@
    // Publishers
@[for topic in send_topics]@
    @(topic)_Publisher _@(topic)_pub;
@[end for]@
@[end if]@

@[if recv_topics]@
    // Subscribers
@[for topic in recv_topics]@
    @(topic)_Subscriber _@(topic)_sub;
@[end for]@

@[end if]@
};
