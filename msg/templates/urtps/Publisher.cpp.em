@###############################################
@#
@# EmPy template for generating <msg>_uRTPS_UART.cpp file
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - msgs (List) list of all msg files
@#  - multi_topics (List) list of all multi-topic names
@#  - ids (List) list of all RTPS msg ids
@###############################################
@{
import genmsg.msgs
import gencpp
from px_generate_uorb_topic_helper import * # this is in Tools/

topic = alias if alias else spec.short_name
ros2_distro = ros2_distro.decode("utf-8")
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

/*!
 * @@file @(topic)_Publisher.cpp
 * This file contains the implementation of the publisher functions.
 *
 * This file was adapted from the fastcdrgen tool.
 */

#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/attributes/PublisherAttributes.h>

#include <fastrtps/Domain.h>

@[if 1.5 <= fastrtpsgen_version <= 1.7]@
#include <fastrtps/utils/eClock.h>
@[end if]@

#include "@(topic)_Publisher.h"


@(topic)_Publisher::@(topic)_Publisher() : mp_participant(nullptr), mp_publisher(nullptr) {}

@(topic)_Publisher::~@(topic)_Publisher() { Domain::removeParticipant(mp_participant);}

bool @(topic)_Publisher::init()
{
    // Create RTPSParticipant
    ParticipantAttributes PParam;
    PParam.rtps.builtin.domainId = 0;
@[if 1.5 <= fastrtpsgen_version <= 1.7 or ros2_distro == "ardent" or ros2_distro == "bouncy" or ros2_distro == "crystal" or ros2_distro == "dashing"]@
    PParam.rtps.builtin.leaseDuration = c_TimeInfinite;
@[else]@
    PParam.rtps.builtin.discovery_config.leaseDuration = c_TimeInfinite;
@[end if]@
    PParam.rtps.setName("@(topic)_publisher");  //You can put here the name you want
    mp_participant = Domain::createParticipant(PParam);
    if(mp_participant == nullptr)
        return false;

    // Register the type
    Domain::registerType(mp_participant, static_cast<TopicDataType*>(&myType));

    // Create Publisher
    PublisherAttributes Wparam;
    Wparam.topic.topicKind = NO_KEY;
    Wparam.topic.topicDataType = myType.getName();  //This type MUST be registered
@[if ros2_distro]@
@[    if ros2_distro == "ardent"]@
    Wparam.qos.m_partition.push_back("rt");
    Wparam.topic.topicName = "@(topic)_PubSubTopic";
@[    else]@
    Wparam.topic.topicName = "rt/@(topic)_PubSubTopic";
@[    end if]@
@[else]@
    Wparam.topic.topicName = "@(topic)_PubSubTopic";
@[end if]@
    mp_publisher = Domain::createPublisher(mp_participant, Wparam, static_cast<PublisherListener*>(&m_listener));
    if(mp_publisher == nullptr)
        return false;
    //std::cout << "Publisher created, waiting for Subscribers." << std::endl;
    return true;
}

void @(topic)_Publisher::PubListener::onPublicationMatched(Publisher* pub, MatchingInfo& info)
{
    if (info.status == MATCHED_MATCHING)
    {
        n_matched++;
        std::cout << "Publisher matched" << std::endl;
    }
    else
    {
        n_matched--;
        std::cout << "Publisher unmatched" << std::endl;
    }
}

void @(topic)_Publisher::run()
{
    while(m_listener.n_matched == 0)
    {
@[if 1.5 <= fastrtpsgen_version <= 1.7]@
        eClock::my_sleep(250); // Sleep 250 ms;
@[else]@
        std::this_thread::sleep_for(std::chrono::milliseconds(250)); // Sleep 250 ms
@[end if]@
    }

    // Publication code
@[if 1.5 <= fastrtpsgen_version <= 1.7]@
@[    if ros2_distro]@
    @(package)::msg::dds_::@(topic)_ st;
@[    else]@
    @(topic)_ st;
@[    end if]@
@[else]@
@[    if ros2_distro]@
    @(package)::msg::@(topic) st;
@[    else]@
    @(topic) st;
@[    end if]@
@[end if]@

    /* Initialize your structure here */

    int msgsent = 0;
    char ch = 'y';
    do
    {
        if(ch == 'y')
        {
            mp_publisher->write(&st);  ++msgsent;
            std::cout << "Sending sample, count=" << msgsent << ", send another sample?(y-yes,n-stop): ";
        }
        else if(ch == 'n')
        {
            std::cout << "Stopping execution " << std::endl;
            break;
        }
        else
        {
            std::cout << "Command " << ch << " not recognized, please enter \"y/n\":";
        }
    }while(std::cin >> ch);
}

@[if 1.5 <= fastrtpsgen_version <= 1.7]@
@[    if ros2_distro]@
    void @(topic)_Publisher::publish(@(package)::msg::dds_::@(topic)_* st)
@[    else]@
    void @(topic)_Publisher::publish(@(topic)_* st)
@[    end if]@
@[else]@
@[    if ros2_distro]@
    void @(topic)_Publisher::publish(@(package)::msg::@(topic)* st)
@[    else]@
    void @(topic)_Publisher::publish(@(topic)* st)
@[    end if]@
@[end if]@
{
    mp_publisher->write(st);
}
