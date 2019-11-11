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
try:
    ros2_distro = ros2_distro.decode("utf-8")
except AttributeError:
    pass
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
 * @@file @(topic)_Subscriber.cpp
 * This file contains the implementation of the subscriber functions.
 *
 * This file was adapted from the fastcdrgen tool.
 */

#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/attributes/SubscriberAttributes.h>

#include <fastrtps/Domain.h>

#include "@(topic)_Subscriber.h"

@(topic)_Subscriber::@(topic)_Subscriber() : mp_participant(nullptr), mp_subscriber(nullptr) {}

@(topic)_Subscriber::~@(topic)_Subscriber() {   Domain::removeParticipant(mp_participant);}

bool @(topic)_Subscriber::init(uint8_t topic_ID, std::condition_variable* t_send_queue_cv, std::mutex* t_send_queue_mutex, std::queue<uint8_t>* t_send_queue)
{
    m_listener.topic_ID = topic_ID;
    m_listener.t_send_queue_cv = t_send_queue_cv;
    m_listener.t_send_queue_mutex = t_send_queue_mutex;
    m_listener.t_send_queue = t_send_queue;

    // Create RTPSParticipant
    ParticipantAttributes PParam;
    PParam.rtps.builtin.domainId = 0; // MUST BE THE SAME AS IN THE PUBLISHER
@[if 1.5 <= fastrtpsgen_version <= 1.7 or ros2_distro == "ardent" or ros2_distro == "bouncy" or ros2_distro == "crystal" or ros2_distro == "dashing"]@
    PParam.rtps.builtin.leaseDuration = c_TimeInfinite;
@[else]@
    PParam.rtps.builtin.discovery_config.leaseDuration = c_TimeInfinite;
@[end if]@
    PParam.rtps.setName("@(topic)_subscriber"); //You can put the name you want
    mp_participant = Domain::createParticipant(PParam);
    if(mp_participant == nullptr)
            return false;

    //Register the type
    Domain::registerType(mp_participant, static_cast<TopicDataType*>(&myType));

    // Create Subscriber
    SubscriberAttributes Rparam;
    Rparam.topic.topicKind = NO_KEY;
    Rparam.topic.topicDataType = myType.getName(); //Must be registered before the creation of the subscriber
@[if ros2_distro]@
@[    if ros2_distro == "ardent"]@
    Rparam.qos.m_partition.push_back("rt");
    Rparam.topic.topicName = "@(topic)_PubSubTopic";
@[    else]@
    Rparam.topic.topicName = "rt/@(topic)_PubSubTopic";
@[    end if]@
@[else]@
    Rparam.topic.topicName = "@(topic)_PubSubTopic";
@[end if]@
    mp_subscriber = Domain::createSubscriber(mp_participant, Rparam, static_cast<SubscriberListener*>(&m_listener));
    if(mp_subscriber == nullptr)
        return false;
    return true;
}

void @(topic)_Subscriber::SubListener::onSubscriptionMatched(Subscriber* sub, MatchingInfo& info)
{
    (void)sub;

    if (info.status == MATCHED_MATCHING)
    {
        n_matched++;
        std::cout << "Subscriber matched" << std::endl;
    }
    else
    {
        n_matched--;
        std::cout << "Subscriber unmatched" << std::endl;
    }
}

void @(topic)_Subscriber::SubListener::onNewDataMessage(Subscriber* sub)
{
        std::unique_lock<std::mutex> has_msg_lock(has_msg_mutex);
        if(has_msg.load() == true) // Check if msg has been fetched
        {
            has_msg_cv.wait(has_msg_lock); // Wait till msg has been fetched
        }
        has_msg_lock.unlock();
        

        // Take data
        if(sub->takeNextData(&msg, &m_info))
        {
            if(m_info.sampleKind == ALIVE)
            {
                std::unique_lock<std::mutex> lk(*t_send_queue_mutex);
            
                ++n_msg;
                has_msg = true;
                
                t_send_queue->push(topic_ID);
                lk.unlock();
                t_send_queue_cv->notify_one();

            }
        }
}

void @(topic)_Subscriber::run()
{
    std::cout << "Waiting for Data, press Enter to stop the Subscriber. "<<std::endl;
    std::cin.ignore();
    std::cout << "Shutting down the Subscriber." << std::endl;
}

bool @(topic)_Subscriber::hasMsg()
{
    return m_listener.has_msg.load();
}

@[if 1.5 <= fastrtpsgen_version <= 1.7]@
@[    if ros2_distro]@
@(package)::msg::dds_::@(topic)_ @(topic)_Subscriber::getMsg()
@[    else]@
@(topic)_ @(topic)_Subscriber::getMsg()
@[    end if]@
@[else]@
@[    if ros2_distro]@
@(package)::msg::@(topic) @(topic)_Subscriber::getMsg()
@[    else]@
@(topic) @(topic)_Subscriber::getMsg()
@[    end if]@
@[end if]@
{
    return m_listener.msg;
}

void @(topic)_Subscriber::unlockMsg()
{
    std::unique_lock<std::mutex> has_msg_lock(m_listener.has_msg_mutex);
    m_listener.has_msg = false;
    has_msg_lock.unlock();
    m_listener.has_msg_cv.notify_one();
}
