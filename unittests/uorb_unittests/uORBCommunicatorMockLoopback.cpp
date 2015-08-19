/****************************************************************************
 *
 * Copyright (c) 2015 Mark Charlebois. All rights reserved.
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

#include "uORBCommunicatorMockLoopback.hpp"
#include "uORB/uORB.h"
#include "uORBGtestTopics.hpp"
#include "uORBManager.hpp"
#include <string.h>
#include "px4_log.h"

#define LOG_TAG "uORBCommunicatorMockLoopback.cpp"


uORB_test::uORBCommunicatorMockLoopback::uORBCommunicatorMockLoopback()
: _rx_handler( nullptr )
{
  //_sub_topicA_clone_fd = orb_subscribe( ORB_ID( topicA_clone ), (void*)&_sub_semaphore );
  //_sub_topicB_clone_fd = orb_subscribe( ORB_ID( topicB_clone ), nullptr );

  _topic_translation_map[ "topicA" ] = "topicA_clone";
  _topic_translation_map[ "topicB" ] = "topicB_clone";
  _topic_translation_map[ "topicA_clone" ] = "topicA";
  _topic_translation_map[ "topicB_clone" ] = "topicB";
}

int16_t uORB_test::uORBCommunicatorMockLoopback::add_subscription
(
    const char * messageName,
    int32_t msgRateInHz
)
{

  int16_t rc = -1;
  PX4_INFO( "got add_subscription for msg[%s] rate[%d]", messageName, msgRateInHz );

  if( _rx_handler )
  {
    if( _topic_translation_map.find( messageName ) != _topic_translation_map.end() )
    {
       rc = _rx_handler->process_add_subscription
           (
              _topic_translation_map[messageName].c_str(),
              msgRateInHz
           );
    }
  }
  return rc;
}

int16_t uORB_test::uORBCommunicatorMockLoopback::remove_subscription
(
    const char * messageName
)
{
  int16_t rc = -1;
  PX4_INFO( "got remove_subscription for msg[%s]", messageName );
  if( _rx_handler )
  {
    if( _topic_translation_map.find( messageName ) != _topic_translation_map.end() )
    {
       rc = _rx_handler->process_remove_subscription
           (
              _topic_translation_map[messageName].c_str()
           );
    }
  }
  return rc;
}

int16_t uORB_test::uORBCommunicatorMockLoopback::register_handler
(
    uORBCommunicator::IChannelRxHandler* handler
)
{
  int16_t rc = 0;
  _rx_handler = handler;
  return rc;
}


int16_t uORB_test::uORBCommunicatorMockLoopback::send_message
(
    const char * messageName,
    int32_t length,
    uint8_t* data
)
{
  int16_t rc = -1;
  PX4_INFO( "send_message for msg[%s] datalen[%d]", messageName, length );
  if( _rx_handler )
  {
    if( _topic_translation_map.find( messageName ) != _topic_translation_map.end() )
    {
      if( uORB::Manager::get_instance()->is_remote_subscriber_present( _topic_translation_map[messageName].c_str() ) )
      {
        rc = _rx_handler->process_received_message
            ( _topic_translation_map[messageName].c_str(), length, data );
        PX4_INFO( "[uORBCommunicatorMockLoopback::send_message] return from[topic(%s)] _rx_handler->process_received_message[%d]", messageName, rc );
      }
      else
      {
        // this is eqvuilanet of not sending the message to the remote.
        rc = 0;
      }
    }
  }
  return rc;
}
