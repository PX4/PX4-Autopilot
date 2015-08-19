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

#include "uORBCommunicatorMock.hpp"
#include "uORB/uORB.h"
#include "uORBGtestTopics.hpp"
#include "px4_log.h"
#include <string.h>
#include "uORBManager.hpp"

#define LOG_TAG "uORBCommunicatorMock.cpp"

uORB_test::uORBCommunicatorMock::uORBCommunicatorMock()
: _rx_handler( nullptr )
{
/*
  _sub_topicA_copy_fd = orb_subscribe( ORB_ID( topicA_copy ), (void*)&_sub_semaphore );
  _sub_topicB_copy_fd = orb_subscribe( ORB_ID( topicB_copy), nullptr );
*/
  _topic_translation_map[ "topicA" ] = "topicA_copy";
  _topic_translation_map[ "topicB" ] = "topicB_copy";
  _topic_translation_map[ "topicA_copy" ] = "topicA";
  _topic_translation_map[ "topicB_copy" ] = "topicB";
}

int16_t uORB_test::uORBCommunicatorMock::add_subscription
(
    const char * messageName,
    int32_t msgRateInHz
)
{

  int16_t rc = 0;
  PX4_INFO( "got add_subscription for msg[%s] rate[%d]", messageName, msgRateInHz );
  _msgCounters[messageName]._add_subscriptionCount++;

  /*
  int16_t rc = -1;
  if( _rx_handler )
  {
    if( _topic_translation_map.find( messageName ) != _topic_translation_map.end() )
    {
       rc = _rx_handler->process_add_subscription
           (
              _topic_translation_map[messageName],
              msgRateInHz
           );
    }
  }
  */
  return rc;
}

int16_t uORB_test::uORBCommunicatorMock::remove_subscription
(
    const char * messageName
)
{
  int16_t rc = 0;
  PX4_INFO( "got remove_subscription for msg[%s]", messageName );
  _msgCounters[messageName]._remove_subscriptionCount++;
/*
  int16_t rc = -1;
  if( _rx_handler )
  {
    if( _topic_translation_map.find( messageName ) != _topic_translation_map.end() )
    {
       rc = _rx_handler->process_remove_subscription
           (
              _topic_translation_map[messageName]
           );
    }
  }
*/
  return rc;
}

int16_t uORB_test::uORBCommunicatorMock::register_handler
(
    uORBCommunicator::IChannelRxHandler* handler
)
{
  int16_t rc = 0;
  _rx_handler = handler;
  return rc;
}


int16_t uORB_test::uORBCommunicatorMock::send_message
(
    const char * messageName,
    int32_t length,
    uint8_t* data
)
{
  int16_t rc = 0;
  PX4_INFO( "send_message for msg[%s] datalen[%d]", messageName, length );
  if( uORB::Manager::get_instance()->is_remote_subscriber_present( messageName ) )
  {
    _msgCounters[messageName]._send_messageCount++;
    if( strcmp(messageName, "topicA") == 0 )
    {
      memcpy( &_topicAData, (void*)data, length );
    }
    else if( strcmp(messageName, "topicB") == 0 )
    {
      memcpy( &_topicBData, (void*)data, length );
    }
    else
    {
      //EPRINTF( "error messageName[%s] is not supported", messageName );
    }
  }
/*
  int16_t rc = -1;
  if( _rx_handler )
  {
    if( _topic_translation_map.find( messageName ) != _topic_translation_map.end() )
    {
      rc = _rx_handler->process_received_message
          ( _topic_translation_map[messageName], length, data );
    }
  }
*/
  return rc;
}


bool uORB_test::uORBCommunicatorMock::get_remote_topicA_data
(
    struct orb_topic_A* data
)
{
  bool rc = false;
  memcpy( data, &_topicAData, sizeof(_topicAData) );
  rc = true;
/*
  if( orb_copy(ORB_ID(topicA_copy), _sub_topicA_copy_fd, data) == OK )
  {
    rc = true;
  }
*/
  return rc;
}

bool uORB_test::uORBCommunicatorMock::get_remote_topicB_data
(
    struct orb_topic_B* data
)
{
  bool rc = false;
  memcpy( data, &_topicBData, sizeof(_topicBData) );
  rc = true;
/*

  if( orb_copy(ORB_ID(topicB_copy), _sub_topicB_copy_fd, data) == OK )
  {
    rc = true;
  }
*/
  return rc;
}

void uORB_test::uORBCommunicatorMock::reset_counters()
{
  InterfaceCounters resetCounter;
  resetCounter._add_subscriptionCount = 0;
  resetCounter._remove_subscriptionCount = 0;
  resetCounter._send_messageCount = 0;

  std::map<std::string, InterfaceCounters>::iterator it;
  for( it = _msgCounters.begin(); it != _msgCounters.end(); ++it )
  {
    it->second = resetCounter;
  }
}

uORB_test::uORBCommunicatorMock::InterfaceCounters uORB_test::uORBCommunicatorMock::get_interface_counters(const char * messageName )
{
  return _msgCounters[ messageName ];
}
