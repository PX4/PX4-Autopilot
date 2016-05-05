
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
#ifndef _uORBCommunicatorMock_hpp_
#define _uORBCommunicatorMock_hpp_

#include "uORB/uORBCommunicator.hpp"
#include "uORBGtestTopics.hpp"
#include <map>
#include <string>
#include <set>

namespace uORB_test
{
   class uORBCommunicatorMock;
}

class uORB_test::uORBCommunicatorMock : public uORBCommunicator::IChannel
{
 public:

  //counters to track how many times the iterface is called from
  // uorb.
  typedef struct
  {
    int64_t _add_subscriptionCount;
    int64_t _remove_subscriptionCount;
    int64_t _send_messageCount;
  }InterfaceCounters;

  uORBCommunicatorMock();

  /**
   * @brief Interface to notify the remote entity of interest of a
   * subscription for a message.
   *
   * @param messageName
   *  This represents the uORB message name; This message name should be
   *  globally unique.
   * @param msgRate
   *  The max rate at which the subscriber can accept the messages.
   * @return
   *  0 = success; This means the messages is successfully sent to the receiver
   *    Note: This does not mean that the receiver as received it.
   *  otherwise = failure.
   */
  virtual int16_t add_subscription( const char *messageName, int32_t msgRateInHz );

  /**
   * @brief Interface to notify the remote entity of removal of a subscription
   *
   * @param messageName
   *  This represents the uORB message name; This message name should be
   *  globally unique.
   * @return
   *  0 = success; This means the messages is successfully sent to the receiver
   *    Note: This does not necessarily mean that the receiver as received it.
   *  otherwise = failure.
   */
  virtual int16_t remove_subscription( const char * messageName );

  /**
   * Register Message Handler.  This is internal for the IChannel implementer*
   */
  virtual int16_t register_handler( uORBCommunicator::IChannelRxHandler* handler );


  /**
   * @brief Sends the data message over the communication link.
   * @param messageName
   *  This represents the uORB message name; This message name should be
   *  globally unique.
   * @param length
   *  The length of the data buffer to be sent.
   * @param data
   *  The actual data to be sent.
   * @return
   *  0 = success; This means the messages is successfully sent to the receiver
   *    Note: This does not mean that the receiver as received it.
   *  otherwise = failure.
   */
  virtual int16_t send_message( const char * messageName, int32_t length, uint8_t* data);

  uORBCommunicator::IChannelRxHandler* get_rx_handler()
  {
    return _rx_handler;
  }

  bool get_remote_topicA_data( struct orb_topic_A* data );
  bool get_remote_topicB_data( struct orb_topic_B* data );

  void reset_counters();

  InterfaceCounters get_interface_counters( const char * messageName );


 private:
  uORBCommunicator::IChannelRxHandler* _rx_handler;
  //int _sub_topicA_copy_fd;
  //int _sub_topicB_copy_fd;

  std::map<std::string, std::string> _topic_translation_map;

  struct orb_topic_A _topicAData;
  struct orb_topic_B _topicBData;

  std::map<std::string, InterfaceCounters> _msgCounters;
};

#endif /* _uORBCommunicatorMock_test_hpp_ */
