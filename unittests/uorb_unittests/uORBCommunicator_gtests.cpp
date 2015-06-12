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
#include "uORBCommunicatorMockLoopback.hpp"
#include "gtest/gtest.h"
#include "uORB.h"
#include "uORBManager.hpp"
#include "uORBGtestTopics.hpp"
#include "uORBDevices.hpp"
#include "px4_log.h"
#include <errno.h>

#define LOG_TAG "uORBCommunicator_gtests.cpp"

namespace px4
{
  void init_once();
}

namespace uORB_test
{

   class uORBCommunicatorTest : public ::testing::Test
   {
    public:
     uORBCommunicatorTest() :
       _masterDevice( nullptr )
     {
       px4::init_once();

       // create the Master Device and initialize it
       _masterDevice = new uORB::DeviceMaster(uORB::PUBSUB);
       if( _masterDevice != nullptr )
       {
         _masterDevice->init();
       }

       // get the uORB::Manager and set the mock instance
       // explicitly.
       uORB::Manager::get_instance()->set_uorb_communicator( &_comm_channel );
     }
    protected:
     uORB_test::uORBCommunicatorMock _comm_channel;
     struct orb_topic_A _topicA;
     struct orb_topic_B _topicB;
     orb_advert_t _pub_ptr;
     int _sub_fd;
     uORB::DeviceMaster* _masterDevice;
   };

   //================= Unit tests for add_subscription ===================
   // this test will validate if the uORB calls the "add_subscription"
   // method if there is atleast one subscriber in the local system.
   //=====================================================================
   TEST_F( uORBCommunicatorTest, add_subscription_case1 )
   {
     // case 1: add subscription should not be called if there are no
     // internal subscriber and only a publisher.
     // Steps:
     // 0. reset the interface counters.
     // 1. advertize a topic
     // 2. check to see if add_subscription is called.
     //    the count should be zero.

     //step 0.
     _comm_channel.reset_counters();
     uORB_test::uORBCommunicatorMock::InterfaceCounters c = _comm_channel.get_interface_counters( "topicA" );
     ASSERT_EQ( c._add_subscriptionCount, 0 );

     //step 1.
     _topicA.val = 1;
     ASSERT_NE( ( _pub_ptr = orb_advertise(ORB_ID(topicA), &_topicA) ), nullptr ) << "Failed to advertize uORB Topic orb_topic_A: errno: " << errno;
     PX4_INFO( "publist handle: 0x%08lx", (unsigned long)_pub_ptr );

     //step 2.
     c = _comm_channel.get_interface_counters( "topicA" );
     ASSERT_EQ( c._add_subscriptionCount, 0 );
   }

   TEST_F( uORBCommunicatorTest, add_subscription_case2 )
   {
     // case 1: add subscription should not be called if there is atleast one
     // internal subscriber and only a publisher.
     // Steps:
     // 0. reset the interface counters.
     // 1. advertize a topic
     // 2. Add in internal subscriber.
     // 3. check to see if add_subscription is called.
     //    the count should be 1.
     // 4. unsubscribe the topic

     //step 0.
     _comm_channel.reset_counters();
     uORB_test::uORBCommunicatorMock::InterfaceCounters c = _comm_channel.get_interface_counters( "topicA" );
     ASSERT_EQ( c._add_subscriptionCount, 0 );

     //step 1.
     _topicA.val = 1;
     _pub_ptr = orb_advertise(ORB_ID(topicA), &_topicA);
     PX4_INFO( "[uORBCommunicatorTest.add_subscription_case2] orb_advertize(topicA) returncode:(%p)", _pub_ptr );
     ASSERT_TRUE( ( _pub_ptr != nullptr ) ) << "Failed to advertize uORB Topic orb_topic_A: errno: " << errno;
     PX4_INFO( "publist handle: 0x%08lx", (unsigned long)_pub_ptr );

     // step 2
     ASSERT_NE( ( _sub_fd = orb_subscribe(ORB_ID(topicA) ) ) , -1 ) << "Subscribe failed: %d" << errno;
     PX4_INFO( "subscribe fd: %d", _sub_fd );

     //step 3.
     c = _comm_channel.get_interface_counters( "topicA" );
     PX4_INFO( "interface counter for topicA: %d", (int)c._add_subscriptionCount );
     ASSERT_EQ( c._add_subscriptionCount, 1 );

     //step 4.
     PX4_INFO( "Before unsubscribe for Topic A" );
     ASSERT_EQ( orb_unsubscribe( _sub_fd ), OK );
     PX4_INFO( "After unsubscribe for Topic A" );
   }

   //============ unit tests for remove_subscribtion =======
   TEST_F( uORBCommunicatorTest, remove_subscribtion )
   {
     // remove subscription should be called if there after a subscription is removed.
     // Steps:
     // 0. reset the interface counters.
     // 1. advertize a topic
     // 2. Add in internal subscriber.
     // 3. unsubscribe the topic
     // 4. check the remove_subsciption counter it should be 1.

     //step 0.
     _comm_channel.reset_counters();
     uORB_test::uORBCommunicatorMock::InterfaceCounters c = _comm_channel.get_interface_counters( "topicA" );
     ASSERT_EQ( c._remove_subscriptionCount, 0 );

     //step 1.
     // step 1.
     _topicA.val = 1;
     _pub_ptr = orb_advertise(ORB_ID(topicA), &_topicA);
     PX4_INFO( "[uORBCommunicatorTest.remove_subscribtion] orb_advertize(topicA) returncode:(%p)", _pub_ptr );
     ASSERT_TRUE( ( _pub_ptr != nullptr ) ) << "Failed to advertize uORB Topic orb_topic_A: errno: " << errno;
     PX4_INFO( "publist handle: 0x%08lx", (unsigned long)_pub_ptr );

     c = _comm_channel.get_interface_counters( "topicA" );
     ASSERT_EQ( c._remove_subscriptionCount, 0 );


     // step 2
     ASSERT_NE( ( _sub_fd = orb_subscribe(ORB_ID(topicA) )), -1  ); // << "Subscribe failed: " << _sub_fd << "errono: "  << errno;
     PX4_INFO( "subscribe fd: %d", _sub_fd );

     c = _comm_channel.get_interface_counters( "topicA" );
     ASSERT_EQ( c._remove_subscriptionCount, 0 );


     //step 3.
     ASSERT_EQ( orb_unsubscribe( _sub_fd ), OK );

     //step 4.
     c = _comm_channel.get_interface_counters( "topicA" );
     ASSERT_EQ( c._remove_subscriptionCount, 1 );
   }

   //============ unit tests for send_message =======
   TEST_F(uORBCommunicatorTest, send_message_case1 )
   {
     // Case1: send message should not be called when a topic is advertized
     // and there are no remote remote subscribers.
     // Steps:
     // 0. reset the interface counters.
     // 1. advertize a topic
     // 2. check to see that the send message counter is 0.
     //step 0.
     _comm_channel.reset_counters();
     uORB_test::uORBCommunicatorMock::InterfaceCounters c = _comm_channel.get_interface_counters( "topicA_sndmsg" );
     ASSERT_EQ( c._send_messageCount, 0 );

     //step 1.
     ORB_DEFINE( topicA_sndmsg, struct orb_topic_A );
     _topicA.val = 1;
     _pub_ptr = orb_advertise(ORB_ID(topicA_sndmsg ), &_topicA );
     ASSERT_TRUE( ( _pub_ptr != nullptr ) ) << "Failed to advertize uORB Topic topicA_sndmsg: errno: " << errno;
     PX4_INFO( "publist handle: 0x%08lx", (unsigned long)_pub_ptr );

     c = _comm_channel.get_interface_counters( "topicA_sndmsg" );
     ASSERT_EQ( c._send_messageCount, 0 );
   }

   TEST_F(uORBCommunicatorTest, send_message_case2 )
   {
     // Case2: send message should be called when a topic is advertized
     // and there are remote remote subscribers.
     // Steps:
     // 0. reset the interface counters.
     // 1. Add a remote subscriber by calling the "process_add_subscription"
     // 2. advertize a topic
     // 3. check to see that the send message counter is 1 and check the value.
     // 4. publish new data.
     // 5. check to see that send_msg is incremented by 1 and check the value.
     // 6. remove remote subscriber by calling the "process_remove_subscription"
     // 7. publish new data.
     // 8. check to see the sndmessage counter, it should not increment.

     //step 0.
     _comm_channel.reset_counters();
     uORB_test::uORBCommunicatorMock::InterfaceCounters c = _comm_channel.get_interface_counters( "topicB" );
     ASSERT_EQ( c._send_messageCount, 0 );

     //step 1.
     uORBCommunicator::IChannelRxHandler* rx_handler = _comm_channel.get_rx_handler();
     // add a local subsciber to avoid the issue of creating a node for the first time
     // remote.
     ASSERT_NE( ( _sub_fd = orb_subscribe(ORB_ID(topicB) ) ), -1 ) << "Subscribe failed for topicB: %d" << errno;
     PX4_INFO( "subscribe fd: %d", _sub_fd );
     ASSERT_EQ( rx_handler->process_add_subscription( "topicB", 1 ), 0 );
     c = _comm_channel.get_interface_counters( "topicB" );
     ASSERT_EQ( c._send_messageCount, 0 );

     // step 2.
     _topicB.val = 1;
     _pub_ptr = orb_advertise(ORB_ID(topicB), &_topicB);
     PX4_INFO( "publist handle: 0x%08lx", (unsigned long)_pub_ptr );
     ASSERT_TRUE( _pub_ptr != nullptr ) << "Failed to advertize uORB Topic topicB: errno: " << errno;

     //step 3.
     c = _comm_channel.get_interface_counters( "topicB" );
     ASSERT_EQ( c._send_messageCount, 1 );

     struct orb_topic_B test_val;
     _comm_channel.get_remote_topicB_data( &test_val );
     ASSERT_EQ( test_val.val, 1 );

     //step 4. publish new data.
     _topicB.val = 2;
     ASSERT_EQ( orb_publish( ORB_ID(topicB), _pub_ptr, &_topicB), OK );

     //step 5.
     c = _comm_channel.get_interface_counters( "topicB" );
     ASSERT_EQ( c._send_messageCount, 2 );

     _comm_channel.get_remote_topicB_data( &test_val );
     ASSERT_EQ( test_val.val, 2 );

     // step 6.
     ASSERT_EQ( rx_handler->process_remove_subscription( "topicB" ), 0 );
     ASSERT_EQ( orb_unsubscribe( _sub_fd ), OK );

     //step 7. publish new data.
     _topicB.val = 5;
     ASSERT_EQ( orb_publish( ORB_ID(topicB), _pub_ptr, &_topicB), OK );

     //step 8.
     c = _comm_channel.get_interface_counters( "topicB" );
     ASSERT_EQ( c._send_messageCount, 2 );

     _comm_channel.get_remote_topicB_data( &test_val );
     ASSERT_EQ( test_val.val, 2 );

   }

   //========== UNIT tests to verify the process_receive_message interface
   //========== of rx handler.
   TEST_F( uORBCommunicatorTest, rx_handler_rcv_data_case1 )
   {
     // this will mimic the process of calling the process_receive_message
     // from remote and verify that the local subscribers got it
     // are the steps.
     // 1. clear the counters.
     // 2. advertize a topic
     // 3. add a local subscriber.
     // 4. call process_receive_message.
     // 5. check to see that the subscriber got the data and the message is not sent back
     //    by looking at the counter for snd message.

     //step 1.
     _comm_channel.reset_counters();
     uORB_test::uORBCommunicatorMock::InterfaceCounters c = _comm_channel.get_interface_counters( "topicB" );
     ASSERT_EQ( c._send_messageCount, 0 );

     // step 2.
     _topicB.val = 1;
     _pub_ptr = orb_advertise(ORB_ID(topicB), &_topicB);
     ASSERT_TRUE( _pub_ptr != nullptr  ) << "Failed to advertize uORB Topic topicB: errno: " << errno;
     PX4_INFO( "publist handle: 0x%08lx", (unsigned long)_pub_ptr );

     // step 3.
     ASSERT_NE( ( _sub_fd = orb_subscribe(ORB_ID(topicB)) ), -1 ) << "Subscribe failed for topicB: %d" << errno;
     PX4_INFO( "subscribe fd: %d", _sub_fd );

     //step 4.
     uORBCommunicator::IChannelRxHandler* rx_handler = _comm_channel.get_rx_handler();
     struct orb_topic_B testVal;
     testVal.val = 10;
     ASSERT_EQ( rx_handler->process_received_message( "topicB", sizeof( testVal ), (uint8_t*)(&testVal) ), 0 );

     // step 5. check the values.
     struct orb_topic_B receivedVal;
     ASSERT_EQ( orb_copy(ORB_ID(topicB), _sub_fd, &receivedVal), OK ) << "copy(1) failed: " << errno;
     ASSERT_EQ( testVal.val, receivedVal.val )
       << "copy(1) mismatch: " << testVal.val
       << " expected " << receivedVal.val;

     c = _comm_channel.get_interface_counters( "topicB" );
     ASSERT_EQ( c._send_messageCount, 0 );

     // cleanup.
     ASSERT_EQ( orb_unsubscribe( _sub_fd ), OK );
   }

   TEST_F( uORBCommunicatorTest, rx_handler_rcv_data_case2 )
   {
     // this will mimic the process of calling the process_receive_message
     // from remote and verify that the local subscribers got it
     // and also the message is  send back to the remote.  The following
     // are the steps.
     // 1. clear the counters.
     // 2. advertize a topic
     // 3. add a local subscriber.
     // 4. add remote subscriber by calling the "process_add_subscription.
     // 5. call process_receive_message.
     // 6. check to see that the subscriber got the data and the message is not sent back
     //    by looking at the counter for snd message.

     //step 1.
     _comm_channel.reset_counters();
     uORB_test::uORBCommunicatorMock::InterfaceCounters c = _comm_channel.get_interface_counters( "topicB" );
     ASSERT_EQ( c._send_messageCount, 0 );

     // step 2.
     _topicB.val = 1;
     _pub_ptr = orb_advertise(ORB_ID(topicB), &_topicB);
     ASSERT_TRUE( _pub_ptr != nullptr ) << "Failed to advertize uORB Topic topicB: errno: " << errno;
     PX4_INFO( "publist handle: 0x%08lx", (unsigned long)_pub_ptr );

     // step 3.
     ASSERT_NE( ( _sub_fd = orb_subscribe(ORB_ID(topicB)) ), -1 ) << "Subscribe failed for topicB: %d" << errno;
     PX4_INFO( "subscribe fd: %d", _sub_fd );

     //step 4.
     uORBCommunicator::IChannelRxHandler* rx_handler = _comm_channel.get_rx_handler();
     ASSERT_EQ( rx_handler->process_add_subscription( "topicB", 1 ), 0 );
     c = _comm_channel.get_interface_counters( "topicB" );
     ASSERT_EQ( c._send_messageCount, 1 );

     //step 5.
     struct orb_topic_B testVal;
     testVal.val = 15;
     ASSERT_EQ( rx_handler->process_received_message( "topicB", sizeof( testVal ), (uint8_t*)(&testVal) ), 0 );
     c = _comm_channel.get_interface_counters( "topicB" );
     ASSERT_EQ( c._send_messageCount, 1 );


     // step 6. check the values.
     struct orb_topic_B receivedVal;
     ASSERT_EQ( orb_copy(ORB_ID(topicB), _sub_fd, &receivedVal), OK ) << "copy(1) failed: " << errno;
     ASSERT_EQ( testVal.val, receivedVal.val )
       << "copy(1) mismatch: " << testVal.val
       << " expected " << receivedVal.val;

     c = _comm_channel.get_interface_counters( "topicB" );
     ASSERT_EQ( c._send_messageCount, 1 );

     // cleanup.
     ASSERT_EQ( orb_unsubscribe( _sub_fd ), OK );
   }

   TEST_F( uORBCommunicatorTest, Loopback )
   {
     // create the loopback instance.
     uORB_test::uORBCommunicatorMockLoopback _comm_channel_loopback;

     //intiailize the uorb to remove the node map entries;
     //orb_initialize();

     //set the communucation channel interface.
     uORB::Manager::get_instance()->set_uorb_communicator( &_comm_channel_loopback );

     // now for the actual test.
     orb_advert_t pub_topicA_ptr;
     int sub_topicA_fd;
     int sub_topicAClone_fd;

     struct orb_topic_A topicA;
     struct orb_topic_A topicAClone;
     struct orb_topic_A topicALocal;

     // step 1.
     topicA.val = 10;
     pub_topicA_ptr = orb_advertise(ORB_ID(topicA), &topicA);
     PX4_INFO( "[uORBCommunicatorTest.Loopback]orb_advertize(topicA) return code:(%p)", pub_topicA_ptr );
     ASSERT_TRUE( pub_topicA_ptr != nullptr ) << "Failed to advertize uORB Topic orb_topic_A: errno: " << errno;
     PX4_INFO( "publist handle: 0x%08lx", (unsigned long)pub_topicA_ptr );

     // step 2.
     ASSERT_NE( ( sub_topicA_fd = orb_subscribe(ORB_ID(topicA)) ), -1 ) << "Subscribe failed: %d" << errno;
     PX4_INFO( "subscribe fd: %d", sub_topicA_fd );

     // step 3. check to see that the subscriber got a value of 10.
     ASSERT_EQ( orb_copy(ORB_ID(topicA), sub_topicA_fd, &topicALocal), OK ) << "copy(1) failed: " << errno;
     ASSERT_EQ( topicA.val, topicALocal.val )
       << "copy(1) mismatch: " << topicA.val
       << " expected " << topicALocal.val;

     // subscribe a remote subscriber.
     ASSERT_NE( ( sub_topicAClone_fd = orb_subscribe(ORB_ID(topicA_clone)) ), -1 ) << "Subscribe failed: %d" << errno;
     PX4_INFO( "subscribe fd: %d", sub_topicAClone_fd );

     ASSERT_EQ( orb_copy(ORB_ID(topicA_clone), sub_topicAClone_fd, &topicAClone), OK ) << "copy(1) failed: " << errno;
     ASSERT_EQ( topicA.val, topicAClone.val )
       << "copy(1) mismatch: " << topicA.val
       << " expected " << topicAClone.val;

     // publish a new data and check to see that the data is received on the remote.
     topicA.val = 15;
     ASSERT_EQ( orb_publish( ORB_ID(topicA), pub_topicA_ptr, &topicA), OK );

     // check to see that the subscriber got a new value.
     ASSERT_EQ( orb_copy(ORB_ID(topicA), sub_topicA_fd, &topicALocal), OK ) << "copy(1) failed: " << errno;
     ASSERT_EQ( topicA.val, topicALocal.val )
       << "copy(1) mismatch: " << topicA.val
       << " expected " << topicALocal.val;

     ASSERT_EQ( orb_copy(ORB_ID(topicA_clone), sub_topicAClone_fd, &topicAClone), OK ) << "copy(1) failed: " << errno;
     ASSERT_EQ( topicA.val, topicAClone.val )
       << "copy(1) mismatch: " << topicA.val
       << " expected " << topicAClone.val;

     // remove the remote subscriber and make sure that the data is not received,
     ASSERT_EQ( orb_unsubscribe( sub_topicAClone_fd ), OK );

     // publish a new data and check to see that the data is received on local this should not crash.
     topicA.val = 20;
     ASSERT_EQ( orb_publish( ORB_ID(topicA), pub_topicA_ptr, &topicA), OK );

     // check to see that the subscriber got a new value.
     ASSERT_EQ( orb_copy(ORB_ID(topicA), sub_topicA_fd, &topicALocal), OK ) << "copy(1) failed: " << errno;
     ASSERT_EQ( topicA.val, topicALocal.val )
       << "copy(1) mismatch: " << topicA.val
       << " expected " << topicALocal.val;

     //remove the local subscriber as well and publish new data; system should not crash.
     ASSERT_EQ( orb_unsubscribe( sub_topicA_fd ), OK );

     // publish a new data; this should not crash.
     topicA.val = 25;
     ASSERT_EQ( orb_publish( ORB_ID(topicA), pub_topicA_ptr, &topicA), OK );

   }
}




