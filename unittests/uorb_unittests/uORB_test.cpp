//=============================================================================
// File: uORB_test.cpp
//
//  @@-COPYRIGHT-START-@@
//
// Copyright 2014 Qualcomm Technologies, Inc. All rights reserved.
// Confidential & Proprietary - Qualcomm Technologies, Inc. ("QTI")
//
// The party receiving this software directly from QTI (the "Recipient")
// may use this software as reasonably necessary solely for the purposes
// set forth in the agreement between the Recipient and QTI (the
// "Agreement"). The software may be used in source code form solely by
// the Recipient's employees (if any) authorized by the Agreement. Unless
// expressly authorized in the Agreement, the Recipient may not sublicense,
// assign, transfer or otherwise provide the source code to any third
// party. Qualcomm Technologies, Inc. retains all ownership rights in and
// to the software
//
// This notice supersedes any other QTI notices contained within the software
// except copyright notices indicating different years of publication for
// different portions of the software. This notice does not supersede the
// application of any third party copyright notice to that third party's
// code.
//
//  @@-COPYRIGHT-END-@@
//
//=============================================================================

#include <unistd.h>
#include <stdarg.h>
#include <assert.h>
#include <string>
#include <palSemaphore.hpp>
#include <palThread.hpp>
#include <errno.h>
#include "uORB/uORB.h"
#include "utils/general.h"
#include "utils/logging.h"

#include <gtest/gtest.h>

#define LOG_TAG "uORB_test.cpp"

//#ifdef INCLUDE_ANDROID_UNIT_TEST_UORB
namespace uORB_test
{

   struct orb_test
   {
      int val;
   };
   ORB_DEFINE(orb_test, struct orb_test);

   pal::Semaphore sub_semaphore;
   static int pfd, sfd;
   static struct orb_test t;
   bool threadTestPassed = false;

   TEST( uORBTest, pub_sub_initialization )
   {
     struct orb_test u;
     bool updated;

     t.val = 0;
     ASSERT_NE( ( pfd = orb_advertise(ORB_ID(orb_test), &t) ), 0 ) << "Failed to advertize uORB Topic orb_test: errno: " << errno;
     IPRINTF( "publist handle: 0x%08x", pfd );


     ASSERT_NE( ( sfd = orb_subscribe(ORB_ID(orb_test), (void *)&sub_semaphore) ), 0 ) << "Subscribe failed: %d" << errno;
     IPRINTF( "subscribe fd: %d", sfd );

     u.val = 1;

     ASSERT_EQ( orb_copy(ORB_ID(orb_test), sfd, &u), OK ) << "copy(1) failed: " << errno;

     ASSERT_EQ( u.val, t.val ) << "copy(1) mismatch: " << u.val << " expected " << t.val;

     ASSERT_EQ(orb_check(sfd, &updated), OK ) << "check(1) failed";

     ASSERT_FALSE( updated ) << "spurious updated flag";

     ASSERT_EQ(orb_unsubscribe(sfd), OK );
     close(pfd);
   }


   void test_subscriber_thread(void *this_ptr)
   {
      struct orb_test u;
      bool updated;
      pal::Semaphore *sub_semaphore = (pal::Semaphore *)this_ptr;
      assert(sub_semaphore != nullptr);

      IPRINTF( "waiting for new subscriber data");
      sub_semaphore->wait();

      if (OK != orb_check(sfd, &updated))
      {
         EPRINTF("check(2) failed");
         return;
      }

      if (!updated)
      {
         EPRINTF("missing updated flag");
         return;
      }

      if (OK != orb_copy(ORB_ID(orb_test), sfd, &u))
      {
         EPRINTF("copy(2) failed: %d", errno);
         return;
      }

      if (u.val != t.val)
      {
         EPRINTF("copy(2) mismatch: %d expected %d", u.val, t.val);
         return;
      }
      threadTestPassed = true;
   }


   TEST( uORB, pub_sub_withThread )
   {
     pal::Thread sub_thread;

     threadTestPassed = false;

     // advertize the topic first if it is not created.
     t.val = 1;
     ASSERT_NE( (pfd = orb_advertise(ORB_ID(orb_test), &t) ), 0 ) << "Failed to advertize uORB Topic orb_test: errno: " << errno;
     IPRINTF( "publist handle: 0x%08x", pfd );

     ASSERT_NE( ( sfd = orb_subscribe(ORB_ID(orb_test), (void *)&sub_semaphore) ), 0 ) << "Subscribe failed: %d" << errno;
     IPRINTF( "subscribe fd: %d", sfd );

     sub_thread.create(test_subscriber_thread, (void *)&sub_semaphore);

     t.val = 2;
     IPRINTF("try publish, creating new thread to await the results");

     ASSERT_EQ(orb_publish(ORB_ID(orb_test), pfd, &t), OK) << "publish failed";

     ASSERT_EQ( t.val, 2 );

     IPRINTF("waiting for the subscriber thread to exit");
     sub_thread.wait();

     ASSERT_TRUE( threadTestPassed );

     ASSERT_EQ(orb_unsubscribe(sfd), OK );
     close(pfd);
   }



#if 0
      /* this is a hacky test that exploits the sensors app to test rate-limiting */

      sfd = orb_subscribe(ORB_ID(sensor_combined));

      hrt_abstime start, end;
      unsigned count;

      start = hrt_absolute_time();
      count = 0;

      do
      {
         orb_check(sfd, &updated);

         if (updated)
         {
            orb_copy(ORB_ID(sensor_combined), sfd, nullptr);
            count++;
         }
      }while (count < 100);

      end = hrt_absolute_time();
      test_note("full-speed, 100 updates in %llu", end - start);

      orb_set_interval(sfd, 10);

      start = hrt_absolute_time();
      count = 0;

      do
      {
         orb_check(sfd, &updated);

         if (updated)
         {
            orb_copy(ORB_ID(sensor_combined), sfd, nullptr);
            count++;
         }
      }while (count < 100);

      end = hrt_absolute_time();
      test_note("100Hz, 100 updates in %llu", end - start);

      orb_unsubscribe(sfd);
#endif

}

