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

#ifndef _uORBGtestTopics_hpp_
#define _uORBGtestTopics_hpp_

#include "uORB/uORB.h"

namespace uORB_test
{
   struct orb_topic_A
   {
     int16_t val;
   };

   struct orb_topic_B
   {
     int16_t val;
   };


   ORB_DEFINE( topicA, struct orb_topic_A );
   ORB_DEFINE( topicB, struct orb_topic_B );

   ORB_DEFINE( topicA_clone, struct orb_topic_A );
   ORB_DEFINE( topicB_clone, struct orb_topic_B );
}

#endif // _UnitTest_uORBTopics_hpp_
