//=============================================================================
// File: px4muorb_KraitRpcWrapper.hpp
//
//  @@-COPYRIGHT-START-@@
//
// Copyright 2015 Qualcomm Technologies, Inc. All rights reserved.
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
#ifndef _px4muorb_KraitRpcWrapper_hpp_
#define _px4muorb_KraitRpcWrapper_hpp_
#include <stdint.h>

namespace px4muorb
{
   class KraitRpcWrapper;
}

class px4muorb::KraitRpcWrapper
{
public:
   /**
    * Constructor
    */
   KraitRpcWrapper();
   
   /**
    * destructor
    */
   ~KraitRpcWrapper();

   /**
    * Initiatizes the rpc channel px4 muorb
    */
   bool Initialize();

   /**
    * Terminate to clean up the resources.  This should be called at program exit
    */
   bool Terminate();

   /**
    * Muorb related functions to pub/sub of orb topic from krait to adsp
    */
   int32_t AddSubscriber( const char* topic );
   int32_t RemoveSubscriber( const char* topic );
   int32_t SendData( const char* topic, int32_t length_in_bytes, const uint8_t* data );
   int32_t ReceiveData( int32_t* msg_type, char** topic, int32_t* length_in_bytes, uint8_t** data );
   int32_t IsSubscriberPresent( const char* topic, int32_t* status );
   int32_t ReceiveBulkData( uint8_t** bulk_data, int32_t* length_in_bytes, int32_t* topic_count );
   int32_t UnblockReceiveData();
};
#endif // _px4muorb_KraitWrapper_hpp_
