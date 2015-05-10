/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/dynamic_node_id_server/event.hpp>


TEST(DynamicNodeIDServer, EventCodeToString)
{
    using namespace uavcan::dynamic_node_id_server::distributed;
    using namespace uavcan::dynamic_node_id_server;

    // Simply checking some error codes
    ASSERT_STREQ("Error",                        IEventTracer::getEventName(TraceError));
    ASSERT_STREQ("RaftActiveSwitch",             IEventTracer::getEventName(TraceRaftActiveSwitch));
    ASSERT_STREQ("RaftAppendEntriesCallFailure", IEventTracer::getEventName(TraceRaftAppendEntriesCallFailure));
    ASSERT_STREQ("DiscoveryReceived",            IEventTracer::getEventName(TraceDiscoveryReceived));
}
