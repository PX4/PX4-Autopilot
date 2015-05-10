/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_EVENT_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_EVENT_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/protocol/dynamic_node_id_server/distributed/types.hpp>

namespace uavcan
{
namespace dynamic_node_id_server
{
/**
 * @ref IEventTracer.
 * Event codes cannot be changed, only new ones can be added.
 */
enum TraceCode
{
    // Event name                          Argument
    // 0
    TraceError,                         // error code (may be negated)
    TraceLogLastIndexRestored,          // recovered last index value
    TraceLogAppend,                     // index of new entry
    TraceLogRemove,                     // new last index value
    TraceCurrentTermRestored,           // current term
    // 5
    TraceCurrentTermUpdate,             // current term
    TraceVotedForRestored,              // value of votedFor
    TraceVotedForUpdate,                // value of votedFor
    TraceDiscoveryBroadcast,            // number of known servers
    TraceNewServerDiscovered,           // node ID of the new server
    // 10
    TraceDiscoveryReceived,             // node ID of the sender
    TraceClusterSizeInited,             // cluster size
    TraceInvalidClusterSizeReceived,    // received cluster size
    TraceRaftCoreInited,                // update interval in usec
    TraceRaftStateSwitch,               // 0 - Follower, 1 - Candidate, 2 - Leader
    // 15
    TraceRaftActiveSwitch,              // 0 - Passive, 1 - Active
    TraceRaftNewLogEntry,               // node ID value
    TraceRaftRequestIgnored,            // node ID of the client
    TraceRaftVoteRequestReceived,       // node ID of the client
    TraceRaftVoteRequestSucceeded,      // node ID of the server
    // 20
    TraceRaftVoteRequestInitiation,     // node ID of the server
    TraceRaftPersistStateUpdateError,   // negative error code
    TraceRaftCommitIndexUpdate,         // new commit index value
    TraceRaftNewerTermInResponse,       // new term value
    TraceRaftNewEntryCommitted,         // new commit index value
    // 25
    TraceRaftAppendEntriesCallFailure,  // error code (may be negated)

    NumTraceCodes
};

/**
 * This interface allows the application to trace events that happen in the server.
 */
class IEventTracer
{
public:
#if UAVCAN_TOSTRING
    /**
     * It is safe to call this function with any argument.
     * If the event code is out of range, an assertion failure will be triggered and an error text will be returned.
     */
    static const char* getEventName(TraceCode code)
    {
        // import re
        // make_strings = lambda s: ',\n'.join('"%s"' % x for x in re.findall(r'\ \ \ \ Trace([A-Za-z0-9]+),', s))
        static const char* const Strings[NumTraceCodes] =
        {
            "Error",
            "LogLastIndexRestored",
            "LogAppend",
            "LogRemove",
            "CurrentTermRestored",
            "CurrentTermUpdate",
            "VotedForRestored",
            "VotedForUpdate",
            "DiscoveryBroadcast",
            "NewServerDiscovered",
            "DiscoveryReceived",
            "ClusterSizeInited",
            "InvalidClusterSizeReceived",
            "RaftCoreInited",
            "RaftStateSwitch",
            "RaftActiveSwitch",
            "RaftNewLogEntry",
            "RaftRequestIgnored",
            "RaftVoteRequestReceived",
            "RaftVoteRequestSucceeded",
            "RaftVoteRequestInitiation",
            "RaftPersistStateUpdateError",
            "RaftCommitIndexUpdate",
            "RaftNewerTermInResponse",
            "RaftNewEntryCommitted",
            "RaftAppendEntriesCallFailure"
        };
        uavcan::StaticAssert<sizeof(Strings) / sizeof(Strings[0]) == NumTraceCodes>::check();
        UAVCAN_ASSERT(code < NumTraceCodes);
        return (code < NumTraceCodes) ? Strings[static_cast<unsigned>(code)] : "INVALID_EVENT_CODE";
    }
#endif

    /**
     * The server invokes this method every time it believes that a noteworthy event has happened.
     * It is guaranteed that event code values will never change, but new ones can be added in future. This ensures
     * full backward compatibility.
     * @param event_code        Event code, see the sources for the enum with values.
     * @param event_argument    Value associated with the event; its meaning depends on the event code.
     */
    virtual void onEvent(TraceCode event_code, int64_t event_argument) = 0;

    virtual ~IEventTracer() { }
};

}
}

#endif // Include guard
