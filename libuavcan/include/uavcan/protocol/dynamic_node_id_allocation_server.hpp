/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SERVER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SERVER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/node/subscriber.hpp>
#include <uavcan/node/publisher.hpp>
#include <uavcan/node/service_server.hpp>
#include <uavcan/node/service_client.hpp>
#include <uavcan/node/timer.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/util/lazy_constructor.hpp>
#include <uavcan/util/map.hpp>
// Types used by the server
#include <uavcan/protocol/dynamic_node_id/server/AppendEntries.hpp>
#include <uavcan/protocol/dynamic_node_id/server/RequestVote.hpp>
#include <uavcan/protocol/dynamic_node_id/server/Entry.hpp>
#include <uavcan/protocol/dynamic_node_id/server/Discovery.hpp>
#include <uavcan/protocol/dynamic_node_id/Allocation.hpp>
#include <uavcan/protocol/GetNodeInfo.hpp>
#include <uavcan/protocol/NodeStatus.hpp>

namespace uavcan
{
/**
 * This interface is used by the server to read and write stable storage.
 * The storage is represented as a key-value container, where keys and values are ASCII strings up to 32
 * characters long, not including the termination byte. Fixed block size allows for absolutely straightforward
 * and efficient implementation of storage backends, e.g. based on text files.
 * Keys and values may contain only non-whitespace, non-formatting printable characters.
 */
class IDynamicNodeIDStorageBackend
{
public:
    /**
     * Maximum length of keys and values. One pair takes twice as much space.
     */
    enum { MaxStringLength = 32 };

    /**
     * It is guaranteed that the server will never require more than this number of key/value pairs.
     * Total storage space needed is (MaxKeyValuePairs * MaxStringLength * 2), not including storage overhead.
     */
    enum { MaxKeyValuePairs = 400 };

    /**
     * This type is used to exchange data chunks with the backend.
     * It doesn't use any dynamic memory; please refer to the Array<> class for details.
     */
    typedef Array<IntegerSpec<8, SignednessUnsigned, CastModeTruncate>, ArrayModeDynamic, MaxStringLength> String;

    /**
     * Read one value from the storage.
     * If such key does not exist, or if read failed, an empty string will be returned.
     * This method should not block for more than 50 ms.
     */
    virtual String get(const String& key) const = 0;

    /**
     * Create or update value for the given key. Empty value should be regarded as a request to delete the key.
     * This method should not block for more than 50 ms.
     * Failures will be ignored.
     */
    virtual void set(const String& key, const String& value) = 0;

    virtual ~IDynamicNodeIDStorageBackend() { }
};

/**
 * This interface allows the application to trace events that happen in the server.
 */
class IDynamicNodeIDAllocationServerEventTracer
{
public:
#if UAVCAN_TOSTRING
    /**
     * It is safe to call this function with any argument.
     * If the event code is out of range, an assertion failure will be triggered and an error text will be returned.
     */
    static const char* getEventName(uint16_t code);
#endif

    /**
     * The server invokes this method every time it believes that a noteworthy event has happened.
     * The table of event codes can be found in the server sources.
     * It is guaranteed that event code values will never change, but new ones can be added in future. This ensures
     * full backward compatibility.
     * @param event_code        Event code, see the sources for the enum with values.
     * @param event_argument    Value associated with the event; its meaning depends on the event code.
     */
    virtual void onEvent(uint16_t event_code, int64_t event_argument) = 0;

    virtual ~IDynamicNodeIDAllocationServerEventTracer() { }
};

/**
 * Internals, do not use anything from this namespace directly.
 */
namespace dynamic_node_id_server_impl
{

using namespace protocol::dynamic_node_id::server;

/**
 * Raft term
 */
typedef StorageType<Entry::FieldTypes::term>::Type Term;

/**
 * @ref IDynamicNodeIDAllocationServerEventTracer.
 * Event codes cannot be changed, only new ones can be added.
 */
enum TraceEvent
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

    NumTraceEventCodes
};

/**
 * This class extends the storage backend interface with serialization/deserialization functionality.
 */
class MarshallingStorageDecorator
{
    IDynamicNodeIDStorageBackend& storage_;

    static uint8_t convertLowerCaseHexCharToNibble(char ch);

public:
    MarshallingStorageDecorator(IDynamicNodeIDStorageBackend& storage)
        : storage_(storage)
    {
        // Making sure that there will be no data loss during serialization.
        StaticAssert<(sizeof(Term) <= sizeof(uint32_t))>::check();
    }

    /**
     * These methods set the value and then immediately read it back.
     *  1. Serialize the value.
     *  2. Update the value on the backend.
     *  3. Call get() with the same value argument.
     * The caller then is supposed to check whether the argument has the desired value.
     */
    int setAndGetBack(const IDynamicNodeIDStorageBackend::String& key, uint32_t& inout_value);
    int setAndGetBack(const IDynamicNodeIDStorageBackend::String& key,
                      Entry::FieldTypes::unique_id& inout_value);

    /**
     * Getters simply read and deserialize the value.
     *  1. Read the value back from the backend; return false if read fails.
     *  2. Deserealize the newly read value; return false if deserialization fails.
     *  3. Update the argument with deserialized value.
     *  4. Return true.
     */
    int get(const IDynamicNodeIDStorageBackend::String& key, uint32_t& out_value) const;
    int get(const IDynamicNodeIDStorageBackend::String& key,
            Entry::FieldTypes::unique_id& out_value) const;
};

/**
 * Raft log.
 * This class transparently replicates its state to the storage backend, keeping the most recent state in memory.
 * Writes are slow, reads are instantaneous.
 */
class Log
{
public:
    typedef uint8_t Index;

    enum { Capacity = NodeID::Max + 1 };

private:
    IDynamicNodeIDStorageBackend& storage_;
    IDynamicNodeIDAllocationServerEventTracer& tracer_;
    Entry entries_[Capacity];
    Index last_index_;             // Index zero always contains an empty entry

    static IDynamicNodeIDStorageBackend::String getLastIndexKey() { return "log_last_index"; }
    static IDynamicNodeIDStorageBackend::String makeEntryKey(Index index, const char* postfix);

    int readEntryFromStorage(Index index, Entry& out_entry);
    int writeEntryToStorage(Index index, const Entry& entry);

    int initEmptyLogStorage();

public:
    Log(IDynamicNodeIDStorageBackend& storage, IDynamicNodeIDAllocationServerEventTracer& tracer)
        : storage_(storage)
        , tracer_(tracer)
        , last_index_(0)
    { }

    /**
     * Initialization is performed as follows (every step may fail and return an error):
     *  1. Log is restored or initialized.
     *  2. Current term is restored. If there was no current term stored and the log is empty, it will be initialized
     *     with zero.
     *  3. VotedFor value is restored. If there was no VotedFor value stored, the log is empty, and the current term is
     *     zero, the value will be initialized with zero.
     */
    int init();

    /**
     * This method invokes storage IO.
     * Returned value indicates whether the entry was successfully appended.
     */
    int append(const Entry& entry);

    /**
     * This method invokes storage IO.
     * Returned value indicates whether the requested operation has been carried out successfully.
     */
    int removeEntriesWhereIndexGreaterOrEqual(Index index);
    int removeEntriesWhereIndexGreater(Index index);

    /**
     * Returns nullptr if there's no such index.
     * This method does not use storage IO.
     */
    const Entry* getEntryAtIndex(Index index) const;

    Index getLastIndex() const { return last_index_; }

    bool isOtherLogUpToDate(Index other_last_index, Term other_last_term) const;
};


/**
 * This class is a convenient container for persistent state variables defined by Raft.
 * Writes are slow, reads are instantaneous.
 */
class PersistentState
{
    IDynamicNodeIDStorageBackend& storage_;
    IDynamicNodeIDAllocationServerEventTracer& tracer_;

    Term current_term_;
    NodeID voted_for_;
    Log log_;

    static IDynamicNodeIDStorageBackend::String getCurrentTermKey() { return "current_term"; }
    static IDynamicNodeIDStorageBackend::String getVotedForKey() { return "voted_for"; }

public:
    PersistentState(IDynamicNodeIDStorageBackend& storage, IDynamicNodeIDAllocationServerEventTracer& tracer)
        : storage_(storage)
        , tracer_(tracer)
        , current_term_(0)
        , log_(storage, tracer)
    { }

    int init();

    Term getCurrentTerm() const { return current_term_; }

    NodeID getVotedFor() const { return voted_for_; }
    bool isVotedForSet() const { return voted_for_.isUnicast(); }

    Log& getLog()             { return log_; }
    const Log& getLog() const { return log_; }

    /**
     * Invokes storage IO.
     */
    int setCurrentTerm(Term term);

    /**
     * Invokes storage IO.
     */
    int setVotedFor(NodeID node_id);
    int resetVotedFor() { return setVotedFor(NodeID(0)); }
};

/**
 * This class maintains the cluster state.
 */
class ClusterManager : private TimerBase
{
public:
    enum { MaxClusterSize = Discovery::FieldTypes::known_nodes::MaxSize };

private:
    typedef MethodBinder<ClusterManager*,
                         void (ClusterManager::*)
                             (const ReceivedDataStructure<Discovery>&)>
        DiscoveryCallback;

    struct Server
    {
        NodeID node_id;
        Log::Index next_index;
        Log::Index match_index;

        Server()
            : next_index(0)
            , match_index(0)
        { }

        void resetIndices(const Log& log);
    };

    IDynamicNodeIDStorageBackend& storage_;
    IDynamicNodeIDAllocationServerEventTracer& tracer_;
    const Log& log_;

    Subscriber<Discovery, DiscoveryCallback> discovery_sub_;
    mutable Publisher<Discovery> discovery_pub_;

    Server servers_[MaxClusterSize - 1];   ///< Minus one because the local server is not listed there.

    uint8_t cluster_size_;
    uint8_t num_known_servers_;

    bool had_discovery_activity_;

    static IDynamicNodeIDStorageBackend::String getStorageKeyForClusterSize() { return "cluster_size"; }

    INode&       getNode()       { return discovery_sub_.getNode(); }
    const INode& getNode() const { return discovery_sub_.getNode(); }

    Server*       findServer(NodeID node_id);
    const Server* findServer(NodeID node_id) const;
    void addServer(NodeID node_id);

    virtual void handleTimerEvent(const TimerEvent&);

    void handleDiscovery(const ReceivedDataStructure<Discovery>& msg);

    void startDiscoveryPublishingTimerIfNotRunning();

public:
    enum { ClusterSizeUnknown = 0 };

    /**
     * @param node          Needed to publish and subscribe to Discovery message
     * @param storage       Needed to read the cluster size parameter from the storage
     * @param log           Needed to initialize nextIndex[] values after elections
     */
    ClusterManager(INode& node, IDynamicNodeIDStorageBackend& storage, const Log& log,
                   IDynamicNodeIDAllocationServerEventTracer& tracer)
        : TimerBase(node)
        , storage_(storage)
        , tracer_(tracer)
        , log_(log)
        , discovery_sub_(node)
        , discovery_pub_(node)
        , cluster_size_(0)
        , num_known_servers_(0)
        , had_discovery_activity_(false)
    { }

    /**
     * If cluster_size is set to ClusterSizeUnknown, the class will try to read this parameter from the
     * storage backend using key 'cluster_size'.
     * Returns negative error code.
     */
    int init(uint8_t init_cluster_size = ClusterSizeUnknown);

    /**
     * Whether such server has been discovered earlier.
     */
    bool isKnownServer(NodeID node_id) const;

    /**
     * An invalid node ID will be returned if there's no such server.
     * The local server is not listed there.
     */
    NodeID getRemoteServerNodeIDAtIndex(uint8_t index) const;

    /**
     * See next_index[] in Raft paper.
     */
    Log::Index getServerNextIndex(NodeID server_node_id) const;
    void incrementServerNextIndexBy(NodeID server_node_id, Log::Index increment);
    void decrementServerNextIndex(NodeID server_node_id);

    /**
     * See match_index[] in Raft paper.
     */
    Log::Index getServerMatchIndex(NodeID server_node_id) const;
    void setServerMatchIndex(NodeID server_node_id, Log::Index match_index);

    /**
     * This method must be called when the current server becomes leader.
     */
    void resetAllServerIndices();

    /**
     * This method returns true if there was at least one Discovery message received since last call.
     */
    bool hadDiscoveryActivity()
    {
        if (had_discovery_activity_)
        {
            had_discovery_activity_ = false;
            return true;
        }
        return false;
    }

    /**
     * Number of known servers can only grow, and it never exceeds the cluster size value.
     * This number does not include the local server.
     */
    uint8_t getNumKnownServers() const { return num_known_servers_; }

    /**
     * Cluster size and quorum size are constant.
     */
    uint8_t getClusterSize() const { return cluster_size_; }
    uint8_t getQuorumSize() const { return static_cast<uint8_t>(cluster_size_ / 2U + 1U); }

    bool isClusterDiscovered() const { return num_known_servers_ == (cluster_size_ - 1); }
};

/**
 * Allocator has to implement this interface so the RaftCore can inform it when a new entry gets committed to the log.
 */
class ILeaderLogCommitHandler
{
public:
    /**
     * This method will be invoked when a new log entry is committed (only if the local server is the current Leader).
     */
    virtual void onEntryCommitted(const Entry& entry) = 0;

    virtual ~ILeaderLogCommitHandler() { }
};

/**
 * This class implements log replication and voting.
 * It does not implement client-server interaction at all; instead it just exposes a public method for adding
 * allocation entries.
 */
class RaftCore : private TimerBase
{
    typedef MethodBinder<RaftCore*, void (RaftCore::*)(const ReceivedDataStructure<AppendEntries::Request>&,
                                                       ServiceResponseDataStructure<AppendEntries::Response>&)>
        AppendEntriesCallback;

    typedef MethodBinder<RaftCore*, void (RaftCore::*)(const ServiceCallResult<AppendEntries>&)>
        AppendEntriesResponseCallback;

    typedef MethodBinder<RaftCore*, void (RaftCore::*)(const ReceivedDataStructure<RequestVote::Request>&,
                                                       ServiceResponseDataStructure<RequestVote::Response>&)>
        RequestVoteCallback;

    typedef MethodBinder<RaftCore*, void (RaftCore::*)(const ServiceCallResult<RequestVote>&)>
        RequestVoteResponseCallback;

    enum ServerState
    {
        ServerStateFollower,
        ServerStateCandidate,
        ServerStateLeader
    };

    struct PendingAppendEntriesFields
    {
        Log::Index prev_log_index;
        Log::Index num_entries;

        PendingAppendEntriesFields()
            : prev_log_index(0)
            , num_entries(0)
        { }
    };

    /*
     * Constants
     */
    const MonotonicDuration update_interval_;           ///< AE requests will be issued at this rate
    const MonotonicDuration base_activity_timeout_;

    IDynamicNodeIDAllocationServerEventTracer& tracer_;
    ILeaderLogCommitHandler& log_commit_handler_;

    /*
     * States
     */
    PersistentState persistent_state_;
    ClusterManager cluster_;
    Log::Index commit_index_;

    MonotonicTime last_activity_timestamp_;
    bool active_mode_;
    ServerState server_state_;

    uint8_t next_server_index_;         ///< Next server to query AE from
    uint8_t num_votes_received_in_this_campaign_;

    PendingAppendEntriesFields pending_append_entries_fields_;

    /*
     * Transport
     */
    ServiceServer<AppendEntries, AppendEntriesCallback>         append_entries_srv_;
    ServiceClient<AppendEntries, AppendEntriesResponseCallback> append_entries_client_;
    ServiceServer<RequestVote, RequestVoteCallback> request_vote_srv_;

    enum { NumRequestVoteClients = ClusterManager::MaxClusterSize - 1 };
    LazyConstructor<ServiceClient<RequestVote, RequestVoteResponseCallback> >
        request_vote_clients_[NumRequestVoteClients];

    void trace(TraceEvent event, int64_t argument) { tracer_.onEvent(event, argument); }

    INode&       getNode()       { return append_entries_srv_.getNode(); }
    const INode& getNode() const { return append_entries_srv_.getNode(); }

    void registerActivity() { last_activity_timestamp_ = getNode().getMonotonicTime(); }
    bool isActivityTimedOut() const;

    void handlePersistentStateUpdateError(int error);

    void updateFollower();
    void updateCandidate();
    void updateLeader();

    void switchState(ServerState new_state);
    void setActiveMode(bool new_active);

    void tryIncrementCurrentTermFromResponse(Term new_term);

    void propagateCommitIndex();

    void handleAppendEntriesRequest(const ReceivedDataStructure<AppendEntries::Request>& request,
                                    ServiceResponseDataStructure<AppendEntries::Response>& response);

    void handleAppendEntriesResponse(const ServiceCallResult<AppendEntries>& result);

    void handleRequestVoteRequest(const ReceivedDataStructure<RequestVote::Request>& request,
                                  ServiceResponseDataStructure<RequestVote::Response>& response);

    void handleRequestVoteResponse(const ServiceCallResult<RequestVote>& result);

    virtual void handleTimerEvent(const TimerEvent& event);

public:
    RaftCore(INode& node,
             IDynamicNodeIDStorageBackend& storage,
             IDynamicNodeIDAllocationServerEventTracer& tracer,
             ILeaderLogCommitHandler& log_commit_handler,
             MonotonicDuration update_interval =
                 MonotonicDuration::fromMSec(AppendEntries::Request::DEFAULT_REQUEST_TIMEOUT_MS),
             MonotonicDuration base_activity_timeout =
                 MonotonicDuration::fromMSec(AppendEntries::Request::DEFAULT_BASE_ELECTION_TIMEOUT_MS))
        : TimerBase(node)
        , update_interval_(update_interval)
        , base_activity_timeout_(base_activity_timeout)
        , tracer_(tracer)
        , log_commit_handler_(log_commit_handler)
        , persistent_state_(storage, tracer)
        , cluster_(node, storage, persistent_state_.getLog(), tracer)
        , commit_index_(0)                                  // Per Raft paper, commitIndex must be initialized to zero
        , last_activity_timestamp_(node.getMonotonicTime())
        , active_mode_(true)
        , server_state_(ServerStateFollower)
        , next_server_index_(0)
        , num_votes_received_in_this_campaign_(0)
        , append_entries_srv_(node)
        , append_entries_client_(node)
        , request_vote_srv_(node)
    {
        for (uint8_t i = 0; i < NumRequestVoteClients; i++)
        {
            request_vote_clients_[i].construct<INode&>(node);
        }
    }

    /**
     * Once started, the logic runs in the background until destructor is called.
     * @param cluster_size      If set, this value will be used and stored in the persistent storage. If not set,
     *                          value from the persistent storage will be used. If not set and there's no such key
     *                          in the persistent storage, initialization will fail.
     */
    int init(uint8_t cluster_size = ClusterManager::ClusterSizeUnknown);

    /**
     * This function is mostly needed for testing.
     */
    Log::Index getCommitIndex() const { return commit_index_; }

    /**
     * Only the leader can call @ref appendLog().
     */
    bool isLeader() const { return server_state_ == ServerStateLeader; }

    /**
     * Inserts one entry into log.
     * Failures are tolerble because all operations are idempotent.
     * This method will trigger an assertion failure and return error if the current node is not the leader.
     */
    int appendLog(const Entry::FieldTypes::unique_id& unique_id, NodeID node_id);

    /**
     * This class is used to perform log searches.
     */
    struct LogEntryInfo
    {
        Entry entry;
        bool committed;

        LogEntryInfo(const Entry& arg_entry, bool arg_committed)
            : entry(arg_entry)
            , committed(arg_committed)
        { }
    };

    /**
     * This method is used by the allocator to query existence of certain entries in the Raft log.
     * Predicate is a callable of the following prototype:
     *  bool (const LogEntryInfo& entry)
     * Once the predicate returns true, the loop will be terminated and the method will return an initialized lazy
     * contructor to the last visited entry; otherwise the constructor will not be initialized. In this case, lazy
     * constructor is used as boost::optional.
     * The log is always traversed from HIGH to LOW index values, i.e. entry 0 will be traversed last.
     */
    template <typename Predicate>
    inline LazyConstructor<LogEntryInfo> traverseLogFromEndUntil(const Predicate& predicate) const
    {
        UAVCAN_ASSERT(try_implicit_cast<bool>(predicate, true));
        for (int index = static_cast<int>(persistent_state_.getLog().getLastIndex()); index--; index >= 0)
        {
            const Entry* const entry = persistent_state_.getLog().getEntryAtIndex(Log::Index(index));
            UAVCAN_ASSERT(entry != NULL);
            const LogEntryInfo info(*entry, Log::Index(index) <= commit_index_);
            if (predicate(info))
            {
                LazyConstructor<LogEntryInfo> ret;
                ret.template construct<const LogEntryInfo&>(info);
                return ret;
            }
        }
        return LazyConstructor<LogEntryInfo>();
    }
};

/**
 * The main allocator must implement this interface.
 */
class IAllocationRequestHandler
{
public:
    typedef protocol::dynamic_node_id::server::Entry::FieldTypes::unique_id UniqueID;

    virtual void handleAllocationRequest(const UniqueID& unique_id, NodeID preferred_node_id) = 0;

    virtual ~IAllocationRequestHandler() { }
};

/**
 * This class manages communication with allocation clients.
 * Three-stage unique ID exchange is implemented here, as well as response publication.
 */
class AllocationRequestManager
{
    typedef MethodBinder<AllocationRequestManager*,
                         void (AllocationRequestManager::*)
                             (const ReceivedDataStructure<protocol::dynamic_node_id::Allocation>&)>
        AllocationCallback;

    const MonotonicDuration stage_timeout_;

    bool active_;
    MonotonicTime last_message_timestamp_;
    protocol::dynamic_node_id::Allocation::FieldTypes::unique_id current_unique_id_;

    IAllocationRequestHandler& handler_;

    Subscriber<protocol::dynamic_node_id::Allocation, AllocationCallback> allocation_sub_;
    Publisher<protocol::dynamic_node_id::Allocation> allocation_pub_;

    enum { InvalidStage = 0 };

    static uint8_t detectRequestStage(const protocol::dynamic_node_id::Allocation& msg);
    uint8_t getExpectedStage() const;

    void broadcastIntermediateAllocationResponse();

    void handleAllocation(const ReceivedDataStructure<protocol::dynamic_node_id::Allocation>& msg);

public:
    AllocationRequestManager(INode& node, IAllocationRequestHandler& handler)
        : stage_timeout_(MonotonicDuration::fromMSec(protocol::dynamic_node_id::Allocation::DEFAULT_REQUEST_PERIOD_MS))
        , active_(false)
        , handler_(handler)
        , allocation_sub_(node)
        , allocation_pub_(node)
    { }

    int init();

    void setActive(bool x);
    bool isActive() const { return active_; }

    int broadcastAllocationResponse(const IAllocationRequestHandler::UniqueID& unique_id, NodeID allocated_node_id);
};

} // namespace dynamic_node_id_server_impl

/**
 * This class implements the top-level allocation logic and server API.
 */
class DynamicNodeIDAllocationServer : public dynamic_node_id_server_impl::IAllocationRequestHandler
                                    , public dynamic_node_id_server_impl::ILeaderLogCommitHandler
{
    typedef MethodBinder<DynamicNodeIDAllocationServer*,
                         void (DynamicNodeIDAllocationServer::*)
                             (const ReceivedDataStructure<protocol::NodeStatus>&)> NodeStatusCallback;

    typedef MethodBinder<DynamicNodeIDAllocationServer*,
                         void (DynamicNodeIDAllocationServer::*)(const ServiceCallResult<protocol::GetNodeInfo>&)>
        GetNodeInfoResponseCallback;

    typedef Map<NodeID, uint8_t, 10> PendingGetNodeInfoAttemptsMap;

    enum { MaxGetNodeInfoAttempts = 5 };

    /*
     * States
     */
    PendingGetNodeInfoAttemptsMap pending_get_node_info_attempts_;
    dynamic_node_id_server_impl::RaftCore raft_core_;
    dynamic_node_id_server_impl::AllocationRequestManager allocation_request_manager_;

    /*
     * Transport
     */
    Subscriber<protocol::NodeStatus, NodeStatusCallback> node_status_sub_;
    ServiceClient<protocol::GetNodeInfo> get_node_info_client_;

    INode& getNode() { return get_node_info_client_.getNode(); }

    virtual void handleAllocationRequest(const UniqueID& unique_id, NodeID preferred_node_id);

    virtual void onEntryCommitted(const protocol::dynamic_node_id::server::Entry& entry);

public:
    DynamicNodeIDAllocationServer(INode& node,
                                  IDynamicNodeIDStorageBackend& storage,
                                  IDynamicNodeIDAllocationServerEventTracer& tracer)
        : pending_get_node_info_attempts_(node.getAllocator())
        , raft_core_(node, storage, tracer, *this)
        , allocation_request_manager_(node, *this)
        , node_status_sub_(node)
        , get_node_info_client_(node)
    { }

    enum { ClusterSizeUnknown = dynamic_node_id_server_impl::ClusterManager::ClusterSizeUnknown };

    int init(uint8_t cluster_size = ClusterSizeUnknown);
};

}

#endif // UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SERVER_HPP_INCLUDED
