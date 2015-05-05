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
 * Internals, do not use anything from this namespace directly.
 */
namespace dynamic_node_id_server_impl
{
/**
 * Raft term
 */
typedef StorageType<protocol::dynamic_node_id::server::Entry::FieldTypes::term>::Type Term;

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
                      protocol::dynamic_node_id::server::Entry::FieldTypes::unique_id& inout_value);

    /**
     * Getters simply read and deserialize the value.
     *  1. Read the value back from the backend; return false if read fails.
     *  2. Deserealize the newly read value; return false if deserialization fails.
     *  3. Update the argument with deserialized value.
     *  4. Return true.
     */
    int get(const IDynamicNodeIDStorageBackend::String& key, uint32_t& out_value) const;
    int get(const IDynamicNodeIDStorageBackend::String& key,
            protocol::dynamic_node_id::server::Entry::FieldTypes::unique_id& out_value) const;
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
    protocol::dynamic_node_id::server::Entry entries_[Capacity];
    Index last_index_;             // Index zero always contains an empty entry

    static IDynamicNodeIDStorageBackend::String getLastIndexKey() { return "log_last_index"; }
    static IDynamicNodeIDStorageBackend::String makeEntryKey(Index index, const char* postfix);

    int readEntryFromStorage(Index index, protocol::dynamic_node_id::server::Entry& out_entry);
    int writeEntryToStorage(Index index, const protocol::dynamic_node_id::server::Entry& entry);

    int initEmptyLogStorage();

public:
    Log(IDynamicNodeIDStorageBackend& storage)
        : storage_(storage)
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
    int append(const protocol::dynamic_node_id::server::Entry& entry);

    /**
     * This method invokes storage IO.
     * Returned value indicates whether the requested operation has been carried out successfully.
     */
    int removeEntriesWhereIndexGreaterOrEqual(Index index);

    /**
     * Returns nullptr if there's no such index.
     * This method does not use storage IO.
     */
    const protocol::dynamic_node_id::server::Entry* getEntryAtIndex(Index index) const;

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

    Term current_term_;
    NodeID voted_for_;
    Log log_;

    static IDynamicNodeIDStorageBackend::String getCurrentTermKey() { return "current_term"; }
    static IDynamicNodeIDStorageBackend::String getVotedForKey() { return "voted_for"; }

public:
    PersistentState(IDynamicNodeIDStorageBackend& storage)
        : storage_(storage)
        , current_term_(0)
        , log_(storage)
    { }

    int init();

    Term getCurrentTerm() const { return current_term_; }

    NodeID getVotedFor() const { return voted_for_; }

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
};

/**
 * This class maintains the cluster state.
 */
class ClusterManager : private TimerBase
{
    typedef MethodBinder<ClusterManager*,
                         void (ClusterManager::*)
                             (const ReceivedDataStructure<protocol::dynamic_node_id::server::Discovery>&)>
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

    enum { MaxServers = protocol::dynamic_node_id::server::Discovery::FieldTypes::known_nodes::MaxSize };

    IDynamicNodeIDStorageBackend& storage_;
    const Log& log_;

    Subscriber<protocol::dynamic_node_id::server::Discovery, DiscoveryCallback> discovery_sub_;
    mutable Publisher<protocol::dynamic_node_id::server::Discovery> discovery_pub_;

    Server servers_[MaxServers - 1];   ///< Minus one because the local server is not listed there.

    uint8_t cluster_size_;
    uint8_t num_known_servers_;

    bool had_discovery_activity_;

    static IDynamicNodeIDStorageBackend::String getStorageKeyForClusterSize() { return "cluster_size"; }

    INode&       getNode()       { return discovery_sub_.getNode(); }
    const INode& getNode() const { return discovery_sub_.getNode(); }

    Server*       findServer(NodeID node_id);
    const Server* findServer(NodeID node_id) const;
    bool isKnownServer(NodeID node_id) const;
    void addServer(NodeID node_id);

    virtual void handleTimerEvent(const TimerEvent&);

    void handleDiscovery(const ReceivedDataStructure<protocol::dynamic_node_id::server::Discovery>& msg);

    void startDiscoveryPublishingTimerIfNotRunning();

public:
    enum { ClusterSizeUnknown = 0 };

    /**
     * @param node          Needed to publish and subscribe to Discovery message
     * @param storage       Needed to read the cluster size parameter from the storage
     * @param log           Needed to initialize nextIndex[] values after elections
     */
    ClusterManager(INode& node, IDynamicNodeIDStorageBackend& storage, const Log& log)
        : TimerBase(node)
        , storage_(storage)
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

    uint8_t getNumKnownServers() const { return num_known_servers_; }
    uint8_t getClusterSize() const { return cluster_size_; }
    uint8_t getQuorumSize() const { return static_cast<uint8_t>(cluster_size_ / 2U + 1U); }

    bool isClusterDiscovered() const { return num_known_servers_ == (cluster_size_ - 1); }
};

/**
 * This class implements log replication and voting.
 * It does not implement client-server interaction at all; instead it just exposes a public method for adding
 * allocation entries.
 */
class RaftCore : private TimerBase
{
    typedef MethodBinder<RaftCore*,
                         void (RaftCore::*)
                             (const protocol::dynamic_node_id::server::AppendEntries::Request&,
                              protocol::dynamic_node_id::server::AppendEntries::Response&)> AppendEntriesCallback;

    typedef MethodBinder<RaftCore*,
                         void (RaftCore::*)
                             (const protocol::dynamic_node_id::server::RequestVote::Request&,
                              protocol::dynamic_node_id::server::RequestVote::Response&)> RequestVoteCallback;

    typedef MethodBinder<RaftCore*,
                         void (RaftCore::*)
                             (const ServiceCallResult<protocol::dynamic_node_id::server::AppendEntries>&)>
        AppendEntriesResponseCallback;

    typedef MethodBinder<RaftCore*,
                         void (RaftCore::*)
                             (const ServiceCallResult<protocol::dynamic_node_id::server::RequestVote>&)>
        RequestVoteResponseCallback;

    enum ServerState
    {
        ServerStateFollower,
        ServerStateCandidate,
        ServerStateLeader
    };

    dynamic_node_id_server_impl::PersistentState persistent_state_;

    dynamic_node_id_server_impl::Log::Index commit_index_;

    dynamic_node_id_server_impl::ClusterManager cluster_;

    MonotonicTime last_activity_timestamp_;
    bool active_mode_;

    ServerState server_state_;

    ServiceServer<protocol::dynamic_node_id::server::AppendEntries, AppendEntriesCallback> append_entries_srv_;
    ServiceServer<protocol::dynamic_node_id::server::RequestVote, RequestVoteCallback> request_vote_srv_;

    ServiceClient<protocol::dynamic_node_id::server::AppendEntries,
                  AppendEntriesResponseCallback> append_entries_client_;
    ServiceClient<protocol::dynamic_node_id::server::RequestVote,
                  RequestVoteResponseCallback> request_vote_client_;

    virtual void handleTimerEvent(const TimerEvent&);

public:
    RaftCore(INode& node, IDynamicNodeIDStorageBackend& storage)
        : TimerBase(node)
        , persistent_state_(storage)
        , commit_index_(0)              // Per Raft paper, commitIndex must be initialized to zero
        , cluster_(node, storage, persistent_state_.getLog())
        , last_activity_timestamp_(node.getMonotonicTime())
        , active_mode_(true)
        , server_state_(ServerStateFollower)
        , append_entries_srv_(node)
        , request_vote_srv_(node)
        , append_entries_client_(node)
        , request_vote_client_(node)
    { }

    /**
     * Once started, the logic runs in the background until destructor is called.
     */
    int init();

    /**
     * Inserts one entry into log. This operation may fail, which will not be reported.
     * Failures are tolerble because all operations are idempotent.
     */
    void appendLog(const protocol::dynamic_node_id::server::Entry& entry);

    /**
     * This class is used to perform log searches.
     */
    struct LogEntryInfo
    {
        protocol::dynamic_node_id::server::Entry entry;
        bool committed;

        LogEntryInfo(const protocol::dynamic_node_id::server::Entry& arg_entry, bool arg_committed)
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
            const protocol::dynamic_node_id::server::Entry* const entry =
                persistent_state_.getLog().getEntryAtIndex(Log::Index(index));
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

} // namespace dynamic_node_id_impl

/**
 *
 */
class DynamicNodeIDAllocationServer
{
    typedef MethodBinder<DynamicNodeIDAllocationServer*,
                         void (DynamicNodeIDAllocationServer::*)
                             (const ReceivedDataStructure<protocol::dynamic_node_id::Allocation>&)>
        AllocationCallback;

    typedef MethodBinder<DynamicNodeIDAllocationServer*,
                         void (DynamicNodeIDAllocationServer::*)
                             (const ReceivedDataStructure<protocol::NodeStatus>&)> NodeStatusCallback;

    typedef Map<NodeID, uint8_t, 10> PendingGetNodeInfoAttemptsMap;

    PendingGetNodeInfoAttemptsMap pending_get_node_info_attempts_;

    Subscriber<protocol::dynamic_node_id::Allocation, AllocationCallback> allocation_sub_;
    Publisher<protocol::dynamic_node_id::Allocation> allocation_pub_;

    Subscriber<protocol::NodeStatus, NodeStatusCallback> node_status_sub_;

};

}

#endif // UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SERVER_HPP_INCLUDED
