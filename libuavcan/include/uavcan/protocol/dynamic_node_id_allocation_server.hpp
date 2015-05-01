/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SERVER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SERVER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/subscriber.hpp>
#include <uavcan/node/publisher.hpp>
#include <uavcan/node/timer.hpp>
#include <uavcan/util/method_binder.hpp>
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
 *
 */
class DynamicNodeIDAllocationServer
{
    /*
     * Public type definitions
     */
public:
    /**
     * This interface is used by the server to read and write stable storage.
     * The storage is represented as a key-value container, where keys and values are ASCII strings up to 32
     * characters long, not including the termination byte. Fixed block size allows for absolutely straightforward
     * and efficient implementation of storage backends, e.g. based on text files.
     */
    class IStorageBackend
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
         * Create or update value for the given key.
         * This method should not block for more than 50 ms.
         * Failures will be ignored.
         */
        virtual void set(const String& key, const String& value) = 0;

        virtual ~IStorageBackend() { }
    };

private:
    /*
     * Callback type definitions
     */
    typedef MethodBinder<DynamicNodeIDAllocationServer*,
                         void (DynamicNodeIDAllocationServer::*)
                             (const ReceivedDataStructure<protocol::dynamic_node_id::Allocation>&)>
        AllocationCallback;

    typedef MethodBinder<DynamicNodeIDAllocationServer*,
                         void (DynamicNodeIDAllocationServer::*)
                             (const ReceivedDataStructure<protocol::NodeStatus>&)> NodeStatusCallback;

    typedef MethodBinder<DynamicNodeIDAllocationServer*,
                         void (DynamicNodeIDAllocationServer::*)
                             (const protocol::dynamic_node_id::server::AppendEntries::Request&,
                              protocol::dynamic_node_id::server::AppendEntries::Response&)> AppendEntriesCallback;

    typedef MethodBinder<DynamicNodeIDAllocationServer*,
                         void (DynamicNodeIDAllocationServer::*)
                             (const protocol::dynamic_node_id::server::RequestVote::Request&,
                              protocol::dynamic_node_id::server::RequestVote::Response&)> RequestVoteCallback;

    typedef MethodBinder<DynamicNodeIDAllocationServer*,
                         void (DynamicNodeIDAllocationServer::*)
                             (const ServiceCallResult<protocol::dynamic_node_id::server::AppendEntries>&)>
        AppendEntriesResponseCallback;

    typedef MethodBinder<DynamicNodeIDAllocationServer*,
                         void (DynamicNodeIDAllocationServer::*)
                             (const ServiceCallResult<protocol::dynamic_node_id::server::RequestVote>&)>
        RequestVoteResponseCallback;

    /*
     * Internal type definitions
     */
    typedef Map<NodeID, uint8_t> PendingGetNodeInfoAttemptsMap;

    typedef StorageType<protocol::dynamic_node_id::server::Entry::FieldTypes::term>::Type Term;

    /**
     * This class extends the storage backend interface with serialization/deserialization functionality.
     */
    class StorageMarshallingDecorator
    {
        IStorageBackend& storage_;

    public:
        StorageMarshallingDecorator(IStorageBackend& storage)
            : storage_(storage)
        {
            // Making sure that there will be no data loss during serialization.
            StaticAssert<(sizeof(Term) <= sizeof(uint32_t))>::check();
        }

        /**
         * Setters do the following:
         *  1. Serialize the value.
         *  2. Update the value on the backend.
         *  3. Read the value back from the backend; return false if read fails.
         *  4. Deserealize the newly read value.
         *  5. Update the argument with deserialized value.
         *  6. Return true.
         * The caller then is supposed to check whether the argument has the desired value.
         */
        bool setAndGetBack(const IStorageBackend::String& key, uint32_t& inout_value);
        bool setAndGetBack(const IStorageBackend::String& key,
                           protocol::dynamic_node_id::server::Entry::FieldTypes::unique_id& inout_value);

        /**
         * Getters simply read and deserialize the value.
         */
        bool get(const IStorageBackend::String& key, uint32_t& out_value) const;
        bool get(const IStorageBackend::String& key,
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

    private:
        enum { Capacity = NodeID::Max + 1 };

        IStorageBackend& storage_;
        protocol::dynamic_node_id::server::Entry entries_[Capacity];
        Index max_index_;             // Index zero always contains an empty entry

    public:
        Log(IStorageBackend& storage)
            : storage_(storage)
            , max_index_(0)
        { }

        /**
         * This method invokes storage IO.
         */
        int init();

        /**
         * This method invokes storage IO.
         */
        void append(const protocol::dynamic_node_id::server::Entry& entry);

        /**
         * This method invokes storage IO.
         */
        void removeEntriesWhereIndexGreaterOrEqual(Index index);

        /**
         * Returns nullptr if there's no such index.
         * This method does not use storage IO.
         */
        const protocol::dynamic_node_id::server::Entry* getEntryAtIndex(Index index) const;

        Index getMaxIndex() const { return max_index_; }

        bool isOtherLogUpToDate(Index other_last_index, Term other_last_term) const;
    };

    /**
     * This class is a convenient container for persistent state variables defined by Raft.
     * Writes are slow, reads are instantaneous.
     */
    class PersistentState
    {
        IStorageBackend& storage_;

        Term current_term_;
        NodeID voted_for_;
        Log log_;

    public:
        PersistentState(IStorageBackend& storage)
            : storage_(storage)
            , current_term_(0)
        { }

        int init();

        Term getCurrentTerm() const { return current_term_; }

        NodeID getVotedFor() const { return voted_for_; }

        Log& getLog()             { return log_; }
        const Log& getLog() const { return log_; }

        /**
         * Invokes storage IO.
         */
        void setCurrentTerm(Term term);

        /**
         * Invokes storage IO.
         */
        void setVotedFor(NodeID node_id);
    };

    /**
     * This class maintains the cluster state.
     */
    class ClusterManager : private TimerBase
    {
        typedef MethodBinder<DynamicNodeIDAllocationServer*,
                             void (DynamicNodeIDAllocationServer::*)
                                 (const ReceivedDataStructure<protocol::dynamic_node_id::server::Discovery>&)>
            DiscoveryCallback;

        struct Server
        {
            const NodeID node_id;
            Log::Index next_index;
            Log::Index match_index;

            Server()
                : next_index(0)
                , match_index(0)
            { }
        };

        enum { MaxServers = protocol::dynamic_node_id::server::Discovery::FieldTypes::known_nodes::MaxSize };

        const IStorageBackend& storage_;
        const Log& log_;

        Subscriber<protocol::dynamic_node_id::server::Discovery, DiscoveryCallback> discovery_sub_;
        mutable Publisher<protocol::dynamic_node_id::server::Discovery> discovery_pub_;

        Server servers_[MaxServers - 1];   ///< Minus one because the local server is not listed there.

        uint8_t cluster_size_;
        uint8_t num_known_servers_;

        virtual void handleTimerEvent(const TimerEvent& event);

        void handleDiscovery(const ReceivedDataStructure<protocol::dynamic_node_id::server::Discovery>& msg);

        void publishDiscovery() const;

    public:
        enum { ClusterSizeUnknown = 0 };

        /**
         * @param node          Needed to publish and subscribe to Discovery message
         * @param storage       Needed to read the cluster size parameter from the storage
         * @param log           Needed to initialize nextIndex[] values after elections
         */
        ClusterManager(INode& node, const IStorageBackend& storage, const Log& log)
            : storage_(storage)
            , log_(log)
            , discovery_sub_(node)
            , discovery_pub_(node)
            , cluster_size_(0)
            , num_known_servers_(0)
        { }

        /**
         * If cluster_size is set to ClusterSizeUnknown, the class will try to read this parameter from the
         * storage backend using key 'cluster_size'.
         * Returns negative error code.
         */
        int init(uint8_t cluster_size = ClusterSizeUnknown);

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

        uint8_t getNumKnownServers() const { return num_known_servers_; }
        uint8_t getConfiguredClusterSize() const { return cluster_size_; }
        uint8_t getQuorumSize() const { return static_cast<uint8_t>(cluster_size_ / 2U + 1U); }
    };

    enum ServerState
    {
        ServerStateFollower,
        ServerStateCandidate,
        ServerStateLeader
    };

    /*
     * Raft-related states
     */
    PersistentState persistent_state_;  ///< Modifications of this state are slow as they involve storage IO.

    Log::Index commit_index_;

    ClusterManager cluster_;

    /*
     * Implementation-specific states
     */
    PendingGetNodeInfoAttemptsMap pending_get_node_info_attempts_;

    MonotonicTime last_activity_timestamp_;
    bool active_mode_;

    ServerState server_state_;

    /*
     * Transport
     */
    Subscriber<protocol::dynamic_node_id::Allocation, AllocationCallback> allocation_sub_;
    Publisher<protocol::dynamic_node_id::Allocation> allocation_pub_;

    Subscriber<protocol::NodeStatus, NodeStatusCallback> node_status_sub_;

    ServiceServer<protocol::dynamic_node_id::server::AppendEntries, AppendEntriesCallback> append_entries_srv_;
    ServiceServer<protocol::dynamic_node_id::server::RequestVote, RequestVoteCallback> request_vote_srv_;

    ServiceClient<protocol::dynamic_node_id::server::AppendEntries,
                  AppendEntriesResponseCallback> append_entries_client_;
    ServiceClient<protocol::dynamic_node_id::server::RequestVote,
                  RequestVoteResponseCallback> request_vote_client_;
};

}

#endif // UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SERVER_HPP_INCLUDED
