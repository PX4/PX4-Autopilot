/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdlib>
#include <uavcan/protocol/dynamic_node_id_allocation_server.hpp>

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
# include <cerrno>
#endif

#ifndef UAVCAN_CPP_VERSION
# error UAVCAN_CPP_VERSION
#endif

namespace uavcan
{
namespace dynamic_node_id_server_impl
{
/*
 * MarshallingStorageDecorator
 */
uint8_t MarshallingStorageDecorator::convertLowerCaseHexCharToNibble(char ch)
{
    const uint8_t ret = (ch > '9') ? static_cast<uint8_t>(ch - 'a' + 10) : static_cast<uint8_t>(ch - '0');
    UAVCAN_ASSERT(ret < 16);
    return ret;
}

int MarshallingStorageDecorator::setAndGetBack(const IDynamicNodeIDStorageBackend::String& key, uint32_t& inout_value)
{
    IDynamicNodeIDStorageBackend::String serialized;
    serialized.appendFormatted("%llu", static_cast<unsigned long long>(inout_value));

    UAVCAN_TRACE("MarshallingStorageDecorator", "Set %s = %s", key.c_str(), serialized.c_str());
    storage_.set(key, serialized);

    return get(key, inout_value);
}

int MarshallingStorageDecorator::setAndGetBack(const IDynamicNodeIDStorageBackend::String& key,
                                               Entry::FieldTypes::unique_id& inout_value)
{
    IDynamicNodeIDStorageBackend::String serialized;
    for (uint8_t i = 0; i < Entry::FieldTypes::unique_id::MaxSize; i++)
    {
        serialized.appendFormatted("%02x", inout_value.at(i));
    }
    UAVCAN_ASSERT(serialized.size() == Entry::FieldTypes::unique_id::MaxSize * 2);

    UAVCAN_TRACE("MarshallingStorageDecorator", "Set %s = %s", key.c_str(), serialized.c_str());
    storage_.set(key, serialized);

    return get(key, inout_value);
}

int MarshallingStorageDecorator::get(const IDynamicNodeIDStorageBackend::String& key, uint32_t& out_value) const
{
    /*
     * Reading the storage
     */
    const IDynamicNodeIDStorageBackend::String val = storage_.get(key);
    if (val.empty())
    {
        return -ErrFailure;
    }

    /*
     * Per MISRA C++ recommendations, checking the inputs instead of relying solely on errno.
     * The value must contain only numeric characters.
     */
    for (IDynamicNodeIDStorageBackend::String::const_iterator it = val.begin(); it != val.end(); ++it)
    {
        if (static_cast<char>(*it) < '0' || static_cast<char>(*it) > '9')
        {
            return -ErrFailure;
        }
    }

    if (val.size() > 10) // len(str(0xFFFFFFFF))
    {
        return -ErrFailure;
    }

    /*
     * Conversion is carried out here
     */
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
    errno = 0;
#endif

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
    const unsigned long long x = std::strtoull(val.c_str(), NULL, 10);
#else
    // There was no strtoull() before C++11, so we need to resort to strtoul()
    StaticAssert<(sizeof(unsigned long) >= sizeof(uint32_t))>::check();
    const unsigned long x = std::strtoul(val.c_str(), NULL, 10);
#endif

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
    if (errno != 0)
    {
        return -ErrFailure;
    }
#endif

    out_value = static_cast<uint32_t>(x);
    return 0;
}

int MarshallingStorageDecorator::get(const IDynamicNodeIDStorageBackend::String& key,
                                     Entry::FieldTypes::unique_id& out_value) const
{
    static const uint8_t NumBytes = Entry::FieldTypes::unique_id::MaxSize;

    /*
     * Reading the storage
     */
    IDynamicNodeIDStorageBackend::String val = storage_.get(key);
    if (val.size() != NumBytes * 2)
    {
        return -ErrFailure;
    }

    /*
     * The value must contain only hexadecimal numbers.
     */
    val.convertToLowerCaseASCII();
    for (IDynamicNodeIDStorageBackend::String::const_iterator it = val.begin(); it != val.end(); ++it)
    {
        if ((static_cast<char>(*it) < '0' || static_cast<char>(*it) > '9') &&
            (static_cast<char>(*it) < 'a' || static_cast<char>(*it) > 'f'))
        {
            return -ErrFailure;
        }
    }

    /*
     * Conversion is carried out here
     */
    IDynamicNodeIDStorageBackend::String::const_iterator it = val.begin();

    for (uint8_t byte_index = 0; byte_index < NumBytes; byte_index++)
    {
        out_value[byte_index] = static_cast<uint8_t>(convertLowerCaseHexCharToNibble(static_cast<char>(*it++)) << 4);
        out_value[byte_index] = static_cast<uint8_t>(convertLowerCaseHexCharToNibble(static_cast<char>(*it++)) |
                                                     out_value[byte_index]);
    }

    return 0;
}

/*
 * Log
 */
IDynamicNodeIDStorageBackend::String Log::makeEntryKey(Index index, const char* postfix)
{
    IDynamicNodeIDStorageBackend::String str;
    // "log0_foobar"
    str += "log";
    str.appendFormatted("%d", int(index));
    str += "_";
    str += postfix;
    return str;
}

int Log::readEntryFromStorage(Index index, Entry& out_entry)
{
    const MarshallingStorageDecorator io(storage_);

    // Term
    if (io.get(makeEntryKey(index, "term"), out_entry.term) < 0)
    {
        return -ErrFailure;
    }

    // Unique ID
    if (io.get(makeEntryKey(index, "unique_id"), out_entry.unique_id) < 0)
    {
        return -ErrFailure;
    }

    // Node ID
    uint32_t node_id = 0;
    if (io.get(makeEntryKey(index, "node_id"), node_id) < 0)
    {
        return -ErrFailure;
    }
    if (node_id > NodeID::Max)
    {
        return -ErrFailure;
    }
    out_entry.node_id = static_cast<uint8_t>(node_id);

    return 0;
}

int Log::writeEntryToStorage(Index index, const Entry& entry)
{
    Entry temp = entry;

    MarshallingStorageDecorator io(storage_);

    // Term
    if (io.setAndGetBack(makeEntryKey(index, "term"), temp.term) < 0)
    {
        return -ErrFailure;
    }

    // Unique ID
    if (io.setAndGetBack(makeEntryKey(index, "unique_id"), temp.unique_id) < 0)
    {
        return -ErrFailure;
    }

    // Node ID
    uint32_t node_id = entry.node_id;
    if (io.setAndGetBack(makeEntryKey(index, "node_id"), node_id) < 0)
    {
        return -ErrFailure;
    }
    temp.node_id = static_cast<uint8_t>(node_id);

    return (temp == entry) ? 0 : -ErrFailure;
}

int Log::initEmptyLogStorage()
{
    MarshallingStorageDecorator io(storage_);

    /*
     * Writing the zero entry - it must always be default-initialized
     */
    entries_[0] = Entry();
    int res = writeEntryToStorage(0, entries_[0]);
    if (res < 0)
    {
        return res;
    }

    /*
     * Initializing last index
     * Last index must be written AFTER the zero entry, otherwise if the write fails here the storage will be
     * left in an inconsistent state.
     */
    last_index_ = 0;
    uint32_t stored_index = 0;
    res = io.setAndGetBack(getLastIndexKey(), stored_index);
    if (res < 0)
    {
        return res;
    }
    if (stored_index != 0)
    {
        return -ErrFailure;
    }

    return 0;
}

int Log::init()
{
    MarshallingStorageDecorator io(storage_);

    // Reading max index
    {
        uint32_t value = 0;
        if (io.get(getLastIndexKey(), value) < 0)
        {
            if (storage_.get(getLastIndexKey()).empty())
            {
                UAVCAN_TRACE("dynamic_node_id_server_impl::Log", "Initializing empty storage");
                return initEmptyLogStorage();
            }
            else
            {
                // There's some data in the storage, but it cannot be parsed - reporting an error
                UAVCAN_TRACE("dynamic_node_id_server_impl::Log", "Failed to read last index");
                return -ErrFailure;
            }
        }
        if (value >= Capacity)
        {
            return -ErrFailure;
        }
        last_index_ = Index(value);
    }

    tracer_.onEvent(TraceLogLastIndexRestored, last_index_);

    // Restoring log entries - note that index 0 always exists
    for (Index index = 0; index <= last_index_; index++)
    {
        const int result = readEntryFromStorage(index, entries_[index]);
        if (result < 0)
        {
            UAVCAN_TRACE("dynamic_node_id_server_impl::Log", "Failed to read entry at index %u: %d",
                         unsigned(index), result);
            return result;
        }
    }

    UAVCAN_TRACE("dynamic_node_id_server_impl::Log", "Restored %u log entries", unsigned(last_index_));
    return 0;
}

int Log::append(const Entry& entry)
{
    if ((last_index_ + 1) >= Capacity)
    {
        return -ErrLogic;
    }

    tracer_.onEvent(TraceLogAppend, last_index_ + 1U);

    // If next operations fail, we'll get a dangling entry, but it's absolutely OK.
    int res = writeEntryToStorage(Index(last_index_ + 1), entry);
    if (res < 0)
    {
        return res;
    }

    // Updating the last index
    MarshallingStorageDecorator io(storage_);
    uint32_t new_last_index = last_index_ + 1U;
    res = io.setAndGetBack(getLastIndexKey(), new_last_index);
    if (res < 0)
    {
        return res;
    }
    if (new_last_index != last_index_ + 1U)
    {
        return -ErrFailure;
    }
    entries_[new_last_index] = entry;
    last_index_ = Index(new_last_index);

    UAVCAN_TRACE("dynamic_node_id_server_impl::Log", "New entry, index %u, node ID %u, term %u",
                 unsigned(last_index_), unsigned(entry.node_id), unsigned(entry.term));
    return 0;
}

int Log::removeEntriesWhereIndexGreaterOrEqual(Index index)
{
    UAVCAN_ASSERT(last_index_ < Capacity);

    if (((index) >= Capacity) || (index == 0))
    {
        return -ErrLogic;
    }

    tracer_.onEvent(TraceLogRemove, index - 1U);

    MarshallingStorageDecorator io(storage_);
    uint32_t new_last_index = index - 1U;
    int res = io.setAndGetBack(getLastIndexKey(), new_last_index);
    if (res < 0)
    {
        return res;
    }
    if (new_last_index != index - 1U)
    {
        return -ErrFailure;
    }
    UAVCAN_TRACE("dynamic_node_id_server_impl::Log", "Entries removed, last index %u --> %u",
                 unsigned(last_index_), unsigned(new_last_index));
    last_index_ = Index(new_last_index);

    // Removal operation leaves dangling entries in storage, it's OK
    return 0;
}

const Entry* Log::getEntryAtIndex(Index index) const
{
    UAVCAN_ASSERT(last_index_ < Capacity);
    return (index <= last_index_) ? &entries_[index] : NULL;
}

bool Log::isOtherLogUpToDate(Log::Index other_last_index, Term other_last_term) const
{
    UAVCAN_ASSERT(last_index_ < Capacity);
    // Terms are different - the one with higher term is more up-to-date
    if (other_last_term != entries_[last_index_].term)
    {
        return other_last_term > entries_[last_index_].term;
    }
    // Terms are equal - longer log wins
    return other_last_index >= last_index_;
}

/*
 * PersistentState
 */
int PersistentState::init()
{
    /*
     * Reading log
     */
    int res = log_.init();
    if (res < 0)
    {
        UAVCAN_TRACE("dynamic_node_id_server_impl::PersistentState", "Log init failed: %d", res);
        return res;
    }

    const Entry* const last_entry = log_.getEntryAtIndex(log_.getLastIndex());
    if (last_entry == NULL)
    {
        UAVCAN_ASSERT(0);
        return -ErrLogic;
    }

    const bool log_is_empty = (log_.getLastIndex() == 0) && (last_entry->term == 0);

    MarshallingStorageDecorator io(storage_);

    /*
     * Reading currentTerm
     */
    if (storage_.get(getCurrentTermKey()).empty() && log_is_empty)
    {
        // First initialization
        current_term_ = 0;
        res = io.setAndGetBack(getCurrentTermKey(), current_term_);
        if (res < 0)
        {
            UAVCAN_TRACE("dynamic_node_id_server_impl::PersistentState", "Failed to init current term: %d", res);
            return res;
        }
        if (current_term_ != 0)
        {
            return -ErrFailure;
        }
    }
    else
    {
        // Restoring
        res = io.get(getCurrentTermKey(), current_term_);
        if (res < 0)
        {
            UAVCAN_TRACE("dynamic_node_id_server_impl::PersistentState", "Failed to read current term: %d", res);
            return res;
        }
    }

    tracer_.onEvent(TraceCurrentTermRestored, current_term_);

    if (current_term_ < last_entry->term)
    {
        UAVCAN_TRACE("dynamic_node_id_server_impl::PersistentState",
                     "Persistent storage is damaged: current term is less than term of the last log entry (%u < %u)",
                     unsigned(current_term_), unsigned(last_entry->term));
        return -ErrLogic;
    }

    /*
     * Reading votedFor
     */
    if (storage_.get(getVotedForKey()).empty() && log_is_empty && (current_term_ == 0))
    {
        // First initialization
        voted_for_ = NodeID(0);
        uint32_t stored_voted_for = 0;
        res = io.setAndGetBack(getVotedForKey(), stored_voted_for);
        if (res < 0)
        {
            UAVCAN_TRACE("dynamic_node_id_server_impl::PersistentState", "Failed to init votedFor: %d", res);
            return res;
        }
        if (stored_voted_for != 0)
        {
            return -ErrFailure;
        }
    }
    else
    {
        // Restoring
        uint32_t stored_voted_for = 0;
        res = io.get(getVotedForKey(), stored_voted_for);
        if (res < 0)
        {
            UAVCAN_TRACE("dynamic_node_id_server_impl::PersistentState", "Failed to read votedFor: %d", res);
            return res;
        }
        if (stored_voted_for > NodeID::Max)
        {
            return -ErrFailure;
        }
        voted_for_ = NodeID(uint8_t(stored_voted_for));
    }

    tracer_.onEvent(TraceVotedForRestored, voted_for_.get());

    return 0;
}

int PersistentState::setCurrentTerm(const Term term)
{
    if (term < current_term_)
    {
        UAVCAN_ASSERT(0);
        return -ErrInvalidParam;
    }

    tracer_.onEvent(TraceCurrentTermUpdate, term);

    MarshallingStorageDecorator io(storage_);

    Term tmp = term;
    int res = io.setAndGetBack(getCurrentTermKey(), tmp);
    if (res < 0)
    {
        return res;
    }

    if (tmp != term)
    {
        return -ErrFailure;
    }

    current_term_ = term;
    return 0;
}

int PersistentState::setVotedFor(const NodeID node_id)
{
    if (!node_id.isValid())
    {
        UAVCAN_ASSERT(0);
        return -ErrInvalidParam;
    }

    tracer_.onEvent(TraceVotedForUpdate, node_id.get());

    MarshallingStorageDecorator io(storage_);

    uint32_t tmp = node_id.get();
    int res = io.setAndGetBack(getVotedForKey(), tmp);
    if (res < 0)
    {
        return res;
    }

    if (node_id.get() != tmp)
    {
        return -ErrFailure;
    }

    voted_for_ = node_id;
    return 0;
}

/*
 * ClusterManager
 */
void ClusterManager::Server::resetIndices(const Log& log)
{
    next_index = Log::Index(log.getLastIndex() + 1U);
    match_index = 0;
}

ClusterManager::Server* ClusterManager::findServer(NodeID node_id)
{
    for (uint8_t i = 0; i < num_known_servers_; i++)
    {
        UAVCAN_ASSERT(servers_[i].node_id.isUnicast());
        if (servers_[i].node_id == node_id)
        {
            return &servers_[i];
        }
    }
    return NULL;
}

const ClusterManager::Server* ClusterManager::findServer(NodeID node_id) const
{
    return const_cast<ClusterManager*>(this)->findServer(node_id);
}

bool ClusterManager::isKnownServer(NodeID node_id) const
{
    if (node_id == getNode().getNodeID())
    {
        return true;
    }
    for (uint8_t i = 0; i < num_known_servers_; i++)
    {
        UAVCAN_ASSERT(servers_[i].node_id.isUnicast());
        UAVCAN_ASSERT(servers_[i].node_id != getNode().getNodeID());
        if (servers_[i].node_id == node_id)
        {
            return true;
        }
    }
    return false;
}

void ClusterManager::addServer(NodeID node_id)
{
    UAVCAN_ASSERT((num_known_servers_ + 1) < (MaxServers - 2));
    if (!isKnownServer(node_id) && node_id.isUnicast())
    {
        tracer_.onEvent(TraceNewServerDiscovered, node_id.get());
        servers_[num_known_servers_].node_id = node_id;
        servers_[num_known_servers_].resetIndices(log_);
        num_known_servers_ = static_cast<uint8_t>(num_known_servers_ + 1U);
    }
    else
    {
        UAVCAN_ASSERT(0);
    }
}

void ClusterManager::handleTimerEvent(const TimerEvent&)
{
    UAVCAN_ASSERT(num_known_servers_ < cluster_size_);

    tracer_.onEvent(TraceDiscoveryBroadcast, num_known_servers_);

    /*
     * Filling the message
     */
    Discovery msg;
    msg.configured_cluster_size = cluster_size_;

    msg.known_nodes.push_back(getNode().getNodeID().get());     // Putting ourselves at index 0

    for (uint8_t i = 0; i < num_known_servers_; i++)
    {
        UAVCAN_ASSERT(servers_[i].node_id.isUnicast());
        msg.known_nodes.push_back(servers_[i].node_id.get());
    }

    UAVCAN_ASSERT(msg.known_nodes.size() == (num_known_servers_ + 1));

    /*
     * Broadcasting
     */
    UAVCAN_TRACE("dynamic_node_id_server_impl::ClusterManager", "Broadcasting Discovery message; known nodes: %d of %d",
                 int(msg.known_nodes.size()), int(cluster_size_));

    const int res = discovery_pub_.broadcast(msg);
    if (res < 0)
    {
        UAVCAN_TRACE("dynamic_node_id_server_impl::ClusterManager", "Discovery broadcst failed: %d", res);
        getNode().registerInternalFailure("Raft discovery broadcast");
    }

    /*
     * Termination condition
     */
    if (isClusterDiscovered())
    {
        UAVCAN_TRACE("dynamic_node_id_server_impl::ClusterManager", "Discovery broadcasting timer stopped");
        stop();
    }
}

void ClusterManager::handleDiscovery(const ReceivedDataStructure<Discovery>& msg)
{
    tracer_.onEvent(TraceDiscoveryReceived, msg.getSrcNodeID().get());

    /*
     * Validating cluster configuration
     * If there's a case of misconfiguration, the message will be ignored.
     */
    if (msg.configured_cluster_size != cluster_size_)
    {
        getNode().registerInternalFailure("Bad Raft cluster size");
        return;
    }

    had_discovery_activity_ = true;

    /*
     * Updating the set of known servers
     */
    for (uint8_t i = 0; i < msg.known_nodes.size(); i++)
    {
        if (isClusterDiscovered())
        {
            break;
        }

        const NodeID node_id(msg.known_nodes[i]);
        if (node_id.isUnicast() && !isKnownServer(node_id))
        {
            addServer(node_id);
        }
    }

    /*
     * Publishing a new Discovery request if the publishing server needs to learn about more servers.
     */
    if (msg.configured_cluster_size > msg.known_nodes.size())
    {
        startDiscoveryPublishingTimerIfNotRunning();
    }
}

void ClusterManager::startDiscoveryPublishingTimerIfNotRunning()
{
    if (!isRunning())
    {
        startPeriodic(MonotonicDuration::fromMSec(Discovery::BROADCASTING_INTERVAL_MS));
    }
}

int ClusterManager::init(const uint8_t init_cluster_size)
{
    /*
     * Figuring out the cluster size
     */
    if (init_cluster_size == ClusterSizeUnknown)
    {
        // Reading from the storage
        MarshallingStorageDecorator io(storage_);
        uint32_t value = 0;
        int res = io.get(getStorageKeyForClusterSize(), value);
        if (res < 0)
        {
            UAVCAN_TRACE("dynamic_node_id_server_impl::ClusterManager",
                         "Cluster size is neither configured nor stored in the storage");
            return res;
        }
        if ((value == 0) || (value > MaxServers))
        {
            UAVCAN_TRACE("dynamic_node_id_server_impl::ClusterManager", "Cluster size is invalid");
            return -ErrFailure;
        }
        cluster_size_ = static_cast<uint8_t>(value);
    }
    else
    {
        if ((init_cluster_size == 0) || (init_cluster_size > MaxServers))
        {
            return -ErrInvalidParam;
        }
        cluster_size_ = init_cluster_size;

        // Writing the storage
        MarshallingStorageDecorator io(storage_);
        uint32_t value = init_cluster_size;
        int res = io.setAndGetBack(getStorageKeyForClusterSize(), value);
        if ((res < 0) || (value != init_cluster_size))
        {
            UAVCAN_TRACE("dynamic_node_id_server_impl::ClusterManager", "Failed to store cluster size");
            return -ErrFailure;
        }
    }

    tracer_.onEvent(TraceClusterSizeInited, cluster_size_);

    UAVCAN_ASSERT(cluster_size_ > 0);
    UAVCAN_ASSERT(cluster_size_ <= MaxServers);

    /*
     * Initializing pub/sub and timer
     */
    int res = discovery_pub_.init();
    if (res < 0)
    {
        return res;
    }

    res = discovery_sub_.start(DiscoveryCallback(this, &ClusterManager::handleDiscovery));
    if (res < 0)
    {
        return res;
    }

    startDiscoveryPublishingTimerIfNotRunning();

    /*
     * Misc
     */
    resetAllServerIndices();
    return 0;
}

NodeID ClusterManager::getRemoteServerNodeIDAtIndex(uint8_t index) const
{
    if (index < num_known_servers_)
    {
        return servers_[index].node_id;
    }
    return NodeID();
}

Log::Index ClusterManager::getServerNextIndex(NodeID server_node_id) const
{
    const Server* const s = findServer(server_node_id);
    if (s != NULL)
    {
        return s->next_index;
    }
    UAVCAN_ASSERT(0);
    return 0;
}

void ClusterManager::incrementServerNextIndexBy(NodeID server_node_id, Log::Index increment)
{
    Server* const s = findServer(server_node_id);
    if (s != NULL)
    {
        s->next_index = Log::Index(s->next_index + increment);
    }
    else
    {
        UAVCAN_ASSERT(0);
    }
}

void ClusterManager::decrementServerNextIndex(NodeID server_node_id)
{
    Server* const s = findServer(server_node_id);
    if (s != NULL)
    {
        s->next_index--;
    }
    else
    {
        UAVCAN_ASSERT(0);
    }
}

Log::Index ClusterManager::getServerMatchIndex(NodeID server_node_id) const
{
    const Server* const s = findServer(server_node_id);
    if (s != NULL)
    {
        return s->match_index;
    }
    UAVCAN_ASSERT(0);
    return 0;
}

void ClusterManager::setServerMatchIndex(NodeID server_node_id, Log::Index match_index)
{
    Server* const s = findServer(server_node_id);
    if (s != NULL)
    {
        s->match_index = match_index;
    }
    else
    {
        UAVCAN_ASSERT(0);
    }
}

void ClusterManager::resetAllServerIndices()
{
    for (uint8_t i = 0; i < num_known_servers_; i++)
    {
        UAVCAN_ASSERT(servers_[i].node_id.isUnicast());
        servers_[i].resetIndices(log_);
    }
}

/*
 * RaftCore
 */
void RaftCore::updateFollower(const MonotonicTime& current_time)
{
    (void)current_time;
}

void RaftCore::updateCandidate(const MonotonicTime& current_time)
{
    (void)current_time;
}

void RaftCore::updateLeader(const MonotonicTime& current_time)
{
    (void)current_time;
}

void RaftCore::switchState(ServerState new_state)
{
    if (server_state_ != new_state)
    {
        trace(TraceRaftStateSwitch, new_state);
    }
}

void RaftCore::handleAppendEntriesRequest(const ReceivedDataStructure<AppendEntries::Request>& request,
                                          ServiceResponseDataStructure<AppendEntries::Response>& response)
{
    (void)request;
    (void)response;
}

void RaftCore::handleAppendEntriesResponse(const ServiceCallResult<AppendEntries>& result)
{
    (void)result;
}

void RaftCore::handleRequestVoteRequest(const ReceivedDataStructure<RequestVote::Request>& request,
                                        ServiceResponseDataStructure<RequestVote::Response>& response)
{
    (void)request;
    (void)response;
}

void RaftCore::handleRequestVoteResponse(const ServiceCallResult<RequestVote>& result)
{
    (void)result;
}

void RaftCore::handleTimerEvent(const TimerEvent& event)
{
    switch (server_state_)
    {
    case ServerStateFollower:
    {
        updateFollower(event.real_time);
        break;
    }
    case ServerStateCandidate:
    {
        updateCandidate(event.real_time);
        break;
    }
    case ServerStateLeader:
    {
        updateLeader(event.real_time);
        break;
    }
    default:
    {
        UAVCAN_ASSERT(0);
        break;
    }
    }
}

int RaftCore::init(uint8_t cluster_size)
{
    /*
     * Initializing state variables
     */
    last_activity_timestamp_ = getNode().getMonotonicTime();
    active_mode_ = true;
    server_state_ = ServerStateFollower;
    next_server_index_ = 0;
    num_votes_received_in_this_campaign_ = 0;
    commit_index_ = 0;

    /*
     * Initializing internals
     */
    int res = persistent_state_.init();
    if (res < 0)
    {
        return res;
    }

    res = cluster_.init(cluster_size);
    if (res < 0)
    {
        return res;
    }

    res = append_entries_srv_.start(AppendEntriesCallback(this, &RaftCore::handleAppendEntriesRequest));
    if (res < 0)
    {
        return res;
    }

    res = request_vote_srv_.start(RequestVoteCallback(this, &RaftCore::handleRequestVoteRequest));
    if (res < 0)
    {
        return res;
    }

    res = append_entries_client_.init();
    if (res < 0)
    {
        return res;
    }
    append_entries_client_.setRequestTimeout(getUpdateInterval());

    res = request_vote_client_.init();
    if (res < 0)
    {
        return res;
    }
    request_vote_client_.setRequestTimeout(getUpdateInterval());

    startPeriodic(getUpdateInterval());

    trace(TraceRaftCoreInited, getUpdateInterval().toUSec());

    UAVCAN_ASSERT(res >= 0);
    return 0;
}

int RaftCore::appendLog(const Entry& entry)
{
    if (isLeader())
    {
        trace(TraceRaftNewLogEntry, entry.node_id);
        return persistent_state_.getLog().append(entry);
    }
    else
    {
        UAVCAN_ASSERT(0);
        return -ErrLogic;
    }
}

} // dynamic_node_id_server_impl

}
