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

    if (((index) >= Capacity) || (index <= 0))
    {
        return -ErrLogic;
    }

    uint32_t new_last_index = index - 1U;

    tracer_.onEvent(TraceLogRemove, new_last_index);

    if (new_last_index != last_index_)
    {
        MarshallingStorageDecorator io(storage_);
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
    }

    // Removal operation leaves dangling entries in storage, it's OK
    return 0;
}

int Log::removeEntriesWhereIndexGreater(Index index)
{
    return removeEntriesWhereIndexGreaterOrEqual(Index(index + 1U));
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

void ClusterManager::addServer(NodeID node_id)
{
    UAVCAN_ASSERT((num_known_servers_ + 1) < (MaxClusterSize - 2));
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
        tracer_.onEvent(TraceInvalidClusterSizeReceived, msg.configured_cluster_size);
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
        if ((value == 0) || (value > MaxClusterSize))
        {
            UAVCAN_TRACE("dynamic_node_id_server_impl::ClusterManager", "Cluster size is invalid");
            return -ErrFailure;
        }
        cluster_size_ = static_cast<uint8_t>(value);
    }
    else
    {
        if ((init_cluster_size == 0) || (init_cluster_size > MaxClusterSize))
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
    UAVCAN_ASSERT(cluster_size_ <= MaxClusterSize);

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
bool RaftCore::isActivityTimedOut() const
{
    const int multiplier = static_cast<int>(getNode().getNodeID().get()) - 1;

    const MonotonicDuration activity_timeout =
        MonotonicDuration::fromUSec(base_activity_timeout_.toUSec() + update_interval_.toUSec() * multiplier);

    return getNode().getMonotonicTime() > (last_activity_timestamp_ + activity_timeout);
}

void RaftCore::handlePersistentStateUpdateError(int error)
{
    UAVCAN_ASSERT(error < 0);
    trace(TraceRaftPersistStateUpdateError, error);
    switchState(ServerStateFollower);
    setActiveMode(false);               // Goodnight sweet prince
    registerActivity();                 // Deferring reelections
}

void RaftCore::updateFollower()
{
    if (active_mode_ && isActivityTimedOut())
    {
        switchState(ServerStateCandidate);
        setActiveMode(true);
        registerActivity();
    }
}

void RaftCore::updateCandidate()
{
    if (num_votes_received_in_this_campaign_ > 0)
    {
        const bool won = num_votes_received_in_this_campaign_ >= cluster_.getQuorumSize();

        UAVCAN_TRACE("dynamic_node_id_server_impl::RaftCore", "Election complete, won: %d", int(won));

        switchState(won ? ServerStateLeader : ServerStateFollower);       // Start over or become leader
        setActiveMode(true);
    }
    else
    {
        // Set votedFor, abort on failure
        int res = persistent_state_.setVotedFor(getNode().getNodeID());
        if (res < 0)
        {
            handlePersistentStateUpdateError(res);
            return;
        }

        // Increment current term, abort on failure
        res = persistent_state_.setCurrentTerm(persistent_state_.getCurrentTerm() + 1U);
        if (res < 0)
        {
            handlePersistentStateUpdateError(res);
            return;
        }

        num_votes_received_in_this_campaign_ = 1;               // Voting for self

        RequestVote::Request req;
        req.last_log_index = persistent_state_.getLog().getLastIndex();
        req.last_log_term = persistent_state_.getLog().getEntryAtIndex(req.last_log_index)->term;
        req.term = persistent_state_.getCurrentTerm();

        for (uint8_t i = 0; i < NumRequestVoteClients; i++)
        {
            const NodeID node_id = cluster_.getRemoteServerNodeIDAtIndex(i);
            if (!node_id.isUnicast())
            {
                break;
            }

            UAVCAN_TRACE("dynamic_node_id_server_impl::RaftCore", "Requesting vote from %d", int(node_id.get()));
            trace(TraceRaftVoteRequestInitiation, node_id.get());

            res = request_vote_clients_[i]->call(node_id, req);
            if (res < 0)
            {
                trace(TraceError, res);
            }
        }
    }
}

void RaftCore::updateLeader()
{
    propagateCommitIndex();

    // Leader simply emits one AppendEntry at every update, iterating over all available servers
    if (next_server_index_ >= cluster_.getClusterSize())
    {
        next_server_index_ = 0;
    }

    const NodeID node_id = cluster_.getRemoteServerNodeIDAtIndex(next_server_index_);
    UAVCAN_ASSERT(node_id.isUnicast());

    AppendEntries::Request req;
    req.term = persistent_state_.getCurrentTerm();
    req.leader_commit = commit_index_;

    req.prev_log_index = Log::Index(cluster_.getServerNextIndex(node_id) - 1U);

    const Entry* const entry = persistent_state_.getLog().getEntryAtIndex(req.prev_log_index);
    if (entry == NULL)
    {
        UAVCAN_ASSERT(0);
        handlePersistentStateUpdateError(-ErrLogic);
        return;
    }

    req.prev_log_term = entry->term;

    for (Log::Index index = cluster_.getServerNextIndex(node_id);
         index <= persistent_state_.getLog().getLastIndex();
         index++)
    {
        req.entries.push_back(*persistent_state_.getLog().getEntryAtIndex(index));
        if (req.entries.size() == req.entries.capacity())
        {
            break;
        }
    }

    pending_append_entries_fields_.num_entries = req.entries.size();
    pending_append_entries_fields_.prev_log_index = req.prev_log_index;

    const int res = append_entries_client_.call(node_id, req);
    if (res < 0)
    {
        trace(TraceRaftAppendEntriesCallFailure, res);
    }
}

void RaftCore::switchState(const ServerState new_state)
{
    if (server_state_ != new_state)
    {
        UAVCAN_TRACE("dynamic_node_id_server_impl::RaftCore", "State switch: %d --> %d",
                     int(server_state_), int(new_state));
        trace(TraceRaftStateSwitch, new_state);

        server_state_ = new_state;

        cluster_.resetAllServerIndices();

        next_server_index_ = 0;
        num_votes_received_in_this_campaign_ = 0;

        for (uint8_t i = 0; i < NumRequestVoteClients; i++)
        {
            request_vote_clients_[i]->cancel();
        }
        append_entries_client_.cancel();
    }
}

void RaftCore::setActiveMode(const bool new_active)
{
    if (active_mode_ != new_active)
    {
        UAVCAN_TRACE("dynamic_node_id_server_impl::RaftCore", "Mode switch: %d --> %d",
                     int(active_mode_), int(new_active));
        trace(TraceRaftModeSwitch, new_active);

        active_mode_ = new_active;
    }
}

void RaftCore::tryIncrementCurrentTermFromResponse(Term new_term)
{
    trace(TraceRaftNewerTermInResponse, new_term);
    const int res = persistent_state_.setCurrentTerm(new_term);
    if (res < 0)
    {
        trace(TraceRaftPersistStateUpdateError, res);
    }
    registerActivity();                             // Deferring future elections
    switchState(ServerStateFollower);
    setActiveMode(false);
}

void RaftCore::propagateCommitIndex()
{
    // Objective is to estimate whether we can safely increment commit index value
    UAVCAN_ASSERT(server_state_ == ServerStateLeader);
    UAVCAN_ASSERT(commit_index_ <= persistent_state_.getLog().getLastIndex());

    if (commit_index_ == persistent_state_.getLog().getLastIndex())
    {
        // All local entries are committed
        bool commit_index_fully_replicated = true;

        for (uint8_t i = 0; i < cluster_.getNumKnownServers(); i++)
        {
            const Log::Index match_index = cluster_.getServerMatchIndex(cluster_.getRemoteServerNodeIDAtIndex(i));
            if (match_index != commit_index_)
            {
                commit_index_fully_replicated = false;
                break;
            }
        }

        if (commit_index_fully_replicated && cluster_.isClusterDiscovered())
        {
            setActiveMode(false);  // Commit index is the same on all nodes, enabling passive mode
        }
    }
    else
    {
        // Not all local entries are committed
        setActiveMode(true);

        uint8_t num_nodes_with_next_log_entry_available = 1; // Local node
        for (uint8_t i = 0; i < cluster_.getNumKnownServers(); i++)
        {
            const Log::Index match_index = cluster_.getServerMatchIndex(cluster_.getRemoteServerNodeIDAtIndex(i));
            if (match_index > commit_index_)
            {
                num_nodes_with_next_log_entry_available++;
            }
        }

        if (num_nodes_with_next_log_entry_available >= cluster_.getQuorumSize())
        {
            commit_index_++;
            UAVCAN_ASSERT(commit_index_ > 0);   // Index 0 is always committed
            trace(TraceRaftNewEntryCommitted, commit_index_);

            // AT THIS POINT ALLOCATION IS COMPLETE
            log_commit_handler_.onEntryCommitted(*persistent_state_.getLog().getEntryAtIndex(commit_index_));
        }
    }
}

void RaftCore::handleAppendEntriesRequest(const ReceivedDataStructure<AppendEntries::Request>& request,
                                          ServiceResponseDataStructure<AppendEntries::Response>& response)
{
    if (!cluster_.isKnownServer(request.getSrcNodeID()))
    {
        trace(TraceRaftRequestIgnored, request.getSrcNodeID().get());
        return;
    }

    registerActivity();

    UAVCAN_ASSERT(response.isResponseEnabled());  // This is default

    /*
     * Checking if our current state is up to date.
     * The request will be ignored if persistent state cannot be updated.
     */
    if (request.term > persistent_state_.getCurrentTerm())
    {
        int res = persistent_state_.setCurrentTerm(request.term);
        if (res < 0)
        {
            response.setResponseEnabled(false);
            trace(TraceRaftPersistStateUpdateError, res);
        }

        res = persistent_state_.resetVotedFor();
        if (res < 0)
        {
            response.setResponseEnabled(false);
            trace(TraceRaftPersistStateUpdateError, res);
        }

        switchState(ServerStateFollower);
        setActiveMode(false);

        if (!response.isResponseEnabled())
        {
            return;
        }
    }

    /*
     * Preparing the response
     */
    response.term = persistent_state_.getCurrentTerm();
    response.success = false;

    /*
     * Step 1 (see Raft paper)
     * Reject the request if the leader has stale term number.
     */
    if (request.term < persistent_state_.getCurrentTerm())
    {
        response.setResponseEnabled(true);
        return;
    }

    switchState(ServerStateFollower);
    setActiveMode(false);

    /*
     * Step 2
     * Reject the request if the assumed log index does not exist on the local node.
     */
    const Entry* const prev_entry = persistent_state_.getLog().getEntryAtIndex(request.prev_log_index);
    if (prev_entry == NULL)
    {
        response.setResponseEnabled(true);
        return;
    }

    /*
     * Step 3
     * Drop log entries if term number does not match.
     * Ignore the request if the persistent state cannot be updated.
     */
    if (prev_entry->term != request.prev_log_term)
    {
        const int res = persistent_state_.getLog().removeEntriesWhereIndexGreaterOrEqual(request.prev_log_index);
        response.setResponseEnabled(res >= 0);
        if (res < 0)
        {
            trace(TraceRaftPersistStateUpdateError, res);
        }
        return;
    }

    /*
     * Step 4
     * Update the log with new entries - this will possibly require to rewrite existing entries.
     * Ignore the request if the persistent state cannot be updated.
     */
    if (request.prev_log_index != persistent_state_.getLog().getLastIndex())
    {
        const int res = persistent_state_.getLog().removeEntriesWhereIndexGreater(request.prev_log_index);
        if (res < 0)
        {
            trace(TraceRaftPersistStateUpdateError, res);
            response.setResponseEnabled(false);
            return;
        }
    }

    for (uint8_t i = 0; i < request.entries.size(); i++)
    {
        const int res = persistent_state_.getLog().append(request.entries[i]);
        if (res < 0)
        {
            trace(TraceRaftPersistStateUpdateError, res);
            response.setResponseEnabled(false);
            return;                     // Response will not be sent, the server will assume that we're dead
        }
    }

    /*
     * Step 5
     * Update the commit index.
     */
    if (request.leader_commit > commit_index_)
    {
        commit_index_ = min(request.leader_commit, persistent_state_.getLog().getLastIndex());
        trace(TraceRaftCommitIndexUpdate, commit_index_);
    }

    response.setResponseEnabled(true);
    response.success = true;
}

void RaftCore::handleAppendEntriesResponse(const ServiceCallResult<AppendEntries>& result)
{
    UAVCAN_ASSERT(server_state_ == ServerStateLeader);          // When state switches, all requests must be cancelled

    if (!result.isSuccessful())
    {
        return;
    }

    if (result.response.term > persistent_state_.getCurrentTerm())
    {
        tryIncrementCurrentTermFromResponse(result.response.term);
    }
    else
    {
        if (result.response.success)
        {
            cluster_.incrementServerNextIndexBy(result.server_node_id, pending_append_entries_fields_.num_entries);
            cluster_.setServerMatchIndex(result.server_node_id,
                                         Log::Index(pending_append_entries_fields_.prev_log_index +
                                                    pending_append_entries_fields_.num_entries));
        }
        else
        {
            cluster_.decrementServerNextIndex(result.server_node_id);
        }
    }

    pending_append_entries_fields_ = PendingAppendEntriesFields();
    // Rest of the logic is implemented in periodic update handlers.
}

void RaftCore::handleRequestVoteRequest(const ReceivedDataStructure<RequestVote::Request>& request,
                                        ServiceResponseDataStructure<RequestVote::Response>& response)
{
    trace(TraceRaftVoteRequestReceived, request.getSrcNodeID().get());

    if (!cluster_.isKnownServer(request.getSrcNodeID()))
    {
        trace(TraceRaftRequestIgnored, request.getSrcNodeID().get());
        return;
    }

    UAVCAN_ASSERT(response.isResponseEnabled());  // This is default

    setActiveMode(true);

    /*
     * Checking if our current state is up to date.
     * The request will be ignored if persistent state cannot be updated.
     */
    if (request.term > persistent_state_.getCurrentTerm())
    {
        int res = persistent_state_.setCurrentTerm(request.term);
        if (res < 0)
        {
            response.setResponseEnabled(false);
            trace(TraceRaftPersistStateUpdateError, res);
        }

        res = persistent_state_.resetVotedFor();
        if (res < 0)
        {
            response.setResponseEnabled(false);
            trace(TraceRaftPersistStateUpdateError, res);
        }

        switchState(ServerStateFollower);
        setActiveMode(false);

        if (!response.isResponseEnabled())
        {
            return;
        }
    }

    /*
     * Preparing the response
     */
    response.term = persistent_state_.getCurrentTerm();

    if (request.term < response.term)
    {
        response.vote_granted = false;
    }
    else
    {
        const bool can_vote = !persistent_state_.isVotedForSet() ||
                              (persistent_state_.getVotedFor() == request.getSrcNodeID());
        const bool log_is_up_to_date =
            persistent_state_.getLog().isOtherLogUpToDate(request.last_log_index, request.last_log_term);

        response.vote_granted = can_vote && log_is_up_to_date;

        if (response.vote_granted)
        {
            registerActivity();                 // This is necessary to avoid excessive elections

            const int res = persistent_state_.setVotedFor(request.getSrcNodeID());
            if (res < 0)
            {
                trace(TraceRaftPersistStateUpdateError, res);
                response.setResponseEnabled(false);
                return;
            }
        }
    }
}

void RaftCore::handleRequestVoteResponse(const ServiceCallResult<RequestVote>& result)
{
    UAVCAN_ASSERT(server_state_ == ServerStateCandidate);       // When state switches, all requests must be cancelled

    if (!result.isSuccessful())
    {
        return;
    }

    trace(TraceRaftVoteRequestSucceeded, result.server_node_id.get());

    if (result.response.term > persistent_state_.getCurrentTerm())
    {
        tryIncrementCurrentTermFromResponse(result.response.term);
    }
    else
    {
        if (result.response.vote_granted)
        {
            num_votes_received_in_this_campaign_++;
        }
    }
    // Rest of the logic is implemented in periodic update handlers.
    // I'm no fan of asynchronous programming. At all.
}

void RaftCore::handleTimerEvent(const TimerEvent&)
{
    if (cluster_.hadDiscoveryActivity() && isLeader())
    {
        setActiveMode(true);
    }

    switch (server_state_)
    {
    case ServerStateFollower:
    {
        updateFollower();
        break;
    }
    case ServerStateCandidate:
    {
        updateCandidate();
        break;
    }
    case ServerStateLeader:
    {
        updateLeader();
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
    append_entries_client_.setCallback(AppendEntriesResponseCallback(this, &RaftCore::handleAppendEntriesResponse));
    append_entries_client_.setRequestTimeout(update_interval_);

    for (uint8_t i = 0; i < NumRequestVoteClients; i++)
    {
        res = request_vote_clients_[i]->init();
        if (res < 0)
        {
            return res;
        }
        request_vote_clients_[i]->setCallback(RequestVoteResponseCallback(this, &RaftCore::handleRequestVoteResponse));
        request_vote_clients_[i]->setRequestTimeout(update_interval_);
    }

    startPeriodic(update_interval_);

    trace(TraceRaftCoreInited, update_interval_.toUSec());

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
