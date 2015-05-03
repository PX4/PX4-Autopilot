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
                                          protocol::dynamic_node_id::server::Entry::FieldTypes::unique_id& inout_value)
{
    IDynamicNodeIDStorageBackend::String serialized;
    for (uint8_t i = 0; i < protocol::dynamic_node_id::server::Entry::FieldTypes::unique_id::MaxSize; i++)
    {
        serialized.appendFormatted("%02x", inout_value.at(i));
    }
    UAVCAN_ASSERT(serialized.size() == protocol::dynamic_node_id::server::Entry::FieldTypes::unique_id::MaxSize * 2);

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
                                     protocol::dynamic_node_id::server::Entry::FieldTypes::unique_id& out_value) const
{
    static const uint8_t NumBytes = protocol::dynamic_node_id::server::Entry::FieldTypes::unique_id::MaxSize;

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

int Log::readEntryFromStorage(Index index, protocol::dynamic_node_id::server::Entry& out_entry)
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

int Log::writeEntryToStorage(Index index, const protocol::dynamic_node_id::server::Entry& entry)
{
    protocol::dynamic_node_id::server::Entry temp = entry;

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

int Log::init()
{
    const MarshallingStorageDecorator io(storage_);

    // Reading max index
    {
        uint32_t value = 0;
        if (io.get(getLastIndexKey(), value) < 0)
        {
            if (storage_.get(getLastIndexKey()).empty())
            {
                // It appears like there's no log in the storage - initializing empty log then
                last_index_ = 0;
                UAVCAN_TRACE("dynamic_node_id_server_impl::Log", "Initializing empty log");
                return 0;
            }
            else
            {
                // There's some data in the storage, but it cannot be parsed - reporting an error
                return -ErrFailure;
            }
        }
        if (value >= Capacity)
        {
            return -ErrFailure;
        }
        last_index_ = Index(value);
    }

    // Restoring log entries
    for (Index index = 0; index <= last_index_; index++)
    {
        const int result = readEntryFromStorage(index, entries_[index]);
        if (result < 0)
        {
            return result;
        }
    }

    UAVCAN_TRACE("dynamic_node_id_server_impl::Log", "Restored %u log entries", unsigned(last_index_));
    return 0;
}

int Log::append(const protocol::dynamic_node_id::server::Entry& entry)
{
    if ((last_index_ + 1) >= Capacity)
    {
        return -ErrLogic;
    }

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

const protocol::dynamic_node_id::server::Entry* Log::getEntryAtIndex(Index index) const
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

} // dynamic_node_id_server_impl

}
