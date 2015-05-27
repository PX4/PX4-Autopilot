/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_CENTRALIZED_STORAGE_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_CENTRALIZED_STORAGE_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/protocol/dynamic_node_id_server/storage_marshaller.hpp>
#include <uavcan/protocol/dynamic_node_id_server/event.hpp>

namespace uavcan
{
namespace dynamic_node_id_server
{
namespace centralized
{
/**
 * This class transparently replicates its state to the storage backend, keeping the most recent state in memory.
 * Writes are slow, reads are instantaneous.
 */
class Storage
{
public:
    typedef uint8_t Size;

    enum { Capacity = NodeID::Max };

    struct Entry
    {
        UniqueID unique_id;
        NodeID node_id;

        Entry() { }

        Entry(const NodeID nid, const UniqueID& uid)
            : unique_id(uid)
            , node_id(nid)
        { }

        bool operator==(const Entry& rhs) const
        {
            return unique_id == rhs.unique_id &&
                   node_id   == rhs.node_id;
        }
    };

private:
    IStorageBackend& storage_;
    Entry entries_[Capacity];
    Size size_;

    static IStorageBackend::String getSizeKey() { return "size"; }

    static IStorageBackend::String makeEntryKey(Size index, const char* postfix)
    {
        IStorageBackend::String str;
        // "0_foobar"
        str.appendFormatted("%d", int(index));
        str += "_";
        str += postfix;
        return str;
    }

    int readEntryFromStorage(Size index, Entry& out_entry)
    {
        const StorageMarshaller io(storage_);

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
        if (node_id > NodeID::Max || node_id == 0)
        {
            return -ErrFailure;
        }
        out_entry.node_id = NodeID(static_cast<uint8_t>(node_id));

        return 0;
    }

    int writeEntryToStorage(Size index, const Entry& entry)
    {
        Entry temp = entry;

        StorageMarshaller io(storage_);

        // Unique ID
        if (io.setAndGetBack(makeEntryKey(index, "unique_id"), temp.unique_id) < 0)
        {
            return -ErrFailure;
        }

        // Node ID
        uint32_t node_id = entry.node_id.get();
        if (io.setAndGetBack(makeEntryKey(index, "node_id"), node_id) < 0)
        {
            return -ErrFailure;
        }
        temp.node_id = NodeID(static_cast<uint8_t>(node_id));

        return (temp == entry) ? 0 : -ErrFailure;
    }

public:
    Storage(IStorageBackend& storage)
        : storage_(storage)
        , size_(0)
    { }

    /**
     * This method reads all entries from the storage.
     */
    int init()
    {
        StorageMarshaller io(storage_);

        // Reading size
        size_ = 0;
        {
            uint32_t value = 0;
            if (io.get(getSizeKey(), value) < 0)
            {
                if (storage_.get(getSizeKey()).empty())
                {
                    int res = io.setAndGetBack(getSizeKey(), value);
                    if (res < 0)
                    {
                        return res;
                    }
                    return (value == 0) ? 0 : -ErrFailure;
                }
                else
                {
                    // There's some data in the storage, but it cannot be parsed - reporting an error
                    return -ErrFailure;
                }
            }

            if (value > Capacity)
            {
                return -ErrFailure;
            }

            size_ = Size(value);
        }

        // Restoring entries
        for (Size index = 0; index < size_; index++)
        {
            const int result = readEntryFromStorage(index, entries_[index]);
            if (result < 0)
            {
                return result;
            }
        }

        return 0;
    }

    /**
     * This method invokes storage IO.
     * Returned value indicates whether the entry was successfully appended.
     */
    int add(const NodeID node_id, const UniqueID& unique_id)
    {
        if (size_ == Capacity)
        {
            return -ErrLogic;
        }

        if (!node_id.isUnicast())
        {
            return -ErrInvalidParam;
        }

        Entry entry;
        entry.node_id = node_id;
        entry.unique_id = unique_id;

        // If next operations fail, we'll get a dangling entry, but it's absolutely OK.
        int res = writeEntryToStorage(size_, entry);
        if (res < 0)
        {
            return res;
        }

        // Updating the size
        StorageMarshaller io(storage_);
        uint32_t new_size_index = size_ + 1U;
        res = io.setAndGetBack(getSizeKey(), new_size_index);
        if (res < 0)
        {
            return res;
        }
        if (new_size_index != size_ + 1U)
        {
            return -ErrFailure;
        }

        entries_[size_] = entry;
        size_++;

        return 0;
    }

    /**
     * Returns nullptr if there's no such entry.
     */
    const Entry* findByNodeID(const NodeID node_id) const
    {
        for (Size i = 0; i < size_; i++)
        {
            if (entries_[i].node_id == node_id)
            {
                return &entries_[i];
            }
        }
        return NULL;
    }

    /**
     * Returns nullptr if there's no such entry.
     */
    const Entry* findByUniqueID(const UniqueID& unique_id) const
    {
        for (Size i = 0; i < size_; i++)
        {
            if (entries_[i].unique_id == unique_id)
            {
                return &entries_[i];
            }
        }
        return NULL;
    }

    Size getSize() const { return size_; }
};

}
}
}

#endif // Include guard
