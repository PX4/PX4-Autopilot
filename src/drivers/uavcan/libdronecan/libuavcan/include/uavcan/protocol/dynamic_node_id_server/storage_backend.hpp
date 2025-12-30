/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_STORAGE_BACKEND_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_STORAGE_BACKEND_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/error.hpp>
#include <uavcan/marshal/types.hpp>

#include <stdint.h>

namespace uavcan
{
namespace dynamic_node_id_server
{
/**
 * This interface is used by the server to read and write stable storage.
 * The storage is represented as a key-value container, where keys and values are ASCII strings up to 32
 * characters long, not including the termination byte. Fixed block size allows for absolutely straightforward
 * and efficient implementation of storage backends, e.g. based on text files.
 * Keys and values may contain only non-whitespace, non-formatting printable characters.
 */
class UAVCAN_EXPORT IStorageBackend
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
    enum { MaxKeyValuePairs = 512 };

    /**
     * This type is used to exchange data chunks with the backend.
     * It doesn't use any dynamic memory; please refer to the Array<> class for details.
     */
    typedef MakeString<MaxStringLength>::Type String;

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

    /**
     * Optional extension: enumerate all keys currently present in the backend.
     *
     * This is used for storage migration and maintenance tasks (e.g., rebuilding reverse indices).
     * Implementations that cannot support enumeration may keep the default implementation.
     */
    typedef void (*ForEachKeyCallback)(const String& key, void* user_data);

    virtual int forEachKey(ForEachKeyCallback cb, void* user_data) const
    {
        (void)cb;
        (void)user_data;
        return -ErrFailure;
    }

    /**
     * Optional extension: return an approximate "last update" timestamp for the given key.
     * Units: microseconds since Unix epoch.
     *
     * Implementations that cannot support timestamps may keep the default implementation.
     */
    virtual int getKeyUpdateTimeUSec(const String& key, uint64_t& out_time_usec) const
    {
        (void)key;
        out_time_usec = 0;
        return -ErrFailure;
    }

    virtual ~IStorageBackend() { }
};

}
}

#endif // Include guard
