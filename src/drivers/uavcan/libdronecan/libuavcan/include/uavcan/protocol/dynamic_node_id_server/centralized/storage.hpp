/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_CENTRALIZED_STORAGE_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_CENTRALIZED_STORAGE_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/protocol/dynamic_node_id_server/storage_marshaller.hpp>
#include <uavcan/protocol/dynamic_node_id_server/event.hpp>
#include <uavcan/util/bitset.hpp>

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
    typedef BitSet<NodeID::Max + 1> OccupationMask;
    typedef Array<IntegerSpec<8, SignednessUnsigned, CastModeSaturate>, ArrayModeStatic,
                  BitLenToByteLen<NodeID::Max + 1>::Result>
            OccupationMaskArray;

    IStorageBackend& storage_;
    OccupationMask occupation_mask_;

    struct RebuildCtx
    {
        Storage* self;
        bool have_uid[NodeID::Max + 1];
        UniqueID uid[NodeID::Max + 1];
        uint64_t t[NodeID::Max + 1];
    };

    static IStorageBackend::String getOccupationMaskKey() { return "occupation_mask"; }
    static IStorageBackend::String getAllocHeadKey() { return "alloc_head"; }
    static IStorageBackend::String getAllocTailKey() { return "alloc_tail"; }

    static IStorageBackend::String makeNodeUIDKey(const NodeID node_id)
    {
        IStorageBackend::String key;
        key.appendFormatted("nid%03u_uid", static_cast<unsigned>(node_id.get()));
        return key;
    }

    static IStorageBackend::String makeNodePrevKey(const NodeID node_id)
    {
        IStorageBackend::String key;
        key.appendFormatted("nid%03u_prev", static_cast<unsigned>(node_id.get()));
        return key;
    }

    static IStorageBackend::String makeNodeNextKey(const NodeID node_id)
    {
        IStorageBackend::String key;
        key.appendFormatted("nid%03u_next", static_cast<unsigned>(node_id.get()));
        return key;
    }

    static bool isHexUniqueIDKey(const IStorageBackend::String& key)
    {
        // UniqueID::MaxSize is 16 bytes -> 32 hex chars
        if (key.size() != UniqueID::MaxSize * 2)
        {
            return false;
        }

        for (IStorageBackend::String::const_iterator it = key.begin(); it != key.end(); ++it)
        {
            const char c = static_cast<char>(*it);
            const bool is_digit = (c >= '0' && c <= '9');
            const bool is_hex_lower = (c >= 'a' && c <= 'f');
            const bool is_hex_upper = (c >= 'A' && c <= 'F');
            if (!(is_digit || is_hex_lower || is_hex_upper))
            {
                return false;
            }
        }
        return true;
    }

    static uint8_t hexNibble(char c)
    {
        if (c >= '0' && c <= '9')
        {
            return static_cast<uint8_t>(c - '0');
        }
        if (c >= 'a' && c <= 'f')
        {
            return static_cast<uint8_t>(c - 'a' + 10);
        }
        return static_cast<uint8_t>(c - 'A' + 10);
    }

    static int parseUniqueIDFromHexKey(const IStorageBackend::String& key, UniqueID& out_uid)
    {
        if (!isHexUniqueIDKey(key))
        {
            return -ErrInvalidParam;
        }

        for (uint8_t i = 0; i < UniqueID::MaxSize; i++)
        {
            const IStorageBackend::String::SizeType hi_index =
                static_cast<IStorageBackend::String::SizeType>(static_cast<unsigned>(i) * 2U);
            const IStorageBackend::String::SizeType lo_index =
                static_cast<IStorageBackend::String::SizeType>(static_cast<unsigned>(i) * 2U + 1U);
            const char hi = static_cast<char>(key.at(hi_index));
            const char lo = static_cast<char>(key.at(lo_index));
            out_uid[i] = static_cast<uint8_t>((hexNibble(hi) << 4) | hexNibble(lo));
        }
        return 0;
    }

    NodeID getAllocHeadNodeID() const
    {
        StorageMarshaller io(storage_);
        uint32_t v = 0;
        if (io.get(getAllocHeadKey(), v) < 0)
        {
            return NodeID();
        }
        return (v > 0 && v <= NodeID::Max) ? NodeID(static_cast<uint8_t>(v)) : NodeID();
    }

    NodeID getAllocTailNodeID() const
    {
        StorageMarshaller io(storage_);
        uint32_t v = 0;
        if (io.get(getAllocTailKey(), v) < 0)
        {
            return NodeID();
        }
        return (v > 0 && v <= NodeID::Max) ? NodeID(static_cast<uint8_t>(v)) : NodeID();
    }

    int setAllocHeadNodeID(const NodeID node_id)
    {
        StorageMarshaller io(storage_);
        uint32_t v = node_id.isValid() ? node_id.get() : 0;
        return io.setAndGetBack(getAllocHeadKey(), v);
    }

    int setAllocTailNodeID(const NodeID node_id)
    {
        StorageMarshaller io(storage_);
        uint32_t v = node_id.isValid() ? node_id.get() : 0;
        return io.setAndGetBack(getAllocTailKey(), v);
    }

    NodeID getLinkNodeID(const IStorageBackend::String& key) const
    {
        StorageMarshaller io(storage_);
        uint32_t v = 0;
        if (io.get(key, v) < 0)
        {
            return NodeID();
        }
        return (v > 0 && v <= NodeID::Max) ? NodeID(static_cast<uint8_t>(v)) : NodeID();
    }

    int setLinkNodeID(const IStorageBackend::String& key, const NodeID node_id)
    {
        StorageMarshaller io(storage_);
        uint32_t v = node_id.isValid() ? node_id.get() : 0;
        return io.setAndGetBack(key, v);
    }

    NodeID getPrevNodeID(const NodeID node_id) const { return getLinkNodeID(makeNodePrevKey(node_id)); }
    NodeID getNextNodeID(const NodeID node_id) const { return getLinkNodeID(makeNodeNextKey(node_id)); }
    int setPrevNodeID(const NodeID node_id, const NodeID prev) { return setLinkNodeID(makeNodePrevKey(node_id), prev); }
    int setNextNodeID(const NodeID node_id, const NodeID next) { return setLinkNodeID(makeNodeNextKey(node_id), next); }

    int getUniqueIDForNodeID(const NodeID node_id, UniqueID& out_uid) const
    {
        StorageMarshaller io(storage_);
        return io.get(makeNodeUIDKey(node_id), out_uid);
    }

    void deleteUniqueIDMapping(const UniqueID& unique_id)
    {
        storage_.set(StorageMarshaller::convertUniqueIDToHex(unique_id), IStorageBackend::String());
    }

    int unlinkFromList(const NodeID node_id)
    {
        const NodeID prev = getPrevNodeID(node_id);
        const NodeID next = getNextNodeID(node_id);

        if (prev.isValid())
        {
            if (setNextNodeID(prev, next) < 0)
            {
                return -ErrFailure;
            }
        }
        else
        {
            // node_id is head
            if (setAllocHeadNodeID(next) < 0)
            {
                return -ErrFailure;
            }
        }

        if (next.isValid())
        {
            if (setPrevNodeID(next, prev) < 0)
            {
                return -ErrFailure;
            }
        }
        else
        {
            // node_id is tail
            if (setAllocTailNodeID(prev) < 0)
            {
                return -ErrFailure;
            }
        }

        (void)setPrevNodeID(node_id, NodeID());
        (void)setNextNodeID(node_id, NodeID());
        return 0;
    }

    int appendToTail(const NodeID node_id)
    {
        const NodeID tail = getAllocTailNodeID();
        if (!tail.isValid())
        {
            // Empty list
            if (setAllocHeadNodeID(node_id) < 0 || setAllocTailNodeID(node_id) < 0)
            {
                return -ErrFailure;
            }
            (void)setPrevNodeID(node_id, NodeID());
            (void)setNextNodeID(node_id, NodeID());
            return 0;
        }

        if (setNextNodeID(tail, node_id) < 0)
        {
            return -ErrFailure;
        }
        if (setPrevNodeID(node_id, tail) < 0)
        {
            return -ErrFailure;
        }
        if (setNextNodeID(node_id, NodeID()) < 0)
        {
            return -ErrFailure;
        }
        if (setAllocTailNodeID(node_id) < 0)
        {
            return -ErrFailure;
        }
        return 0;
    }

    int moveToTail(const NodeID node_id)
    {
        const NodeID tail = getAllocTailNodeID();
        if (!tail.isValid() || tail == node_id)
        {
            return 0;
        }

        // If node isn't linked, treat as append.
        const NodeID prev = getPrevNodeID(node_id);
        const NodeID next = getNextNodeID(node_id);
        const NodeID head = getAllocHeadNodeID();
        const bool seems_linked = (prev.isValid() || next.isValid() || head == node_id);

        if (seems_linked)
        {
            const int res = unlinkFromList(node_id);
            if (res < 0)
            {
                return res;
            }
        }

        return appendToTail(node_id);
    }

    int rebuildAllocationListFromBackend()
    {
        // Best-effort migration: discover UniqueID->NodeID entries, order them by backend update time.
        // The server stores allocations under keys that are the hex-encoded unique IDs.
        RebuildCtx ctx;
        ctx.self = this;
        for (unsigned i = 0; i <= NodeID::Max; i++)
        {
            ctx.have_uid[i] = false;
            ctx.t[i] = 0;
        }

        const int enum_res = storage_.forEachKey(&Storage::forEachKeyTrampoline, &ctx);
        if (enum_res < 0)
        {
            return enum_res;
        }

        // Reset list first
        (void)setAllocHeadNodeID(NodeID());
        (void)setAllocTailNodeID(NodeID());

        // Build list in oldest->newest order.
        bool appended[NodeID::Max + 1];
        for (unsigned i = 0; i <= NodeID::Max; i++)
        {
            appended[i] = false;
            // Also ensure link keys are cleared (best-effort)
            if (i > 0)
            {
                const NodeID nid(static_cast<uint8_t>(i));
                (void)setPrevNodeID(nid, NodeID());
                (void)setNextNodeID(nid, NodeID());
            }
        }

        const unsigned num_occ = static_cast<unsigned>(occupation_mask_.count());
        for (unsigned k = 0; k < num_occ; k++)
        {
            uint64_t best_t = 0;
            uint8_t best_id = 0;
            bool best_set = false;

            for (uint8_t nid = 1; nid <= NodeID::Max; nid++)
            {
                if (!occupation_mask_[nid] || appended[nid])
                {
                    continue;
                }

                const uint64_t ts = ctx.have_uid[nid] ? ctx.t[nid] : 0ULL;
                if (!best_set || ts < best_t || (ts == best_t && nid < best_id))
                {
                    best_set = true;
                    best_t = ts;
                    best_id = nid;
                }
            }

            if (!best_set)
            {
                break;
            }

            appended[best_id] = true;
            const NodeID node_id(best_id);

            // Ensure reverse mapping exists when we know the UID
            if (ctx.have_uid[best_id])
            {
                StorageMarshaller io(storage_);
                UniqueID tmp = ctx.uid[best_id];
                (void)io.setAndGetBack(makeNodeUIDKey(node_id), tmp);
            }

            (void)appendToTail(node_id);
        }

        return 0;
    }

    static void forEachKeyTrampoline(const IStorageBackend::String& key, void* user_data)
    {
        RebuildCtx* ctx = static_cast<RebuildCtx*>(user_data);
        if (ctx == UAVCAN_NULLPTR || ctx->self == UAVCAN_NULLPTR)
        {
            return;
        }

        if (!isHexUniqueIDKey(key))
        {
            return;
        }

        StorageMarshaller io(ctx->self->storage_);
        uint32_t node_id_int = 0;
        if (io.get(key, node_id_int) < 0)
        {
            return;
        }

        if (node_id_int == 0 || node_id_int > NodeID::Max)
        {
            return;
        }

        const uint8_t node_id_u8 = static_cast<uint8_t>(node_id_int);
        UniqueID uid;
        if (parseUniqueIDFromHexKey(key, uid) < 0)
        {
            return;
        }

        uint64_t ts = 0;
        (void)ctx->self->storage_.getKeyUpdateTimeUSec(key, ts);

        // If multiple entries point to the same node ID, keep the oldest timestamp.
        if (!ctx->have_uid[node_id_u8] || ts < ctx->t[node_id_u8])
        {
            ctx->have_uid[node_id_u8] = true;
            ctx->uid[node_id_u8] = uid;
            ctx->t[node_id_u8] = ts;
        }
    }

    static void maskFromArray(const OccupationMaskArray& array, OccupationMask& out_mask)
    {
        for (uint8_t byte = 0; byte < array.size(); byte++)
        {
            for (uint8_t bit = 0; bit < 8; bit++)
            {
                out_mask[byte * 8U + bit] = (array[byte] & (1U << bit)) != 0;
            }
        }
    }

    static OccupationMaskArray maskToArray(const OccupationMask& mask)
    {
        OccupationMaskArray array;
        for (uint8_t byte = 0; byte < array.size(); byte++)
        {
            for (uint8_t bit = 0; bit < 8; bit++)
            {
                if (mask[byte * 8U + bit])
                {
                    array[byte] = static_cast<uint8_t>(array[byte] | (1U << bit));
                }
            }
        }
        return array;
    }

public:
    Storage(IStorageBackend& storage) :
        storage_(storage)
    { }

    /**
     * This method reads only the occupation mask from the storage.
     */
    int init()
    {
        StorageMarshaller io(storage_);
        OccupationMaskArray array;
        io.get(getOccupationMaskKey(), array);
        OccupationMask tmp;
        maskFromArray(array, tmp);
        occupation_mask_ = tmp;

        // If allocation list is missing, try to rebuild it from backend keys.
        // This enables oldest-first reuse and allows removal of evicted UniqueID->NodeID mappings.
        const bool list_present = !storage_.get(getAllocHeadKey()).empty() || !storage_.get(getAllocTailKey()).empty();
        if (!list_present && occupation_mask_.count() > 0)
        {
            (void)rebuildAllocationListFromBackend();
        }
        return 0;
    }

    /**
     * This method invokes storage IO.
     * Returned value indicates whether the entry was successfully appended.
     */
    int add(const NodeID node_id, const UniqueID& unique_id)
    {
        if (!node_id.isUnicast())
        {
            return -ErrInvalidParam;
        }

        StorageMarshaller io(storage_);

        // If this Node ID is already occupied, remove the old UniqueID->NodeID mapping if possible.
        if (occupation_mask_[node_id.get()])
        {
            UniqueID old_uid;
            if (getUniqueIDForNodeID(node_id, old_uid) >= 0)
            {
                deleteUniqueIDMapping(old_uid);
            }
        }

        // If next operations fail, we'll get a dangling entry, but it's absolutely OK.
        {
            uint32_t node_id_int = node_id.get();
            int res = io.setAndGetBack(StorageMarshaller::convertUniqueIDToHex(unique_id), node_id_int);
            if (res < 0)
            {
                return res;
            }
            if (node_id_int != node_id.get())
            {
                return -ErrFailure;
            }
        }

        // Store reverse mapping NodeID -> UniqueID for future eviction
        {
            UniqueID tmp = unique_id;
            const int res = io.setAndGetBack(makeNodeUIDKey(node_id), tmp);
            if (res < 0)
            {
                return res;
            }
        }

        // Updating the mask in the storage
        OccupationMask new_occupation_mask;
        new_occupation_mask = occupation_mask_;
        new_occupation_mask[node_id.get()] = true;
        OccupationMaskArray occupation_array = maskToArray(new_occupation_mask);

        int res = io.setAndGetBack(getOccupationMaskKey(), occupation_array);
        if (res < 0)
        {
            return res;
        }
        if (occupation_array != maskToArray(new_occupation_mask))
        {
            return -ErrFailure;
        }

        // Updating the cached mask only if the storage was updated successfully
        occupation_mask_ = new_occupation_mask;

        // Update allocation order: move the Node ID to the tail (newest).
        // If the list is not initialized for some reason, best-effort rebuild.
        if (!getAllocHeadNodeID().isValid() && occupation_mask_.count() > 0)
        {
            (void)rebuildAllocationListFromBackend();
        }
        (void)moveToTail(node_id);

        return 0;
    }

    /**
     * Returns the oldest allocated node ID, skipping the protected one.
     * Returns an invalid node ID if none is available.
     */
    NodeID getOldestAllocatedNodeIDForReuse(const NodeID protected_node_id) const
    {
        NodeID it = getAllocHeadNodeID();
        for (unsigned i = 0; i <= NodeID::Max && it.isValid(); i++)
        {
            if (it != protected_node_id)
            {
                return it;
            }
            it = getNextNodeID(it);
        }
        return NodeID();
    }

    /**
     * Reuse the oldest allocation (oldest->newest). The reused ID becomes the newest.
     */
    NodeID reuseOldestAllocatedNodeID(const NodeID protected_node_id, const UniqueID& unique_id)
    {
        const NodeID victim = getOldestAllocatedNodeIDForReuse(protected_node_id);
        if (!victim.isUnicast())
        {
            return NodeID();
        }

        const int res = add(victim, unique_id);
        if (res < 0)
        {
            return NodeID();
        }
        return victim;
    }

    /**
     * Returns an invalid node ID if there's no such allocation.
     */
    NodeID getNodeIDForUniqueID(const UniqueID& unique_id) const
    {
        StorageMarshaller io(storage_);
        uint32_t node_id = 0;
        io.get(StorageMarshaller::convertUniqueIDToHex(unique_id), node_id);
        return (node_id > 0 && node_id <= NodeID::Max) ? NodeID(static_cast<uint8_t>(node_id)) : NodeID();
    }

    bool isNodeIDOccupied(NodeID node_id) const { return occupation_mask_[node_id.get()]; }

    uint8_t getSize() const { return static_cast<uint8_t>(occupation_mask_.count()); }
};

}
}
}

#endif // Include guard
