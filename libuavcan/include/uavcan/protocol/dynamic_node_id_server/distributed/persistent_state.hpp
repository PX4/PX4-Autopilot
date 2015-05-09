/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISTRIBUTED_PERSISTENT_STATE_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISTRIBUTED_PERSISTENT_STATE_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/protocol/dynamic_node_id_server/distributed/types.hpp>
#include <uavcan/protocol/dynamic_node_id_server/distributed/event.hpp>
#include <uavcan/protocol/dynamic_node_id_server/distributed/log.hpp>
#include <uavcan/protocol/dynamic_node_id_server/storage_marshaller.hpp>

namespace uavcan
{
namespace dynamic_node_id_server
{
namespace distributed
{
/**
 * This class is a convenient container for persistent state variables defined by Raft.
 * Writes are slow, reads are instantaneous.
 */
class PersistentState
{
    IStorageBackend& storage_;
    IEventTracer& tracer_;

    Term current_term_;
    NodeID voted_for_;
    Log log_;

    static IStorageBackend::String getCurrentTermKey() { return "current_term"; }
    static IStorageBackend::String getVotedForKey() { return "voted_for"; }

public:
    PersistentState(IStorageBackend& storage, IEventTracer& tracer)
        : storage_(storage)
        , tracer_(tracer)
        , current_term_(0)
        , log_(storage, tracer)
    { }

    int init()
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

        StorageMarshaller io(storage_);

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

    Term getCurrentTerm() const { return current_term_; }

    NodeID getVotedFor() const { return voted_for_; }
    bool isVotedForSet() const { return voted_for_.isUnicast(); }

    Log& getLog()             { return log_; }
    const Log& getLog() const { return log_; }

    /**
     * Invokes storage IO.
     */
    int setCurrentTerm(Term term)
    {
        if (term < current_term_)
        {
            UAVCAN_ASSERT(0);
            return -ErrInvalidParam;
        }

        tracer_.onEvent(TraceCurrentTermUpdate, term);

        StorageMarshaller io(storage_);

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

    /**
     * Invokes storage IO.
     */
    int setVotedFor(NodeID node_id)
    {
        if (!node_id.isValid())
        {
            UAVCAN_ASSERT(0);
            return -ErrInvalidParam;
        }

        tracer_.onEvent(TraceVotedForUpdate, node_id.get());

        StorageMarshaller io(storage_);

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

    int resetVotedFor() { return setVotedFor(NodeID(0)); }
};

}
}
}

#endif // Include guard
