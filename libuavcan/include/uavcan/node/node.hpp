/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <uavcan/error.hpp>
#include <uavcan/impl_constants.hpp>
#include <uavcan/dynamic_memory.hpp>
#include <uavcan/node/abstract_node.hpp>
#include <uavcan/node/marshal_buffer.hpp>

// High-level functionality available by default
#include <uavcan/protocol/data_type_info_provider.hpp>
#include <uavcan/protocol/logger.hpp>
#include <uavcan/protocol/node_status_provider.hpp>
#include <uavcan/protocol/restart_request_server.hpp>
#include <uavcan/protocol/transport_stats_provider.hpp>
#include <uavcan/protocol/node_initializer.hpp>

#if !defined(UAVCAN_CPP_VERSION) || !defined(UAVCAN_CPP11)
# error UAVCAN_CPP_VERSION
#endif

namespace uavcan
{

template <std::size_t MemPoolSize_,
          unsigned OutgoingTransferRegistryStaticEntries = 10,
          unsigned OutgoingTransferMaxPayloadLen = MaxTransferPayloadLen>
class Node : public INode
{
    enum
    {
        MemPoolSize = (MemPoolSize_ < std::size_t(MemPoolBlockSize)) ? std::size_t(MemPoolBlockSize) : MemPoolSize_
    };

    PoolAllocator<MemPoolSize, MemPoolBlockSize> pool_allocator_;
    MarshalBufferProvider<OutgoingTransferMaxPayloadLen> marsh_buf_;
    OutgoingTransferRegistry<OutgoingTransferRegistryStaticEntries> outgoing_trans_reg_;
    Scheduler scheduler_;

    DataTypeInfoProvider proto_dtp_;
    Logger proto_logger_;
    NodeStatusProvider proto_nsp_;
    RestartRequestServer proto_rrs_;
    TransportStatsProvider proto_tsp_;

    bool started_;

    int initNetwork(NodeInitializationResult& node_init_result)
    {
        int res = NodeInitializer::publishGlobalDiscoveryRequest(*this);
        if (res < 0)
        {
            return res;
        }
        NodeInitializer initializer(*this);
        StaticAssert<(sizeof(initializer) < 1024)>::check();
        res = initializer.execute();
        node_init_result = initializer.getResult();
        return res;
    }

protected:
    virtual void registerInternalFailure(const char* msg)
    {
        UAVCAN_TRACE("Node", "Internal failure: %s", msg);
        (void)logError("UAVCAN", msg);
    }

    virtual IAllocator& getAllocator() { return pool_allocator_; }

    virtual IMarshalBufferProvider& getMarshalBufferProvider() { return marsh_buf_; }

public:
    Node(ICanDriver& can_driver, ISystemClock& system_clock)
        : outgoing_trans_reg_(pool_allocator_)
        , scheduler_(can_driver, pool_allocator_, system_clock, outgoing_trans_reg_)
        , proto_dtp_(*this)
        , proto_logger_(*this)
        , proto_nsp_(*this)
        , proto_rrs_(*this)
        , proto_tsp_(*this)
        , started_(false)
    { }

    virtual Scheduler& getScheduler() { return scheduler_; }
    virtual const Scheduler& getScheduler() const { return scheduler_; }

    int spin(MonotonicTime deadline)
    {
        if (started_)
        {
            return INode::spin(deadline);
        }
        return -ErrNotInited;
    }

    int spin(MonotonicDuration duration)
    {
        if (started_)
        {
            return INode::spin(duration);
        }
        return -ErrNotInited;
    }

    bool isStarted() const { return started_; }

    int start(NodeInitializationResult& node_init_result)
    {
        if (started_)
        {
            return 0;
        }
        GlobalDataTypeRegistry::instance().freeze();

        int res = 0;
        res = proto_dtp_.start();
        if (res < 0)
        {
            goto fail;
        }
        res = proto_logger_.init();
        if (res < 0)
        {
            goto fail;
        }
        res = proto_nsp_.startAndPublish();
        if (res < 0)
        {
            goto fail;
        }
        res = proto_rrs_.start();
        if (res < 0)
        {
            goto fail;
        }
        res = proto_tsp_.start();
        if (res < 0)
        {
            goto fail;
        }
        res = initNetwork(node_init_result);
        started_ = (res >= 0) && node_init_result.isOk();
        return res;
    fail:
        assert(res < 0);
        return res;
    }

    /*
     * Initialization methods
     */
    void setName(const char* name) { proto_nsp_.setName(name); }

    void setStatusOk()           { proto_nsp_.setStatusOk(); }
    void setStatusInitializing() { proto_nsp_.setStatusInitializing(); }
    void setStatusWarning()      { proto_nsp_.setStatusWarning(); }
    void setStatusCritical()     { proto_nsp_.setStatusCritical(); }
    void setStatusOffline()
    {
        proto_nsp_.setStatusOffline();
        (void)proto_nsp_.forcePublish();
    }

    void setSoftwareVersion(const protocol::SoftwareVersion& version) { proto_nsp_.setSoftwareVersion(version); }
    void setHardwareVersion(const protocol::HardwareVersion& version) { proto_nsp_.setHardwareVersion(version); }

    NodeStatusProvider& getNodeStatusProvider() { return proto_nsp_; }

    /*
     * Restart handler
     */
    void setRestartRequestHandler(IRestartRequestHandler* handler) { proto_rrs_.setHandler(handler); }

    RestartRequestServer& getRestartRequestServer() { return proto_rrs_; }

    /*
     * Logging
     */
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11

    template <typename... Args>
    inline int logDebug(const char* source, const char* format, Args... args)
    {
        return proto_logger_.logDebug(source, format, args...);
    }

    template <typename... Args>
    inline int logInfo(const char* source, const char* format, Args... args)
    {
        return proto_logger_.logInfo(source, format, args...);
    }

    template <typename... Args>
    inline int logWarning(const char* source, const char* format, Args... args)
    {
        return proto_logger_.logWarning(source, format, args...);
    }

    template <typename... Args>
    inline int logError(const char* source, const char* format, Args... args)
    {
        return proto_logger_.logError(source, format, args...);
    }

#else

    int logDebug(const char* source, const char* text)   { return proto_logger_.logDebug(source, text); }
    int logInfo(const char* source, const char* text)    { return proto_logger_.logInfo(source, text); }
    int logWarning(const char* source, const char* text) { return proto_logger_.logWarning(source, text); }
    int logError(const char* source, const char* text)   { return proto_logger_.logError(source, text); }

#endif

    Logger& getLogger() { return proto_logger_; }
};

}
