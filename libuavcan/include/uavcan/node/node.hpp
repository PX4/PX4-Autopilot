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
#include <uavcan/protocol/node_status_provider.hpp>

#if !UAVCAN_TINY
# include <uavcan/protocol/logger.hpp>
# include <uavcan/protocol/restart_request_server.hpp>
# include <uavcan/protocol/transport_stats_provider.hpp>
# include <uavcan/protocol/network_compat_checker.hpp>
#endif

#if !defined(UAVCAN_CPP_VERSION) || !defined(UAVCAN_CPP11)
# error UAVCAN_CPP_VERSION
#endif

namespace uavcan
{

template <std::size_t MemPoolSize_,
          unsigned OutgoingTransferRegistryStaticEntries = 10,
          unsigned OutgoingTransferMaxPayloadLen = MaxTransferPayloadLen>
class UAVCAN_EXPORT Node : public INode
{
    enum
    {
        MemPoolSize = (MemPoolSize_ < std::size_t(MemPoolBlockSize)) ? std::size_t(MemPoolBlockSize) : MemPoolSize_
    };

    typedef PoolAllocator<MemPoolSize, MemPoolBlockSize> Allocator;

    Allocator pool_allocator_;
    MarshalBufferProvider<OutgoingTransferMaxPayloadLen> marsh_buf_;
    OutgoingTransferRegistry<OutgoingTransferRegistryStaticEntries> outgoing_trans_reg_;
    Scheduler scheduler_;

    DataTypeInfoProvider proto_dtp_;
    NodeStatusProvider proto_nsp_;
#if !UAVCAN_TINY
    Logger proto_logger_;
    RestartRequestServer proto_rrs_;
    TransportStatsProvider proto_tsp_;
#endif

    bool started_;

protected:
    virtual void registerInternalFailure(const char* msg)
    {
        UAVCAN_TRACE("Node", "Internal failure: %s", msg);
#if UAVCAN_TINY
        (void)msg;
#else
        (void)getLogger().log(protocol::debug::LogLevel::ERROR, "UAVCAN", msg);
#endif
    }

    virtual IMarshalBufferProvider& getMarshalBufferProvider() { return marsh_buf_; }

public:
    Node(ICanDriver& can_driver, ISystemClock& system_clock)
        : outgoing_trans_reg_(pool_allocator_)
        , scheduler_(can_driver, pool_allocator_, system_clock, outgoing_trans_reg_)
        , proto_dtp_(*this)
        , proto_nsp_(*this)
#if !UAVCAN_TINY
        , proto_logger_(*this)
        , proto_rrs_(*this)
        , proto_tsp_(*this)
#endif
        , started_(false)
    { }

    virtual Allocator& getAllocator() { return pool_allocator_; }

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

    int start();

#if !UAVCAN_TINY
    int checkNetworkCompatibility(NetworkCompatibilityCheckResult& result);
#endif

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

#if !UAVCAN_TINY
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
    inline void logDebug(const char* source, const char* format, Args... args)
    {
        (void)proto_logger_.logDebug(source, format, args...);
    }

    template <typename... Args>
    inline void logInfo(const char* source, const char* format, Args... args)
    {
        (void)proto_logger_.logInfo(source, format, args...);
    }

    template <typename... Args>
    inline void logWarning(const char* source, const char* format, Args... args)
    {
        (void)proto_logger_.logWarning(source, format, args...);
    }

    template <typename... Args>
    inline void logError(const char* source, const char* format, Args... args)
    {
        (void)proto_logger_.logError(source, format, args...);
    }

#else

    void logDebug(const char* source, const char* text)   { (void)proto_logger_.logDebug(source, text); }
    void logInfo(const char* source, const char* text)    { (void)proto_logger_.logInfo(source, text); }
    void logWarning(const char* source, const char* text) { (void)proto_logger_.logWarning(source, text); }
    void logError(const char* source, const char* text)   { (void)proto_logger_.logError(source, text); }

#endif

    Logger& getLogger() { return proto_logger_; }

#endif  // UAVCAN_TINY
};

// ----------------------------------------------------------------------------

template <std::size_t MemPoolSize_, unsigned OutgoingTransferRegistryStaticEntries,
          unsigned OutgoingTransferMaxPayloadLen>
int Node<MemPoolSize_, OutgoingTransferRegistryStaticEntries, OutgoingTransferMaxPayloadLen>::start()
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
    res = proto_nsp_.startAndPublish();
    if (res < 0)
    {
        goto fail;
    }
#if !UAVCAN_TINY
    res = proto_logger_.init();
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
#endif
    started_ = res >= 0;
    return res;
fail:
    assert(res < 0);
    return res;
}

#if !UAVCAN_TINY

template <std::size_t MemPoolSize_, unsigned OutgoingTransferRegistryStaticEntries,
          unsigned OutgoingTransferMaxPayloadLen>
int Node<MemPoolSize_, OutgoingTransferRegistryStaticEntries, OutgoingTransferMaxPayloadLen>::
checkNetworkCompatibility(NetworkCompatibilityCheckResult& result)
{
    if (!started_)
    {
        return -ErrNotInited;
    }

    int res = NetworkCompatibilityChecker::publishGlobalDiscoveryRequest(*this);
    if (res < 0)
    {
        return res;
    }

    NetworkCompatibilityChecker checker(*this);
    StaticAssert<(sizeof(checker) < 2048)>::check();
    res = checker.execute();
    result = checker.getResult();
    return res;
}

#endif

}
