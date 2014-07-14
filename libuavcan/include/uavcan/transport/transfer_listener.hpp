/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <uavcan/error.hpp>
#include <uavcan/stdint.hpp>
#include <uavcan/transport/transfer_receiver.hpp>
#include <uavcan/transport/perf_counter.hpp>
#include <uavcan/util/linked_list.hpp>
#include <uavcan/util/map.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/transport/crc.hpp>
#include <uavcan/data_type.hpp>

namespace uavcan
{
/**
 * Container for received transfer.
 */
class UAVCAN_EXPORT IncomingTransfer : public ITransferBuffer
{
    MonotonicTime ts_mono_;
    UtcTime ts_utc_;
    TransferType transfer_type_;
    TransferID transfer_id_;
    NodeID src_node_id_;
    uint8_t iface_index_;

    /// That's a no-op, asserts in debug builds
    virtual int write(unsigned offset, const uint8_t* data, unsigned len);

protected:
    IncomingTransfer(MonotonicTime ts_mono, UtcTime ts_utc, TransferType transfer_type,
                     TransferID transfer_id, NodeID source_node_id, uint8_t iface_index)
        : ts_mono_(ts_mono)
        , ts_utc_(ts_utc)
        , transfer_type_(transfer_type)
        , transfer_id_(transfer_id)
        , src_node_id_(source_node_id)
        , iface_index_(iface_index)
    { }

public:
    /**
     * Dispose the payload buffer. Further calls to read() will not be possible.
     */
    virtual void release() { }

    MonotonicTime getMonotonicTimestamp() const { return ts_mono_; }
    UtcTime getUtcTimestamp()             const { return ts_utc_; }
    TransferType getTransferType()        const { return transfer_type_; }
    TransferID getTransferID()            const { return transfer_id_; }
    NodeID getSrcNodeID()                 const { return src_node_id_; }
    uint8_t getIfaceIndex()               const { return iface_index_; }
};

/**
 * Internal.
 */
class UAVCAN_EXPORT SingleFrameIncomingTransfer : public IncomingTransfer
{
    const uint8_t* const payload_;
    const uint8_t payload_len_;
public:
    explicit SingleFrameIncomingTransfer(const RxFrame& frm);
    virtual int read(unsigned offset, uint8_t* data, unsigned len) const;
};

/**
 * Internal.
 */
class UAVCAN_EXPORT MultiFrameIncomingTransfer : public IncomingTransfer, Noncopyable
{
    TransferBufferAccessor& buf_acc_;
public:
    MultiFrameIncomingTransfer(MonotonicTime ts_mono, UtcTime ts_utc, const RxFrame& last_frame,
                               TransferBufferAccessor& tba);
    virtual int read(unsigned offset, uint8_t* data, unsigned len) const;
    virtual void release() { buf_acc_.remove(); }
};

/**
 * Internal, refer to the transport dispatcher class.
 */
class UAVCAN_EXPORT TransferListenerBase : public LinkedListNode<TransferListenerBase>, Noncopyable
{
    const DataTypeDescriptor& data_type_;
    const TransferCRC crc_base_;                      ///< Pre-initialized with data type hash, thus constant
    MapBase<TransferBufferManagerKey, TransferReceiver>& receivers_;
    ITransferBufferManager& bufmgr_;
    TransferPerfCounter& perf_;

    class TimedOutReceiverPredicate
    {
        const MonotonicTime ts_;
        ITransferBufferManager& parent_bufmgr_;

    public:
        TimedOutReceiverPredicate(MonotonicTime arg_ts, ITransferBufferManager& arg_bufmgr)
            : ts_(arg_ts)
            , parent_bufmgr_(arg_bufmgr)
        { }

        bool operator()(const TransferBufferManagerKey& key, const TransferReceiver& value) const;
    };

    bool checkPayloadCrc(const uint16_t compare_with, const ITransferBuffer& tbb) const;

protected:
    TransferListenerBase(TransferPerfCounter& perf, const DataTypeDescriptor& data_type,
                         MapBase<TransferBufferManagerKey, TransferReceiver>& receivers,
                         ITransferBufferManager& bufmgr)
        : data_type_(data_type)
        , crc_base_(data_type.getSignature().toTransferCRC())
        , receivers_(receivers)
        , bufmgr_(bufmgr)
        , perf_(perf)
    { }

    virtual ~TransferListenerBase() { }

    void handleReception(TransferReceiver& receiver, const RxFrame& frame, TransferBufferAccessor& tba);

    virtual void handleIncomingTransfer(IncomingTransfer& transfer) = 0;

public:
    const DataTypeDescriptor& getDataTypeDescriptor() const { return data_type_; }

    void cleanup(MonotonicTime ts);

    virtual void handleFrame(const RxFrame& frame);
};

/**
 * This class should be derived by transfer receivers (subscribers, servers).
 */
template <unsigned MaxBufSize, unsigned NumStaticBufs, unsigned NumStaticReceivers>
class UAVCAN_EXPORT TransferListener : public TransferListenerBase
{
    TransferBufferManager<MaxBufSize, NumStaticBufs> bufmgr_;
    Map<TransferBufferManagerKey, TransferReceiver, NumStaticReceivers> receivers_;

public:
    TransferListener(TransferPerfCounter& perf, const DataTypeDescriptor& data_type, IPoolAllocator& allocator)
        : TransferListenerBase(perf, data_type, receivers_, bufmgr_)
        , bufmgr_(allocator)
        , receivers_(allocator)
    {
#if UAVCAN_TINY
        StaticAssert<NumStaticBufs == 0>::check();
        StaticAssert<NumStaticReceivers == 0>::check();
#endif
        StaticAssert<(NumStaticReceivers >= NumStaticBufs)>::check();  // Otherwise it would be meaningless
    }

    virtual ~TransferListener()
    {
        // Map must be cleared before bufmgr is destructed
        receivers_.removeAll();
    }
};

/**
 * This class should be derived by callers.
 */
template <unsigned MaxBufSize>
class UAVCAN_EXPORT ServiceResponseTransferListener
#if UAVCAN_TINY
    : public TransferListener<MaxBufSize, 0, 0>
#else
    : public TransferListener<MaxBufSize, 1, 1>
#endif
{
public:
    struct ExpectedResponseParams
    {
        NodeID src_node_id;
        TransferID transfer_id;

        ExpectedResponseParams()
        {
            UAVCAN_ASSERT(!src_node_id.isValid());
        }

        ExpectedResponseParams(NodeID arg_src_node_id, TransferID arg_transfer_id)
            : src_node_id(arg_src_node_id)
            , transfer_id(arg_transfer_id)
        {
            UAVCAN_ASSERT(src_node_id.isUnicast());
        }

        bool match(const RxFrame& frame) const
        {
            UAVCAN_ASSERT(frame.getTransferType() == TransferTypeServiceResponse);
            return (frame.getSrcNodeID() == src_node_id) && (frame.getTransferID() == transfer_id);
        }
    };

private:
    typedef TransferListener<MaxBufSize, 1, 1> BaseType;

    ExpectedResponseParams response_params_;

    void handleFrame(const RxFrame& frame);

public:
    ServiceResponseTransferListener(TransferPerfCounter& perf, const DataTypeDescriptor& data_type,
                                    IPoolAllocator& allocator)
        : BaseType(perf, data_type, allocator)
    { }

    void setExpectedResponseParams(const ExpectedResponseParams& erp);

    const ExpectedResponseParams& getExpectedResponseParams() const { return response_params_; }

    void stopAcceptingAnything();
};

// ----------------------------------------------------------------------------

/*
 * ServiceResponseTransferListener<>
 */
template <unsigned MaxBufSize>
void ServiceResponseTransferListener<MaxBufSize>::handleFrame(const RxFrame& frame)
{
    if (response_params_.match(frame))
    {
        BaseType::handleFrame(frame);
    }
}

template <unsigned MaxBufSize>
void ServiceResponseTransferListener<MaxBufSize>::setExpectedResponseParams(const ExpectedResponseParams& erp)
{
    response_params_ = erp;
}

template <unsigned MaxBufSize>
void ServiceResponseTransferListener<MaxBufSize>::stopAcceptingAnything()
{
    response_params_ = ExpectedResponseParams();
}

}
