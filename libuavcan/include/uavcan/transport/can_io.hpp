/*
 * CAN bus IO logic.
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <uavcan/error.hpp>
#include <uavcan/stdint.hpp>
#include <uavcan/linked_list.hpp>
#include <uavcan/dynamic_memory.hpp>
#include <uavcan/impl_constants.hpp>
#include <uavcan/util/compile_time.hpp>
#include <uavcan/driver/can.hpp>
#include <uavcan/driver/system_clock.hpp>
#include <uavcan/time.hpp>

namespace uavcan
{

enum { MaxCanIfaces = 3 };

struct UAVCAN_EXPORT CanRxFrame : public CanFrame
{
    MonotonicTime ts_mono;
    UtcTime ts_utc;
    uint8_t iface_index;

    CanRxFrame()
        : iface_index(0)
    { }

    std::string toString(StringRepresentation mode = StrTight) const;
};


class UAVCAN_EXPORT CanTxQueue : Noncopyable
{
public:
    enum Qos { Volatile, Persistent };

    struct Entry : public LinkedListNode<Entry>  // Not required to be packed - fits the block in any case
    {
        MonotonicTime deadline;
        CanFrame frame;
        uint8_t qos;
        CanIOFlags flags;

        Entry(const CanFrame& frame, MonotonicTime deadline, Qos qos, CanIOFlags flags)
            : deadline(deadline)
            , frame(frame)
            , qos(uint8_t(qos))
            , flags(flags)
        {
            assert(qos == Volatile || qos == Persistent);
            IsDynamicallyAllocatable<Entry>::check();
        }

        static void destroy(Entry*& obj, IAllocator& allocator);

        bool isExpired(MonotonicTime timestamp) const { return timestamp > deadline; }

        bool qosHigherThan(const CanFrame& rhs_frame, Qos rhs_qos) const;
        bool qosLowerThan(const CanFrame& rhs_frame, Qos rhs_qos) const;
        bool qosHigherThan(const Entry& rhs) const { return qosHigherThan(rhs.frame, Qos(rhs.qos)); }
        bool qosLowerThan(const Entry& rhs)  const { return qosLowerThan(rhs.frame, Qos(rhs.qos)); }

        std::string toString() const;
    };

private:
    class PriorityInsertionComparator
    {
        const CanFrame& frm_;
    public:
        PriorityInsertionComparator(const CanFrame& frm) : frm_(frm) { }
        bool operator()(const Entry* entry)
        {
            assert(entry);
            return frm_.priorityHigherThan(entry->frame);
        }
    };

    LinkedListRoot<Entry> queue_;
    IAllocator* const allocator_;
    ISystemClock* const sysclock_;
    uint32_t rejected_frames_cnt_;

    void registerRejectedFrame();

public:
    CanTxQueue()
        : allocator_(NULL)
        , sysclock_(NULL)
        , rejected_frames_cnt_(0)
    { }

    CanTxQueue(IAllocator* allocator, ISystemClock* sysclock)
        : allocator_(allocator)
        , sysclock_(sysclock)
        , rejected_frames_cnt_(0)
    { }

    ~CanTxQueue();

    void push(const CanFrame& frame, MonotonicTime tx_deadline, Qos qos, CanIOFlags flags);

    Entry* peek();               // Modifier
    void remove(Entry*& entry);

    bool topPriorityHigherOrEqual(const CanFrame& rhs_frame) const;

    uint32_t getRejectedFrameCount() const { return rejected_frames_cnt_; }

    bool isEmpty() const { return queue_.isEmpty(); }
};


struct UAVCAN_EXPORT CanIfacePerfCounters
{
    uint64_t frames_tx;
    uint64_t frames_rx;
    uint64_t errors;

    CanIfacePerfCounters()
        : frames_tx(0)
        , frames_rx(0)
        , errors(0)
    { }
};


class UAVCAN_EXPORT CanIOManager : Noncopyable
{
    struct IfaceFrameCounters
    {
        uint64_t frames_tx;
        uint64_t frames_rx;

        IfaceFrameCounters()
            : frames_tx(0)
            , frames_rx(0)
        { }
    };

    ICanDriver& driver_;
    ISystemClock& sysclock_;

    CanTxQueue tx_queues_[MaxCanIfaces];
    IfaceFrameCounters counters_[MaxCanIfaces];

    int sendToIface(uint8_t iface_index, const CanFrame& frame, MonotonicTime tx_deadline, CanIOFlags flags);
    int sendFromTxQueue(uint8_t iface_index);
    uint8_t makePendingTxMask() const;
    int callSelect(CanSelectMasks& inout_masks, MonotonicTime blocking_deadline);

public:
    CanIOManager(ICanDriver& driver, IAllocator& allocator, ISystemClock& sysclock)
        : driver_(driver)
        , sysclock_(sysclock)
    {
        assert(driver.getNumIfaces() <= MaxCanIfaces);
        // We can't initialize member array with non-default constructors in C++03
        for (int i = 0; i < MaxCanIfaces; i++)
        {
            tx_queues_[i].~CanTxQueue();
            new (tx_queues_ + i) CanTxQueue(&allocator, &sysclock);
        }
    }

    uint8_t getNumIfaces() const;

    CanIfacePerfCounters getIfacePerfCounters(uint8_t iface_index) const;

    /**
     * Returns:
     *  0 - rejected/timedout/enqueued
     *  1+ - sent/received
     *  negative - failure
     */
    int send(const CanFrame& frame, MonotonicTime tx_deadline, MonotonicTime blocking_deadline,
             uint8_t iface_mask, CanTxQueue::Qos qos, CanIOFlags flags);
    int receive(CanRxFrame& out_frame, MonotonicTime blocking_deadline, CanIOFlags& out_flags);
};

}
