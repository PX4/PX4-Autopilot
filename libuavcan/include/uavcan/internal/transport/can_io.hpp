/*
 * CAN bus IO logic.
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <stdint.h>
#include <uavcan/internal/linked_list.hpp>
#include <uavcan/internal/dynamic_memory.hpp>
#include <uavcan/can_driver.hpp>
#include <uavcan/system_clock.hpp>

namespace uavcan
{

struct CanRxFrame
{
    CanFrame frame;
    uint64_t timestamp;
    uint8_t iface_index;
};


class CanTxQueue
{
public:
    enum Qos { VOLATILE, PERSISTENT };

#pragma pack(push, 1)
    struct Entry : public LinkedListNode<Entry>
    {
        uint64_t monotonic_deadline;
        CanFrame frame;
        uint8_t qos;

        Entry(const CanFrame& frame, uint64_t monotonic_deadline, Qos qos)
        : monotonic_deadline(monotonic_deadline)
        , frame(frame)
        , qos(uint8_t(qos))
        {
            assert(qos == VOLATILE || qos == PERSISTENT);
        }

        bool isExpired(uint64_t monotonic_timestamp) const { return monotonic_timestamp > monotonic_deadline; }

        bool qosHigherThan(const CanFrame& rhs_frame, Qos rhs_qos) const;
        bool qosLowerThan(const CanFrame& rhs_frame, Qos rhs_qos) const;
        bool qosHigherThan(const Entry& rhs) const { return qosHigherThan(rhs.frame, Qos(rhs.qos)); }
        bool qosLowerThan(const Entry& rhs)  const { return qosLowerThan(rhs.frame, Qos(rhs.qos)); }

        std::string toString() const;
    };
#pragma pack(pop)

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
    {
        assert(allocator);
        assert(sysclock);
    }

    ~CanTxQueue();

    void push(const CanFrame& frame, uint64_t monotonic_tx_deadline, Qos qos);

    Entry* peek();               // Modifier
    void remove(Entry* entry);

    bool topPriorityHigherOrEqual(const CanFrame& rhs_frame) const;

    uint32_t getNumRejectedFrames() const { return rejected_frames_cnt_; }

    bool isEmpty() const { return queue_.isEmpty(); }
};


class CanIOManager
{
public:
    enum { MAX_IFACES = 3 };

private:
    ICanDriver* const driver_;
    ISystemClock* const sysclock_;

    CanTxQueue tx_queues_[MAX_IFACES];

    // Noncopyable
    CanIOManager(CanIOManager&);
    CanIOManager& operator=(CanIOManager&);

    int sendToIface(int iface_index, const CanFrame& frame, uint64_t monotonic_tx_deadline);
    int sendFromTxQueue(int iface_index);
    int makePendingTxMask() const;
    uint64_t getTimeUntilMonotonicDeadline(uint64_t monotonic_deadline) const;

public:
    CanIOManager(ICanDriver* driver, IAllocator* allocator, ISystemClock* sysclock)
    : driver_(driver)
    , sysclock_(sysclock)
    {
        assert(driver);
        assert(allocator);
        assert(sysclock);
        assert(driver->getNumIfaces() <= MAX_IFACES);
        // We can't initialize member array with non-default constructors in C++03
        for (int i = 0; i < MAX_IFACES; i++)
        {
            tx_queues_[i].~CanTxQueue();
            new (tx_queues_ + i) CanTxQueue(allocator, sysclock);
        }
    }

    int getNumIfaces() const;

    uint64_t getNumErrors(int iface_index) const;

    /**
     * Returns:
     *  0 - rejected/timedout/enqueued
     *  1+ - sent/received
     *  negative - failure
     */
    int send(const CanFrame& frame, uint64_t monotonic_tx_deadline, uint64_t monotonic_blocking_deadline,
             int iface_mask, CanTxQueue::Qos qos);
    int receive(CanRxFrame& frame, uint64_t monotonic_blocking_deadline);
};

}
