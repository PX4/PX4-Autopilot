/*
 * CAN bus IO logic.
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <algorithm>
#include <sstream>
#include <cassert>
#include <limits>
#include <uavcan/internal/transport/can_io.hpp>
#include <uavcan/internal/debug.hpp>

namespace uavcan
{
/*
 * CanRxFrame
 */
std::string CanRxFrame::toString(StringRepresentation mode) const
{
    std::ostringstream os;
    os << CanFrame::toString(mode)
        << " ts_m=" << ts_monotonic << " ts_utc=" << ts_utc << " iface=" << int(iface_index);
    return os.str();
}

/*
 * CanTxQueue::Entry
 */
void CanTxQueue::Entry::destroy(Entry*& obj, IAllocator* allocator)
{
    assert(allocator);
    if (obj != NULL)
    {
        obj->~Entry();
        allocator->deallocate(obj);
        obj = NULL;
    }
}

bool CanTxQueue::Entry::qosHigherThan(const CanFrame& rhs_frame, Qos rhs_qos) const
{
    if (qos != rhs_qos)
        return qos > rhs_qos;
    return frame.priorityHigherThan(rhs_frame);
}

bool CanTxQueue::Entry::qosLowerThan(const CanFrame& rhs_frame, Qos rhs_qos) const
{
    if (qos != rhs_qos)
        return qos < rhs_qos;
    return frame.priorityLowerThan(rhs_frame);
}

std::string CanTxQueue::Entry::toString() const
{
    std::string str_qos;
    switch (qos)
    {
    case VOLATILE:
        str_qos = "<volat> ";
        break;
    case PERSISTENT:
        str_qos = "<perst> ";
        break;
    default:
        assert(0);
        str_qos = "<?WTF?> ";
    }
    return str_qos + frame.toString();
}

/*
 * CanTxQueue
 */
CanTxQueue::~CanTxQueue()
{
    Entry* p = queue_.get();
    while (p)
    {
        Entry* const next = p->getNextListNode();
        remove(p);
        p = next;
    }
}

void CanTxQueue::registerRejectedFrame()
{
    if (rejected_frames_cnt_ < std::numeric_limits<uint32_t>::max())
        rejected_frames_cnt_++;
}

void CanTxQueue::push(const CanFrame& frame, uint64_t monotonic_tx_deadline, Qos qos)
{
    const uint64_t timestamp = sysclock_->getMonotonicMicroseconds();

    if (timestamp >= monotonic_tx_deadline)
    {
        UAVCAN_TRACE("CanTxQueue", "Push rejected: already expired");
        registerRejectedFrame();
        return;
    }

    void* praw = allocator_->allocate(sizeof(Entry));
    if (praw == NULL)
    {
        UAVCAN_TRACE("CanTxQueue", "Push OOM #1, cleanup");
        // No memory left in the pool, so we try to remove expired frames
        Entry* p = queue_.get();
        while (p)
        {
            Entry* const next = p->getNextListNode();
            if (p->isExpired(timestamp))
            {
                UAVCAN_TRACE("CanTxQueue", "Push: Expired %s", p->toString().c_str());
                registerRejectedFrame();
                remove(p);
            }
            p = next;
        }
        praw = allocator_->allocate(sizeof(Entry));         // Try again
    }

    if (praw == NULL)
    {
        UAVCAN_TRACE("CanTxQueue", "Push OOM #2, QoS arbitration");
        registerRejectedFrame();

        // Find a frame with lowest QoS
        Entry* p = queue_.get();
        Entry* lowestqos = p;
        while (p)
        {
            if (lowestqos->qosHigherThan(*p))
                lowestqos = p;
            p = p->getNextListNode();
        }
        // Note that frame with *equal* QoS will be replaced too.
        if (lowestqos->qosHigherThan(frame, qos))           // Frame that we want to transmit has lowest QoS
        {
            UAVCAN_TRACE("CanTxQueue", "Push rejected: low QoS");
            return;                                         // What a loser.
        }
        UAVCAN_TRACE("CanTxQueue", "Push: Replacing %s", lowestqos->toString().c_str());
        remove(lowestqos);
        praw = allocator_->allocate(sizeof(Entry));        // Try again
    }

    if (praw == NULL)
        return;                                            // Seems that there is no memory at all.

    Entry* entry = new (praw) Entry(frame, monotonic_tx_deadline, qos);
    assert(entry);
    queue_.insertBefore(entry, PriorityInsertionComparator(frame));
}

CanTxQueue::Entry* CanTxQueue::peek()
{
    const uint64_t timestamp = sysclock_->getMonotonicMicroseconds();
    Entry* p = queue_.get();
    while (p)
    {
        if (p->isExpired(timestamp))
        {
            UAVCAN_TRACE("CanTxQueue", "Peek: Expired %s", p->toString().c_str());
            Entry* const next = p->getNextListNode();
            registerRejectedFrame();
            remove(p);
            p = next;
        }
        else
            return p;
    }
    return NULL;
}

void CanTxQueue::remove(Entry*& entry)
{
    if (entry == NULL)
    {
        assert(0);
        return;
    }
    queue_.remove(entry);
    Entry::destroy(entry, allocator_);
}

bool CanTxQueue::topPriorityHigherOrEqual(const CanFrame& rhs_frame) const
{
    const Entry* entry = queue_.get();
    if (entry == NULL)
        return false;
    return !rhs_frame.priorityHigherThan(entry->frame);
}

/*
 * CanIOManager
 */
int CanIOManager::sendToIface(int iface_index, const CanFrame& frame, uint64_t monotonic_tx_deadline)
{
    assert(iface_index >= 0 && iface_index < MAX_IFACES);
    ICanIface* const iface = driver_->getIface(iface_index);
    if (iface == NULL)
    {
        assert(0);   // Nonexistent interface
        return -1;
    }
    const uint64_t timestamp = sysclock_->getMonotonicMicroseconds();
    const uint64_t timeout = (timestamp >= monotonic_tx_deadline) ? 0 : (monotonic_tx_deadline - timestamp);
    const int res = iface->send(frame, timeout);
    if (res != 1)
    {
        UAVCAN_TRACE("CanIOManager", "Send failed: code %i, iface %i, frame %s",
                     res, iface_index, frame.toString().c_str());
    }
    return res;
}

int CanIOManager::sendFromTxQueue(int iface_index)
{
    assert(iface_index >= 0 && iface_index < MAX_IFACES);
    CanTxQueue::Entry* entry = tx_queues_[iface_index].peek();
    if (entry == NULL)
        return 0;
    const int res = sendToIface(iface_index, entry->frame, entry->monotonic_deadline);
    if (res > 0)
        tx_queues_[iface_index].remove(entry);
    return res;
}

int CanIOManager::makePendingTxMask() const
{
    int write_mask = 0;
    for (int i = 0; i < getNumIfaces(); i++)
    {
        if (!tx_queues_[i].isEmpty())
            write_mask |= 1 << i;
    }
    return write_mask;
}

uint64_t CanIOManager::getTimeUntilMonotonicDeadline(uint64_t monotonic_deadline) const
{
    const uint64_t timestamp = sysclock_->getMonotonicMicroseconds();
    return (timestamp >= monotonic_deadline) ? 0 : (monotonic_deadline - timestamp);
}

int CanIOManager::getNumIfaces() const
{
    const int num = driver_->getNumIfaces();
    assert(num > 0 && num <= MAX_IFACES);
    return std::min(std::max(num, 0), (int)MAX_IFACES);
}

uint64_t CanIOManager::getNumErrors(int iface_index) const
{
    ICanIface* const iface = driver_->getIface(iface_index);
    if (iface == NULL || iface_index >= MAX_IFACES || iface_index < 0)
    {
        assert(0);
        return std::numeric_limits<uint64_t>::max();
    }
    return iface->getNumErrors() + tx_queues_[iface_index].getNumRejectedFrames();
}

int CanIOManager::send(const CanFrame& frame, uint64_t monotonic_tx_deadline, uint64_t monotonic_blocking_deadline,
                        int iface_mask, CanTxQueue::Qos qos)
{
    const int num_ifaces = getNumIfaces();
    const int all_ifaces_mask = (1 << num_ifaces) - 1;

    assert((iface_mask & ~all_ifaces_mask) == 0);
    iface_mask &= all_ifaces_mask;

    if (monotonic_blocking_deadline > monotonic_tx_deadline)
        monotonic_blocking_deadline = monotonic_tx_deadline;

    int retval = 0;

    while (true)
    {
        if (iface_mask == 0)
            break;
        int write_mask = iface_mask | makePendingTxMask();

        const uint64_t timeout = getTimeUntilMonotonicDeadline(monotonic_blocking_deadline);
        {
            int read_mask = 0;
            const int select_res = driver_->select(write_mask, read_mask, timeout);
            if (select_res < 0)
                return select_res;
        }

        // Transmission
        for (int i = 0; i < num_ifaces; i++)
        {
            if (write_mask & (1 << i))
            {
                int res = 0;
                if (iface_mask & (1 << i))
                {
                    if (tx_queues_[i].topPriorityHigherOrEqual(frame))
                    {
                        res = sendFromTxQueue(i);                 // May return 0 if nothing to transmit (e.g. expired)
                    }
                    if (res <= 0)
                    {
                        res = sendToIface(i, frame, monotonic_tx_deadline);
                        if (res > 0)
                            iface_mask &= ~(1 << i);              // Mark transmitted
                    }
                }
                else
                {
                    res = sendFromTxQueue(i);
                }
                if (res > 0)
                    retval++;
            }
        }

        // Timeout. Enqueue the frame if wasn't transmitted and leave.
        if (write_mask == 0 || timeout == 0)
        {
            if ((timeout > 0) && (sysclock_->getMonotonicMicroseconds() < monotonic_blocking_deadline))
            {
                UAVCAN_TRACE("CanIOManager", "Send: Premature timeout in select(), will try again");
                continue;
            }
            for (int i = 0; i < num_ifaces; i++)
            {
                if (iface_mask & (1 << i))
                    tx_queues_[i].push(frame, monotonic_tx_deadline, qos);
            }
            break;
        }
    }
    return retval;
}

int CanIOManager::receive(CanRxFrame& frame, uint64_t monotonic_deadline)
{
    const int num_ifaces = getNumIfaces();

    while (true)
    {
        int write_mask = makePendingTxMask();
        int read_mask = (1 << num_ifaces) - 1;
        const uint64_t timeout = getTimeUntilMonotonicDeadline(monotonic_deadline);
        {
            const int select_res = driver_->select(write_mask, read_mask, timeout);
            if (select_res < 0)
                return select_res;
        }

        // Write - if buffers are not empty, one frame will be sent for each iface per one receive() call
        for (int i = 0; i < num_ifaces; i++)
        {
            if (write_mask & (1 << i))
                sendFromTxQueue(i);
        }

        // Read
        for (int i = 0; i < num_ifaces; i++)
        {
            if (read_mask & (1 << i))
            {
                ICanIface* const iface = driver_->getIface(i);
                if (iface == NULL)
                {
                    assert(0);   // Nonexistent interface
                    continue;
                }
                const int res = iface->receive(frame, frame.ts_monotonic, frame.ts_utc);
                if (res == 0)
                {
                    assert(0);   // select() reported that iface has pending RX frames, but receive() returned none
                    continue;
                }
                frame.iface_index = i;
                return res;
            }
        }

        if (timeout == 0)  // Timeout checked in the last order - this way we can operate with expired deadline
            break;
    }
    return 0;
}

}
