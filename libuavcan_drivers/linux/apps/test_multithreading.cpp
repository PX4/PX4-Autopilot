/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <iostream>
#include <thread>
#include <condition_variable>
#include <uavcan_linux/uavcan_linux.hpp>
#include <uavcan/node/sub_node.hpp>
#include <uavcan/protocol/node_status_monitor.hpp>
#include "debug.hpp"

/**
 * Objects of this class are owned by the sub-node thread.
 */
class VirtualCanIface : public uavcan::ICanIface, uavcan::Noncopyable
{
    struct RxItem
    {
        const uavcan::CanRxFrame frame;
        const uavcan::CanIOFlags flags;

        RxItem(const uavcan::CanRxFrame& arg_frame, uavcan::CanIOFlags arg_flags) :
            frame(arg_frame),
            flags(arg_flags)
        { }
    };

    std::mutex& mutex_;
    uavcan::CanTxQueue tx_queue_;
    std::queue<RxItem> rx_queue_;

    int16_t send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline, uavcan::CanIOFlags flags) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        tx_queue_.push(frame, tx_deadline, uavcan::CanTxQueue::Volatile, flags);
        return 1;
    }

    int16_t receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                    uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags) override
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (rx_queue_.empty())
        {
            return 0;
        }

        const auto item = rx_queue_.front();
        rx_queue_.pop();

        out_frame = item.frame;
        out_ts_monotonic = item.frame.ts_mono;
        out_ts_utc = item.frame.ts_utc;
        out_flags = item.flags;

        return 1;
    }

    int16_t configureFilters(const uavcan::CanFilterConfig*, std::uint16_t) override { return -uavcan::ErrDriver; }
    uint16_t getNumFilters() const override { return 0; }
    uint64_t getErrorCount() const override { return 0; }

    static unsigned computeTxQueuePoolQuota(uavcan::INode& node)
    {
        return node.getAllocator().getNumBlocks() / 4;
    }

public:
    VirtualCanIface(uavcan::INode& node, std::mutex& arg_mutex) :
        mutex_(arg_mutex),
        tx_queue_(node.getAllocator(), node.getSystemClock(), computeTxQueuePoolQuota(node))
    { }

    /**
     * Call this from the main thread only.
     * No additional locking is required.
     */
    void addRxFrame(const uavcan::CanRxFrame& frame, uavcan::CanIOFlags flags)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        rx_queue_.emplace(frame, flags);
    }

    /**
     * Call this from the main thread only.
     * No additional locking is required.
     */
    void flushTxQueueTo(uavcan::INode& main_node, std::uint8_t iface_index)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        const std::uint8_t iface_mask = static_cast<std::uint8_t>(1U << iface_index);

        while (auto e = tx_queue_.peek())
        {
            const int res = main_node.injectTxFrame(e->frame, e->deadline, iface_mask,
                                                    uavcan::CanTxQueue::Qos(e->qos), e->flags);
            if (res <= 0)
            {
                break;
            }
            tx_queue_.remove(e);
        }
    }

    /**
     * Call this from the sub-node thread only.
     * No additional locking is required.
     */
    bool hasDataInRxQueue() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return !rx_queue_.empty();
    }
};

/**
 * Objects of this class are owned by the sub-node thread.
 */
class VirtualCanDriver : public uavcan::ICanDriver, public uavcan::IRxFrameListener
{
    class Event
    {
        std::mutex m_;
        std::condition_variable cv_;

    public:
        /**
         * Note that this method may return spuriously.
         */
        void waitFor(uavcan::MonotonicDuration duration)
        {
            std::unique_lock<std::mutex> lk(m_);
            (void)cv_.wait_for(lk, std::chrono::microseconds(duration.toUSec()));
        }

        void signal() { cv_.notify_all(); }
    };

    Event event_;
    std::mutex mutex_;
    uavcan::INode& sub_node_;
    std::vector<std::shared_ptr<VirtualCanIface>> ifaces_;

    uavcan::ICanIface* getIface(uint8_t iface_index) override
    {
        return ifaces_.at(iface_index).get();
    }

    uint8_t getNumIfaces() const override { return ifaces_.size(); }

    /**
     * This and other methods of ICanDriver will be invoked by the sub-node thread.
     */
    int16_t select(uavcan::CanSelectMasks& inout_masks, uavcan::MonotonicTime blocking_deadline) override
    {
        bool need_block = (inout_masks.write == 0);    // Write queue is infinite
        for (unsigned i = 0; need_block && (i < ifaces_.size()); i++)
        {
            const bool need_read = inout_masks.read & (1U << i);
            if (need_read && ifaces_[i]->hasDataInRxQueue())
            {
                need_block = false;
            }
        }

        if (need_block)
        {
            event_.waitFor(blocking_deadline - sub_node_.getMonotonicTime());
        }

        inout_masks = uavcan::CanSelectMasks();
        for (unsigned i = 0; i < ifaces_.size(); i++)
        {
            const std::uint8_t iface_mask = 1U << i;
            inout_masks.write |= iface_mask;           // Always ready to write
            if (ifaces_[i]->hasDataInRxQueue())
            {
                inout_masks.read |= iface_mask;
            }
        }

        return ifaces_.size();       // We're always ready for write, hence > 0.
    }

    /**
     * This handler will be invoked by the main node thread.
     */
    void handleRxFrame(const uavcan::CanRxFrame& frame, uavcan::CanIOFlags flags) override
    {
        ifaces_.at(frame.iface_index)->addRxFrame(frame, flags);
        event_.signal();
    }

public:
    VirtualCanDriver(uavcan::INode& arg_sub_node, unsigned num_ifaces) :
        sub_node_(arg_sub_node)
    {
        for (unsigned i = 0; i < num_ifaces; i++)
        {
            ifaces_.emplace_back(new VirtualCanIface(arg_sub_node, mutex_));
        }
    }

    /**
     * This method should be invoked from the main node thread periodically in order to move contents of the
     * sub-node's TX queues into the TX queues of the main node. A good practice is to call this method
     * immediately after executing spinOnce() on the main node.
     * No additional locking is required.
     */
    void flushTxQueuesTo(uavcan::INode& main_node)
    {
        for (unsigned i = 0; i < ifaces_.size(); i++)
        {
            ifaces_.at(i)->flushTxQueueTo(main_node, i);
        }
        event_.signal();
    }
};

static uavcan_linux::NodePtr initMainNode(const std::vector<std::string>& ifaces, uavcan::NodeID nid,
                                          const std::string& name)
{
    std::cout << "Initializing main node" << std::endl;

    auto node = uavcan_linux::makeNode(ifaces);

    node->setNodeID(nid);
    node->setName(name.c_str());

    node->getLogger().setLevel(uavcan::protocol::debug::LogLevel::DEBUG);

    const int start_res = node->start();
    ENFORCE(0 == start_res);

    uavcan::NetworkCompatibilityCheckResult init_result;
    ENFORCE(0 == node->checkNetworkCompatibility(init_result));
    if (!init_result.isOk())
    {
        throw std::runtime_error("Network conflict with node " + std::to_string(init_result.conflicting_node.get()));
    }

    node->setStatusOk();
    return node;
}

static uavcan_linux::SubNodePtr initSubNode(const std::vector<std::string>& ifaces, uavcan::NodeID nid)
{
    std::cout << "Initializing sub node" << std::endl;
    auto node = uavcan_linux::makeSubNode(ifaces);
    node->setNodeID(nid);
    return node;
}

static void runMainNode(const uavcan_linux::NodePtr& node)
{
    std::cout << "Running main node" << std::endl;

    auto do_nothing_once_a_minute = [&node](const uavcan::TimerEvent&)
    {
        node->logInfo("timer", "Another minute passed...");
        node->setVendorSpecificStatusCode(static_cast<std::uint16_t>(std::rand())); // Setting to an arbitrary value
    };
    auto timer = node->makeTimer(uavcan::MonotonicDuration::fromMSec(60000), do_nothing_once_a_minute);

    while (true)
    {
        const int res = node->spin(uavcan::MonotonicDuration::getInfinite());
        if (res < 0)
        {
            node->logError("spin", "Error %*", res);
        }
    }
}

static void runSubNode(const uavcan_linux::SubNodePtr& node)
{
    std::cout << "Running sub node" << std::endl;

    auto log_handler = [](const uavcan::ReceivedDataStructure<uavcan::protocol::debug::LogMessage>& msg)
    {
        std::cout << msg << std::endl;
    };
    auto log_sub = node->makeSubscriber<uavcan::protocol::debug::LogMessage>(log_handler);

    struct NodeStatusMonitor : public uavcan::NodeStatusMonitor
    {
        explicit NodeStatusMonitor(uavcan::INode& node) : uavcan::NodeStatusMonitor(node) { }

        virtual void handleNodeStatusChange(const NodeStatusChangeEvent& event) override
        {
            std::cout << "Remote node NID " << int(event.node_id.get()) << " changed status: "
                      << int(event.old_status.status_code) << " --> "
                      << int(event.status.status_code) << std::endl;
        }
    };

    NodeStatusMonitor nsm(*node);
    ENFORCE(0 == nsm.start());

    while (true)
    {
        const int res = node->spin(uavcan::MonotonicDuration::getInfinite());
        if (res < 0)
        {
            std::cerr << "SubNode spin error: " << res << std::endl;
        }
    }
}

int main(int argc, const char** argv)
{
    try
    {
        if (argc < 3)
        {
            std::cerr << "Usage:\n\t" << argv[0] << " <node-id> <can-iface-name-1> [can-iface-name-N...]" << std::endl;
            return 1;
        }

        const int self_node_id = std::stoi(argv[1]);
        std::vector<std::string> iface_names(argv + 2, argv + argc);

        auto node = initMainNode(iface_names, self_node_id, "org.uavcan.linux_test_node");
        auto sub_node = initSubNode(iface_names, self_node_id);

        std::thread sub_thread([&sub_node](){ runSubNode(sub_node); });

        runMainNode(node);

        if (sub_thread.joinable())
        {
            std::cout << "Waiting for the sub thread to join" << std::endl;
            sub_thread.join();
        }

        return 0;
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Exception: " << ex.what() << std::endl;
        return 1;
    }
}
