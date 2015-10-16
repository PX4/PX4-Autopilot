/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef NDEBUG
# define UAVCAN_DEBUG 1
#endif

#include <iostream>
#include <thread>
#include <condition_variable>
#include <uavcan_linux/uavcan_linux.hpp>
#include <uavcan/node/sub_node.hpp>
#include <uavcan/protocol/node_status_monitor.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>
#include "debug.hpp"

/**
 * Generic queue based on the linked list class defined in libuavcan.
 * This class does not use heap memory.
 */
template <typename T>
class Queue
{
    struct Item : public uavcan::LinkedListNode<Item>
    {
        T payload;

        template <typename... Args>
        Item(Args... args) : payload(args...) { }
    };

    uavcan::LimitedPoolAllocator allocator_;
    uavcan::LinkedListRoot<Item> list_;

public:
    Queue(uavcan::IPoolAllocator& arg_allocator, std::size_t block_allocation_quota) :
        allocator_(arg_allocator, block_allocation_quota)
    {
        uavcan::IsDynamicallyAllocatable<Item>::check();
    }

    bool isEmpty() const { return list_.isEmpty(); }

    /**
     * Creates one item in-place at the end of the list.
     * Returns true if the item was appended successfully, false if there's not enough memory.
     * Complexity is O(N) where N is queue length.
     */
    template <typename... Args>
    bool tryEmplace(Args... args)
    {
        // Allocating memory
        void* const ptr = allocator_.allocate(sizeof(Item));
        if (ptr == nullptr)
        {
            return false;
        }

        // Constructing the new item
        Item* const item = new (ptr) Item(args...);
        assert(item != nullptr);

        // Inserting the new item at the end of the list
        Item* p = list_.get();
        if (p == nullptr)
        {
            list_.insert(item);
        }
        else
        {
            while (p->getNextListNode() != nullptr)
            {
                p = p->getNextListNode();
            }
            assert(p->getNextListNode() == nullptr);
            p->setNextListNode(item);
            assert(p->getNextListNode()->getNextListNode() == nullptr);
        }

        return true;
    }

    /**
     * Accesses the first element.
     * Nullptr will be returned if the queue is empty.
     * Complexity is O(1).
     */
    T*       peek()       { return isEmpty() ? nullptr : &list_.get()->payload; }
    const T* peek() const { return isEmpty() ? nullptr : &list_.get()->payload; }

    /**
     * Removes the first element.
     * If the queue is empty, nothing will be done and assertion failure will be triggered.
     * Complexity is O(1).
     */
    void pop()
    {
        Item* const item = list_.get();
        assert(item != nullptr);
        if (item != nullptr)
        {
            list_.remove(item);
            item->~Item();
            allocator_.deallocate(item);
        }
    }
};

/**
 * Feel free to remove.
 */
static void testQueue()
{
    uavcan::PoolAllocator<1024, uavcan::MemPoolBlockSize> allocator;
    Queue<typename uavcan::MakeString<50>::Type> q(allocator, 4);
    ENFORCE(q.isEmpty());
    ENFORCE(q.peek() == nullptr);
    ENFORCE(q.tryEmplace("One"));
    ENFORCE(q.tryEmplace("Two"));
    ENFORCE(q.tryEmplace("Three"));
    ENFORCE(q.tryEmplace("Four"));
    ENFORCE(!q.tryEmplace("Five"));
    ENFORCE(*q.peek() == "One");
    q.pop();
    ENFORCE(*q.peek() == "Two");
    q.pop();
    ENFORCE(*q.peek() == "Three");
    q.pop();
    ENFORCE(*q.peek() == "Four");
    q.pop();
    ENFORCE(q.isEmpty());
    ENFORCE(q.peek() == nullptr);
}

/**
 * Objects of this class are owned by the sub-node thread.
 * This class does not use heap memory.
 */
class VirtualCanIface : public uavcan::ICanIface,
                        uavcan::Noncopyable
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
    uavcan::CanTxQueue prioritized_tx_queue_;
    Queue<RxItem> rx_queue_;

    int16_t send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline, uavcan::CanIOFlags flags) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        prioritized_tx_queue_.push(frame, tx_deadline, uavcan::CanTxQueue::Volatile, flags);
        return 1;
    }

    int16_t receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                    uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags) override
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (rx_queue_.isEmpty())
        {
            return 0;
        }

        const auto item = *rx_queue_.peek();
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

public:
    VirtualCanIface(uavcan::IPoolAllocator& allocator, uavcan::ISystemClock& clock,
                    std::mutex& arg_mutex, unsigned quota_per_queue) :
        mutex_(arg_mutex),
        prioritized_tx_queue_(allocator, clock, quota_per_queue),
        rx_queue_(allocator, quota_per_queue)
    { }

    /**
     * Note that RX queue overwrites oldest items when overflowed.
     * Call this from the main thread only.
     * No additional locking is required.
     */
    void addRxFrame(const uavcan::CanRxFrame& frame, uavcan::CanIOFlags flags)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!rx_queue_.tryEmplace(frame, flags) && !rx_queue_.isEmpty())
        {
            rx_queue_.pop();
            (void)rx_queue_.tryEmplace(frame, flags);
        }
    }

    /**
     * Call this from the main thread only.
     * No additional locking is required.
     */
    void flushTxQueueTo(uavcan::INode& main_node, std::uint8_t iface_index)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        const std::uint8_t iface_mask = static_cast<std::uint8_t>(1U << iface_index);

        while (auto e = prioritized_tx_queue_.peek())
        {
            UAVCAN_TRACE("VirtualCanIface", "TX injection [iface=0x%02x]: %s",
                         unsigned(iface_mask), e->toString().c_str());

            const int res = main_node.injectTxFrame(e->frame, e->deadline, iface_mask,
                                                    uavcan::CanTxQueue::Qos(e->qos), e->flags);
            prioritized_tx_queue_.remove(e);
            if (res <= 0)
            {
                break;
            }
        }
    }

    /**
     * Call this from the sub-node thread only.
     * No additional locking is required.
     */
    bool hasDataInRxQueue() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return !rx_queue_.isEmpty();
    }
};

/**
 * This interface defines one method that will be called by the main node thread periodically in order to
 * transfer contents of TX queue of the sub-node into the TX queue of the main node.
 */
class ITxQueueInjector
{
public:
    virtual ~ITxQueueInjector() { }

    /**
     * Flush contents of TX queues into the main node.
     * @param main_node         Reference to the main node.
     */
    virtual void injectTxFramesInto(uavcan::INode& main_node) = 0;
};

/**
 * Objects of this class are owned by the sub-node thread.
 * This class does not use heap memory.
 * @tparam SharedMemoryPoolSize         Amount of memory, in bytes, that will be statically allocated for the
 *                                      memory pool that will be shared across all interfaces for RX/TX queues.
 *                                      Typically this value should be no less than 4K per interface.
 */
template <unsigned SharedMemoryPoolSize>
class VirtualCanDriver : public uavcan::ICanDriver,
                         public uavcan::IRxFrameListener,
                         public ITxQueueInjector,
                         uavcan::Noncopyable
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

    Event event_;               ///< Used to unblock the select() call when IO happens.
    std::mutex mutex_;                                                                  ///< Shared across all ifaces
    uavcan::PoolAllocator<SharedMemoryPoolSize, uavcan::MemPoolBlockSize> allocator_;   ///< Shared across all ifaces
    uavcan::LazyConstructor<VirtualCanIface> ifaces_[uavcan::MaxCanIfaces];
    const unsigned num_ifaces_;
    uavcan_linux::SystemClock clock_;

    uavcan::ICanIface* getIface(uint8_t iface_index) override
    {
        return (iface_index < num_ifaces_) ? ifaces_[iface_index].operator VirtualCanIface*() : nullptr;
    }

    uint8_t getNumIfaces() const override { return num_ifaces_; }

    /**
     * This and other methods of ICanDriver will be invoked by the sub-node thread.
     */
    int16_t select(uavcan::CanSelectMasks& inout_masks,
                   const uavcan::CanFrame* (&)[uavcan::MaxCanIfaces],
                   uavcan::MonotonicTime blocking_deadline) override
    {
        bool need_block = (inout_masks.write == 0);    // Write queue is infinite
        for (unsigned i = 0; need_block && (i < num_ifaces_); i++)
        {
            const bool need_read = inout_masks.read & (1U << i);
            if (need_read && ifaces_[i]->hasDataInRxQueue())
            {
                need_block = false;
            }
        }

        if (need_block)
        {
            event_.waitFor(blocking_deadline - clock_.getMonotonic());
        }

        inout_masks = uavcan::CanSelectMasks();
        for (unsigned i = 0; i < num_ifaces_; i++)
        {
            const std::uint8_t iface_mask = 1U << i;
            inout_masks.write |= iface_mask;           // Always ready to write
            if (ifaces_[i]->hasDataInRxQueue())
            {
                inout_masks.read |= iface_mask;
            }
        }

        return num_ifaces_;       // We're always ready to write, hence > 0.
    }

    /**
     * This handler will be invoked by the main node thread.
     */
    void handleRxFrame(const uavcan::CanRxFrame& frame, uavcan::CanIOFlags flags) override
    {
        UAVCAN_TRACE("VirtualCanDriver", "RX [flags=%u]: %s", unsigned(flags), frame.toString().c_str());
        if (frame.iface_index < num_ifaces_)
        {
            ifaces_[frame.iface_index]->addRxFrame(frame, flags);
            event_.signal();
        }
    }

    /**
     * This method will be invoked by the main node thread.
     */
    void injectTxFramesInto(uavcan::INode& main_node) override
    {
        for (unsigned i = 0; i < num_ifaces_; i++)
        {
            ifaces_[i]->flushTxQueueTo(main_node, i);
        }
        event_.signal();
    }

public:
    VirtualCanDriver(unsigned arg_num_ifaces) : num_ifaces_(arg_num_ifaces)
    {
        assert(num_ifaces_ > 0 && num_ifaces_ <= uavcan::MaxCanIfaces);

        const unsigned quota_per_iface = allocator_.getBlockCapacity() / num_ifaces_;
        const unsigned quota_per_queue = quota_per_iface;             // 2x overcommit

        UAVCAN_TRACE("VirtualCanDriver", "Total blocks: %u, quota per queue: %u",
                     unsigned(allocator_.getBlockCapacity()), unsigned(quota_per_queue));

        for (unsigned i = 0; i < num_ifaces_; i++)
        {
            ifaces_[i].template construct<uavcan::IPoolAllocator&, uavcan::ISystemClock&,
                                          std::mutex&, unsigned>(allocator_, clock_, mutex_, quota_per_queue);
        }
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

    node->setModeOperational();
    return node;
}

template <unsigned QueuePoolSize>
static uavcan_linux::SubNodePtr initSubNode(unsigned num_ifaces, uavcan::INode& main_node)
{
    std::cout << "Initializing sub node" << std::endl;

    typedef VirtualCanDriver<QueuePoolSize> Driver;

    std::shared_ptr<Driver> driver(new Driver(num_ifaces));
    auto node = uavcan_linux::makeSubNode(driver);
    node->setNodeID(main_node.getNodeID());

    main_node.getDispatcher().installRxFrameListener(driver.get());

    return node;
}

static void runMainNode(const uavcan_linux::NodePtr& node)
{
    std::cout << "Running main node" << std::endl;

    auto timer = node->makeTimer(uavcan::MonotonicDuration::fromMSec(10000), [&node](const uavcan::TimerEvent&)
        {
            node->logInfo("timer", "Your time is running out.");
            // coverity[dont_call]
            node->setVendorSpecificStatusCode(static_cast<std::uint16_t>(std::rand()));
        });

    /*
     * We know that in this implementation, VirtualCanDriver inherits uavcan::IRxFrameListener, so we can simply
     * restore the reference to ITxQueueInjector using dynamic_cast. In other implementations this may be
     * unacceptable, so a reference to ITxQueueInjector will have to be passed using some other means.
     */
    if (node->getDispatcher().getRxFrameListener() == nullptr)
    {
        throw std::logic_error("RX frame listener is not configured");
    }
    ITxQueueInjector& tx_injector = dynamic_cast<ITxQueueInjector&>(*node->getDispatcher().getRxFrameListener());

    while (true)
    {
        const int res = node->spin(uavcan::MonotonicDuration::fromMSec(1));
        if (res < 0)
        {
            node->logError("spin", "Error %*", res);
        }
        // TX queue transfer occurs here.
        tx_injector.injectTxFramesInto(*node);
    }
}

static void runSubNode(const uavcan_linux::SubNodePtr& node)
{
    std::cout << "Running sub node" << std::endl;

    /*
     * Log subscriber
     */
    auto log_sub = node->makeSubscriber<uavcan::protocol::debug::LogMessage>(
        [](const uavcan::ReceivedDataStructure<uavcan::protocol::debug::LogMessage>& msg)
        {
            std::cout << msg << std::endl;
        });

    /*
     * Node status monitor
     */
    struct NodeStatusMonitor : public uavcan::NodeStatusMonitor
    {
        explicit NodeStatusMonitor(uavcan::INode& node) : uavcan::NodeStatusMonitor(node) { }

        virtual void handleNodeStatusChange(const NodeStatusChangeEvent& event) override
        {
            std::cout << "Remote node NID " << int(event.node_id.get()) << " changed status: "
                      << event.old_status.toString() << " --> " << event.status.toString() << std::endl;
        }
    };
    NodeStatusMonitor nsm(*node);
    ENFORCE(0 == nsm.start());

    /*
     * KV subscriber
     */
    auto kv_sub = node->makeSubscriber<uavcan::protocol::debug::KeyValue>(
        [](const uavcan::ReceivedDataStructure<uavcan::protocol::debug::KeyValue>& msg)
        {
            std::cout << msg << std::endl;
        });

    /*
     * KV publisher
     */
    unsigned kv_value = 0;
    auto kv_pub = node->makePublisher<uavcan::protocol::debug::KeyValue>();
    auto timer = node->makeTimer(uavcan::MonotonicDuration::fromMSec(5000), [&](const uavcan::TimerEvent&)
        {
            uavcan::protocol::debug::KeyValue kv;
            kv.key = "five_seconds";
            kv.value = kv_value++;
            const int res = kv_pub->broadcast(kv);
            if (res < 0)
            {
                std::cerr << "Sub KV pub err " << res << std::endl;
            }
        });

    while (true)
    {
        const int res = node->spin(uavcan::MonotonicDuration::fromMSec(1000));
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
        testQueue();

        constexpr unsigned VirtualIfacePoolSize = 32768;

        if (argc < 3)
        {
            std::cerr << "Usage:\n\t" << argv[0] << " <node-id> <can-iface-name-1> [can-iface-name-N...]" << std::endl;
            return 1;
        }

        const int self_node_id = std::stoi(argv[1]);
        std::vector<std::string> iface_names(argv + 2, argv + argc);

        auto node = initMainNode(iface_names, self_node_id, "org.uavcan.linux_test_node");
        auto sub_node = initSubNode<VirtualIfacePoolSize>(iface_names.size(), *node);

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
