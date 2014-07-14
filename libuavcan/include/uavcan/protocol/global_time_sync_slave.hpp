/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/node/subscriber.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/protocol/GlobalTimeSync.hpp>

namespace uavcan
{
/**
 * Please read the specs to learn how the time synchronization works.
 *
 * No more than one object of this class is allowed per node; otherwise a disaster is bound to happen.
 *
 * NOTE: In order for this class to work, the platform driver must implement:
 *  - CAN bus RX UTC timestamping;
 *  - Clock adjustment method in the system clock interface @ref ISystemClock::adjustUtc().
 *
 * Ref. M. Gergeleit, H. Streich - "Implementing a Distributed High-Resolution Real-Time Clock using the CAN-Bus"
 * http://modecs.cs.uni-salzburg.at/results/related_documents/CAN_clock.pdf
 */
class UAVCAN_EXPORT GlobalTimeSyncSlave : Noncopyable
{
    typedef MethodBinder<GlobalTimeSyncSlave*,
                         void (GlobalTimeSyncSlave::*)(const ReceivedDataStructure<protocol::GlobalTimeSync>&)>
        GlobalTimeSyncCallback;

    // Static buffers are explicitly disabled because time should never be unicasted.
    Subscriber<protocol::GlobalTimeSync, GlobalTimeSyncCallback, 2, 0> sub_;

    UtcTime prev_ts_utc_;
    MonotonicTime prev_ts_mono_;
    MonotonicTime last_adjustment_ts_;
    enum State { Update, Adjust } state_;
    NodeID master_nid_;
    TransferID prev_tid_;
    uint8_t prev_iface_index_;
    bool suppressed_;

    ISystemClock& getSystemClock() const { return sub_.getNode().getSystemClock(); }

    void adjustFromMsg(const ReceivedDataStructure<protocol::GlobalTimeSync>& msg);

    void updateFromMsg(const ReceivedDataStructure<protocol::GlobalTimeSync>& msg);

    void processMsg(const ReceivedDataStructure<protocol::GlobalTimeSync>& msg);

    void handleGlobalTimeSync(const ReceivedDataStructure<protocol::GlobalTimeSync>& msg);

public:
    explicit GlobalTimeSyncSlave(INode& node)
        : sub_(node)
        , state_(Update)
        , prev_iface_index_(0xFF)
        , suppressed_(false)
    { }

    /**
     * Starts the time sync slave. Once started, it works on its own and does not require any
     * attention from the application, other than to handle a clock adjustment request occasionally.
     * Returns negative error code.
     */
    int start();

    /**
     * Enable or disable the suppressed mode.
     *
     * In suppressed mode the slave will continue tracking time sync masters in the network, but will not
     * perform local clock adjustments. So it's kind of a dry run - all the time sync logic works except
     * the local clock will not receive adjustments.
     *
     * Suppressed mode is useful for nodes that can act as a back-up clock sync masters - as long as the
     * node sees a higher priority time sync master in the network, its slave will be NOT suppressed
     * in order to sync the local clock with the global master. As soon as all other higher priority
     * masters go down, the local node will suppress its time sync slave instance and become a new master.
     *
     * Suppressed mode is disabled by default.
     */
    void suppress(bool suppressed) { suppressed_ = suppressed; }
    bool isSuppressed() const { return suppressed_; }

    /**
     * If the clock sync slave sees any clock sync masters in the network, it is ACTIVE.
     * When the last master times out (PUBLISHER_TIMEOUT), the slave will be INACTIVE.
     * Note that immediately after start up the slave will be INACTIVE until it finds a master.
     * Please read the specs to learn more.
     */
    bool isActive() const;

    /**
     * Node ID of the master the slave is currently locked on.
     * Returns an invalid Node ID if there's no active master.
     */
    NodeID getMasterNodeID() const;

    /**
     * Last time when the local clock adjustment was performed.
     */
    MonotonicTime getLastAdjustmentTime() const { return last_adjustment_ts_; }
};

}
