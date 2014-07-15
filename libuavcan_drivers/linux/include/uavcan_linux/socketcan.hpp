/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <cstdint>
#include <queue>
#include <vector>
#include <map>
#include <unordered_set>
#include <algorithm>

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <poll.h>

#include <uavcan/uavcan.hpp>
#include <uavcan_linux/clock.hpp>
#include <uavcan_linux/exception.hpp>

namespace uavcan_linux
{
/**
 * SocketCan driver class keeps number of each kind of errors occurred since the object was created.
 */
enum class SocketCanError
{
    SocketReadFailure,
    SocketWriteFailure,
    TxTimeout
};

/**
 * Single SocketCAN socket interface.
 *
 * SocketCAN socket adapter maintains TX and RX queues in user space. At any moment socket's buffer contains
 * no more than 'max_frames_in_socket_tx_queue_' TX frames, rest is waiting in the user space TX queue; when the
 * socket produces loopback for the previously sent TX frame the next frame from the user space TX queue will
 * be sent into the socket.
 * This approach allows to properly maintain TX timeouts (http://stackoverflow.com/questions/19633015/).
 * TX timestamping is implemented by means of reading RX timestamps of loopback frames (see "TX timestamping" on
 * linux-can mailing list, http://permalink.gmane.org/gmane.linux.can/5322).
 *
 * This class is too complex and needs to be refactored later. At least, basic socket IO and configuration
 * should be extracted into a different class.
 */
class SocketCanIface : public uavcan::ICanIface
{
    static inline ::can_frame makeSocketCanFrame(const uavcan::CanFrame& uavcan_frame)
    {
        ::can_frame sockcan_frame { uavcan_frame.id & uavcan::CanFrame::MaskExtID, uavcan_frame.dlc, { } };
        (void)std::copy(uavcan_frame.data, uavcan_frame.data + uavcan_frame.dlc, sockcan_frame.data);
        if (uavcan_frame.isExtended())
        {
            sockcan_frame.can_id |= CAN_EFF_FLAG;
        }
        if (uavcan_frame.isErrorFrame())
        {
            sockcan_frame.can_id |= CAN_ERR_FLAG;
        }
        if (uavcan_frame.isRemoteTransmissionRequest())
        {
            sockcan_frame.can_id |= CAN_RTR_FLAG;
        }
        return sockcan_frame;
    }

    static inline uavcan::CanFrame makeUavcanFrame(const ::can_frame& sockcan_frame)
    {
        uavcan::CanFrame uavcan_frame(sockcan_frame.can_id & CAN_EFF_MASK, sockcan_frame.data, sockcan_frame.can_dlc);
        if (sockcan_frame.can_id & CAN_EFF_FLAG)
        {
            uavcan_frame.id |= uavcan::CanFrame::FlagEFF;
        }
        if (sockcan_frame.can_id & CAN_ERR_FLAG)
        {
            uavcan_frame.id |= uavcan::CanFrame::FlagERR;
        }
        if (sockcan_frame.can_id & CAN_RTR_FLAG)
        {
            uavcan_frame.id |= uavcan::CanFrame::FlagRTR;
        }
        return uavcan_frame;
    }

    struct TxItem
    {
        uavcan::CanFrame frame;
        uavcan::MonotonicTime deadline;
        uavcan::CanIOFlags flags;

        TxItem()
            : flags(0)
        { }

        TxItem(const uavcan::CanFrame& arg_frame, uavcan::MonotonicTime arg_deadline, uavcan::CanIOFlags arg_flags)
            : frame(arg_frame)
            , deadline(arg_deadline)
            , flags(arg_flags)
        { }

        bool operator<(const TxItem& rhs) const { return frame.priorityLowerThan(rhs.frame); }
    };

    struct RxItem
    {
        uavcan::CanFrame frame;
        uavcan::MonotonicTime ts_mono;
        uavcan::UtcTime ts_utc;
        uavcan::CanIOFlags flags;

        RxItem()
            : flags(0)
        { }
    };

    const SystemClock& clock_;
    const int fd_;

    const unsigned max_frames_in_socket_tx_queue_;
    unsigned frames_in_socket_tx_queue_;

    std::map<SocketCanError, std::uint64_t> errors_;

    std::priority_queue<TxItem> tx_queue_;                          // TODO: Use pool allocator
    std::queue<RxItem> rx_queue_;                                   // TODO: Use pool allocator
    std::unordered_multiset<std::uint32_t> pending_loopback_ids_;   // TODO: Use pool allocator

    void registerError(SocketCanError e) { errors_[e]++; }

    void incrementNumFramesInSocketTxQueue()
    {
        assert(frames_in_socket_tx_queue_ < max_frames_in_socket_tx_queue_);
        frames_in_socket_tx_queue_++;
    }

    void confirmSentFrame()
    {
        if (frames_in_socket_tx_queue_ > 0)
        {
            frames_in_socket_tx_queue_--;
        }
        else
        {
            assert(0); // Loopback for a frame that we didn't send.
        }
    }

    bool wasInPendingLoopbackSet(const uavcan::CanFrame& frame)
    {
        if (pending_loopback_ids_.count(frame.id) > 0)
        {
            (void)pending_loopback_ids_.erase(frame.id);
            return true;
        }
        return false;
    }

    int write(const uavcan::CanFrame& frame) const
    {
        const ::can_frame sockcan_frame = makeSocketCanFrame(frame);
        const int res = ::write(fd_, &sockcan_frame, sizeof(sockcan_frame));
        if (res <= 0)
        {
            return res;
        }
        if (res != sizeof(sockcan_frame))
        {
            return -1;
        }
        return 1;
    }

    /**
     * SocketCAN git show 1e55659ce6ddb5247cee0b1f720d77a799902b85
     *    MSG_DONTROUTE is set for any packet from localhost,
     *    MSG_CONFIRM is set for any pakcet of your socket.
     * Diff: https://git.ucsd.edu/abuss/linux/commit/1e55659ce6ddb5247cee0b1f720d77a799902b85
     * Man: https://www.kernel.org/doc/Documentation/networking/can.txt (chapter 4.1.6).
     */
    int read(uavcan::CanFrame& frame, uavcan::UtcTime& ts_utc, bool& loopback) const
    {
        auto iov = ::iovec();
        auto sockcan_frame = ::can_frame();
        iov.iov_base = &sockcan_frame;
        iov.iov_len  = sizeof(sockcan_frame);

        struct Control
        {
            cmsghdr cm;
            std::uint8_t data[sizeof(::timeval)];
        };
        auto control = Control();

        auto msg = ::msghdr();
        msg.msg_iov    = &iov;
        msg.msg_iovlen = 1;
        msg.msg_control = &control;
        msg.msg_controllen = sizeof(control);

        const int res = ::recvmsg(fd_, &msg, MSG_DONTWAIT);
        if (res <= 0)
        {
            return (res < 0 && errno == EWOULDBLOCK) ? 0 : res;
        }
        frame = makeUavcanFrame(sockcan_frame);
        /*
         * Timestamp
         */
        const ::cmsghdr* const cmsg = CMSG_FIRSTHDR(&msg);
        assert(cmsg != nullptr);
        if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMP)
        {
            auto tv = ::timeval();
            (void)std::memcpy(&tv, CMSG_DATA(cmsg), sizeof(tv));  // Copy to avoid alignment problems
            assert(tv.tv_sec >= 0 && tv.tv_usec >= 0);
            ts_utc = uavcan::UtcTime::fromUSec(std::uint64_t(tv.tv_sec) * 1000000ULL + tv.tv_usec);
        }
        else
        {
            assert(0);
            return -1;
        }
        /*
         * Flags
         */
        loopback = (msg.msg_flags & static_cast<int>(MSG_CONFIRM)) != 0;
        return 1;
    }

    void pollWrite()
    {
        while (!tx_queue_.empty() && (frames_in_socket_tx_queue_ < max_frames_in_socket_tx_queue_))
        {
            const TxItem tx = tx_queue_.top();
            tx_queue_.pop();
            assert(tx_queue_.empty() ? true : !tx.frame.priorityLowerThan(tx_queue_.top().frame)); // Order check
            if (tx.deadline >= clock_.getMonotonic())
            {
                const int res = write(tx.frame);
                if (res == 1)
                {
                    incrementNumFramesInSocketTxQueue();
                    if (tx.flags & uavcan::CanIOFlagLoopback)
                    {
                        (void)pending_loopback_ids_.insert(tx.frame.id);
                    }
                }
                else
                {
                    registerError(SocketCanError::SocketWriteFailure);
                }
            }
            else
            {
                registerError(SocketCanError::TxTimeout);
            }
        }
    }

    void pollRead()
    {
        while (true)
        {
            RxItem rx;
            rx.ts_mono = clock_.getMonotonic();  // Monotonic timestamp is not required to be precise (unlike UTC)
            bool loopback = false;
            const int res = read(rx.frame, rx.ts_utc, loopback);
            if (res == 1)
            {
                assert(!rx.ts_utc.isZero());
                bool accept = true;
                if (loopback)                   // We receive loopback for all CAN frames
                {
                    confirmSentFrame();
                    rx.flags |= uavcan::CanIOFlagLoopback;
                    accept = wasInPendingLoopbackSet(rx.frame); // Do we need to send this loopback into the lib?
                }
                if (accept)
                {
                    rx.ts_utc += clock_.getPrivateAdjustment();
                    rx_queue_.push(rx);
                }
            }
            else if (res == 0)
            {
                break;
            }
            else
            {
                registerError(SocketCanError::SocketReadFailure);
            }
        }
    }

public:
    /**
     * Takes ownership of socket's file descriptor.
     */
    SocketCanIface(const SystemClock& clock, int socket_fd, int max_frames_in_socket_tx_queue = 3)
        : clock_(clock)
        , fd_(socket_fd)
        , max_frames_in_socket_tx_queue_(max_frames_in_socket_tx_queue)
        , frames_in_socket_tx_queue_(0)
    {
        assert(fd_ >= 0);
    }

    /**
     * Socket file descriptor will be closed.
     */
    virtual ~SocketCanIface()
    {
        (void)::close(fd_);
    }

    /**
     * Assumes that the socket is writeable
     */
    virtual std::int16_t send(const uavcan::CanFrame& frame, const uavcan::MonotonicTime tx_deadline,
                              const uavcan::CanIOFlags flags)
    {
        tx_queue_.emplace(frame, tx_deadline, flags);
        pollRead();     // Read poll is necessary because it can release the pending TX flag
        pollWrite();
        return 1;
    }

    /**
     * Will read the socket only if RX queue is empty.
     * Normally, poll() needs to be executed first.
     */
    virtual std::int16_t receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                                 uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags)
    {
        if (rx_queue_.empty())
        {
            pollRead();            // This allows to use the socket not calling poll() explicitly.
            if (rx_queue_.empty())
            {
                return 0;
            }
        }
        {
            const RxItem& rx = rx_queue_.front();
            out_frame        = rx.frame;
            out_ts_monotonic = rx.ts_mono;
            out_ts_utc       = rx.ts_utc;
            out_flags        = rx.flags;
        }
        rx_queue_.pop();
        return 1;
    }

    /**
     * Performs socket read/write.
     * @param read  Socket is readable
     * @param write Socket is writeable
     */
    void poll(bool read, bool write)
    {
        if (read)
        {
            pollRead();  // Read poll must be executed first because it may decrement frames_in_socket_tx_queue_
        }
        if (write)
        {
            pollWrite();
        }
    }

    bool hasPendingTx() const { return !tx_queue_.empty(); }
    bool hasReadyRx()   const { return !rx_queue_.empty(); }

    virtual std::int16_t configureFilters(const uavcan::CanFilterConfig* const filter_configs,
                                          const std::uint16_t num_configs)
    {
        if (filter_configs == nullptr)
        {
            assert(0);
            return -1;
        }
        std::vector< ::can_filter> filts(num_configs);
        for (unsigned i = 0; i < num_configs; i++)
        {
            const uavcan::CanFilterConfig& fc = filter_configs[i];
            filts[i].can_id   = fc.id   & uavcan::CanFrame::MaskExtID;
            filts[i].can_mask = fc.mask & uavcan::CanFrame::MaskExtID;
            if (fc.id & uavcan::CanFrame::FlagEFF)
            {
                filts[i].can_id |= CAN_EFF_FLAG;
            }
            if (fc.id & uavcan::CanFrame::FlagRTR)
            {
                filts[i].can_id |= CAN_RTR_FLAG;
            }
            if (fc.mask & uavcan::CanFrame::FlagEFF)
            {
                filts[i].can_mask |= CAN_EFF_FLAG;
            }
            if (fc.mask & uavcan::CanFrame::FlagRTR)
            {
                filts[i].can_mask |= CAN_RTR_FLAG;
            }
        }
        int ret = setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filts.data(), sizeof(::can_filter) * num_configs);
        return (ret < 0) ? -1 : 0;
    }

    /**
     * SocketCAN emulates the CAN filters in software, so the number of filters is virtually unlimited.
     * This method returns a constant value.
     */
    virtual std::uint16_t getNumFilters() const { return 255; }

    /**
     * Returns total number of errors of each kind detected since the object was created.
     */
    virtual std::uint64_t getErrorCount() const
    {
        std::uint64_t ec = 0;
        for (auto& kv : errors_) { ec += kv.second; }
        return ec;
    }

    /**
     * Returns number of errors of each kind in a map.
     */
    const decltype(errors_)& getErrors() const { return errors_; }

    int getFileDescriptor() const { return fd_; }

    /**
     * Open and configure a CAN socket on iface specified by name.
     * @param iface_name String containing iface name, e.g. "can0", "vcan1", "slcan0"
     * @return Socket descriptor or negative number on error.
     */
    static int openSocket(const std::string& iface_name)
    {
        const int s = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (s < 0)
        {
            return s;
        }
        // Detect the iface index
        auto ifr = ::ifreq();
        if (iface_name.length() >= IFNAMSIZ)
        {
            goto fail;
        }
        (void)std::strncpy(ifr.ifr_name, iface_name.c_str(), iface_name.length());
        if (::ioctl(s, SIOCGIFINDEX, &ifr) < 0 || ifr.ifr_ifindex < 0)
        {
            goto fail;
        }
        // Bind to a CAN iface
        {
            auto addr = ::sockaddr_can();
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;
            if (::bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
            {
                goto fail;
            }
        }
        // Configure
        {
            const int on = 1;
            // Timestamping
            if (::setsockopt(s, SOL_SOCKET, SO_TIMESTAMP, &on, sizeof(on)) < 0)
            {
                goto fail;
            }
            // Socket loopback
            if (::setsockopt(s, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &on, sizeof(on)) < 0)
            {
                goto fail;
            }
            // Non-blocking
            if (::fcntl(s, F_SETFL , O_NONBLOCK) < 0)
            {
                goto fail;
            }
        }
        return s;

    fail:
        (void)::close(s);
        return -1;
    }
};

/**
 * Multiplexing container for multiple SocketCAN sockets.
 * Uses ppoll() for multiplexing.
 */
class SocketCanDriver : public uavcan::ICanDriver
{
public:
    static constexpr unsigned MaxIfaces = uavcan::MaxCanIfaces;

private:
    const SystemClock& clock_;
    uavcan::LazyConstructor<SocketCanIface> ifaces_[MaxIfaces];
    ::pollfd pollfds_[MaxIfaces];
    std::uint8_t num_ifaces_;

public:
    /**
     * Reference to the clock object shall remain valid.
     */
    explicit SocketCanDriver(const SystemClock& clock)
        : clock_(clock)
        , num_ifaces_(0)
    {
        for (auto& p : pollfds_)
        {
            p = ::pollfd();
            p.fd = -1;
        }
    }

    /**
     * This function may return before deadline expiration even if no requested IO operations become possible.
     * This behavior makes implementation way simpler, and it is OK since libuavcan can properly handle such
     * early returns.
     * Also it can return more events than were originally requested by uavcan, which is also acceptable.
     */
    virtual std::int16_t select(uavcan::CanSelectMasks& inout_masks, uavcan::MonotonicTime blocking_deadline)
    {
        // Detecting whether we need to block at all
        bool need_block = (inout_masks.write == 0);    // Write queue is infinite
        for (unsigned i = 0; need_block && (i < num_ifaces_); i++)
        {
            const bool need_read = inout_masks.read  & (1 << i);
            if (need_read && ifaces_[i]->hasReadyRx())
            {
                need_block = false;
            }
        }

        if (need_block)
        {
            // Poll FD set setup
            for (unsigned i = 0; i < num_ifaces_; i++)
            {
                pollfds_[i].events = POLLIN;
                if (ifaces_[i]->hasPendingTx() || (inout_masks.write & (1 << i)))
                {
                    pollfds_[i].events |= POLLOUT;
                }
            }

            // Timeout conversion
            const std::int64_t timeout_usec = (blocking_deadline - clock_.getMonotonic()).toUSec();
            auto ts = ::timespec();
            if (timeout_usec > 0)
            {
                ts.tv_sec = timeout_usec / 1000000LL;
                ts.tv_nsec = (timeout_usec % 1000000LL) * 1000;
            }

            // Blocking here
            const int res = ::ppoll(pollfds_, num_ifaces_, &ts, nullptr);
            if (res < 0)
            {
                return res;
            }

            // Handling poll output
            for (unsigned i = 0; i < num_ifaces_; i++)
            {
                const bool poll_read  = pollfds_[i].revents & POLLIN;
                const bool poll_write = pollfds_[i].revents & POLLOUT;
                ifaces_[i]->poll(poll_read, poll_write);
            }
        }

        // Writing the output masks
        inout_masks = uavcan::CanSelectMasks();
        for (unsigned i = 0; i < num_ifaces_; i++)
        {
            const std::uint8_t iface_mask = 1 << i;
            inout_masks.write |= iface_mask;           // Always ready to write
            if (ifaces_[i]->hasReadyRx())
            {
                inout_masks.read |= iface_mask;
            }
        }
        // Since all ifaces are always ready to write, return value is always the same
        return num_ifaces_;
    }

    virtual SocketCanIface* getIface(std::uint8_t iface_index)
    {
        return (iface_index >= num_ifaces_) ? nullptr : static_cast<SocketCanIface*>(ifaces_[iface_index]);
    }

    virtual std::uint8_t getNumIfaces() const { return num_ifaces_; }

    /**
     * Adds one iface by name. Will fail if there are @ref MaxIfaces ifaces registered already.
     * @param iface_name E.g. "can0", "vcan1"
     * @return Negative on error, zero on success.
     * @throws uavcan_linux::Exception.
     */
    int addIface(const std::string& iface_name)
    {
        if (num_ifaces_ >= MaxIfaces)
        {
            return -1;
        }
        // Open the socket
        const int fd = SocketCanIface::openSocket(iface_name);
        if (fd < 0)
        {
            return fd;
        }
        // Construct the iface - upon successful construction the iface will take ownership of the fd.
        try
        {
            ifaces_[num_ifaces_].construct<const SystemClock&, int>(clock_, fd);
        }
        catch (...)
        {
            (void)::close(fd);
            throw;
        }
        // Init pollfd
        pollfds_[num_ifaces_].fd = fd;
        num_ifaces_++;
        return 0;
    }
};

}
