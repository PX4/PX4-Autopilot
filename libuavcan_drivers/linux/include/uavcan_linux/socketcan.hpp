/*
 * Copyright (C) 2014-2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 *                         Ilia Sheremet <illia.sheremet@gmail.com>
 */

#pragma once

#include <cassert>
#include <cstdint>
#include <queue>
#include <vector>
#include <map>
#include <unordered_set>
#include <memory>
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
 *
 * This approach allows to properly maintain TX timeouts (http://stackoverflow.com/questions/19633015/).
 * TX timestamping is implemented by means of reading RX timestamps of loopback frames (see "TX timestamping" on
 * linux-can mailing list, http://permalink.gmane.org/gmane.linux.can/5322).
 *
 * Note that if max_frames_in_socket_tx_queue_ is greater than one, frame reordering may occur (depending on the
 * unrderlying logic).
 *
 * This class is too complex and needs to be refactored later. At least, basic socket IO and configuration
 * should be extracted into a different class.
 */
class SocketCanIface : public uavcan::ICanIface
{
    static inline ::can_frame makeSocketCanFrame(const uavcan::CanFrame& uavcan_frame)
    {
        ::can_frame sockcan_frame { uavcan_frame.id& uavcan::CanFrame::MaskExtID, uavcan_frame.dlc, { } };
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
        uavcan::CanIOFlags flags = 0;
        std::uint64_t order = 0;

        TxItem(const uavcan::CanFrame& arg_frame, uavcan::MonotonicTime arg_deadline,
               uavcan::CanIOFlags arg_flags, std::uint64_t arg_order)
            : frame(arg_frame)
            , deadline(arg_deadline)
            , flags(arg_flags)
            , order(arg_order)
        { }

        bool operator<(const TxItem& rhs) const
        {
            if (frame.priorityLowerThan(rhs.frame))
            {
                return true;
            }
            if (frame.priorityHigherThan(rhs.frame))
            {
                return false;
            }
            return order > rhs.order;
        }
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
    unsigned frames_in_socket_tx_queue_ = 0;

    std::uint64_t tx_frame_counter_ = 0;        ///< Increments with every frame pushed into the TX queue

    std::map<SocketCanError, std::uint64_t> errors_;

    std::priority_queue<TxItem> tx_queue_;                          // TODO: Use pool allocator
    std::queue<RxItem> rx_queue_;                                   // TODO: Use pool allocator
    std::unordered_multiset<std::uint32_t> pending_loopback_ids_;   // TODO: Use pool allocator

    std::vector<::can_filter> hw_filters_container_;

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
        errno = 0;

        const ::can_frame sockcan_frame = makeSocketCanFrame(frame);

        const int res = ::write(fd_, &sockcan_frame, sizeof(sockcan_frame));
        if (res <= 0)
        {
            if (errno == ENOBUFS || errno == EAGAIN)    // Writing is not possible atm, not an error
            {
                return 0;
            }
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
        /*
         * Flags
         */
        loopback = (msg.msg_flags & static_cast<int>(MSG_CONFIRM)) != 0;

        if (!loopback && !checkHWFilters(sockcan_frame))
        {
            return 0;
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
        return 1;
    }

    void pollWrite()
    {
        while (hasReadyTx())
        {
            const TxItem tx = tx_queue_.top();

            if (tx.deadline >= clock_.getMonotonic())
            {
                const int res = write(tx.frame);
                if (res == 1)                   // Transmitted successfully
                {
                    incrementNumFramesInSocketTxQueue();
                    if (tx.flags & uavcan::CanIOFlagLoopback)
                    {
                        (void)pending_loopback_ids_.insert(tx.frame.id);
                    }
                }
                else if (res == 0)              // Not transmitted, nor is it an error
                {
                    break;                      // Leaving the loop, the frame remains enqueued for the next retry
                }
                else                            // Transmission error
                {
                    registerError(SocketCanError::SocketWriteFailure);
                }
            }
            else
            {
                registerError(SocketCanError::TxTimeout);
            }

            // Removing the frame from the queue even if transmission failed
            tx_queue_.pop();
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
                break;
            }
        }
    }

    /**
     * Returns true if a frame accepted by HW filters
     */
    bool checkHWFilters(const ::can_frame& frame) const
    {
        if (!hw_filters_container_.empty())
        {
            for (auto& f : hw_filters_container_)
            {
                if (((frame.can_id & f.can_mask) ^ f.can_id) == 0)
                {
                    return true;
                }
            }
            return false;
        }
        else
        {
            return true;
        }
    }

public:
    /**
     * Takes ownership of socket's file descriptor.
     *
     * @ref max_frames_in_socket_tx_queue       See a note in the class comment.
     */
    SocketCanIface(const SystemClock& clock, int socket_fd, int max_frames_in_socket_tx_queue = 2)
        : clock_(clock)
        , fd_(socket_fd)
        , max_frames_in_socket_tx_queue_(max_frames_in_socket_tx_queue)
    {
        assert(fd_ >= 0);
    }

    /**
     * Socket file descriptor will be closed.
     */
    virtual ~SocketCanIface()
    {
        UAVCAN_TRACE("SocketCAN", "SocketCanIface: Closing fd %d", fd_);
        (void)::close(fd_);
    }

    /**
     * Assumes that the socket is writeable
     */
    std::int16_t send(const uavcan::CanFrame& frame, const uavcan::MonotonicTime tx_deadline,
                      const uavcan::CanIOFlags flags) override
    {
        tx_queue_.emplace(frame, tx_deadline, flags, tx_frame_counter_);
        tx_frame_counter_++;
        pollRead();     // Read poll is necessary because it can release the pending TX flag
        pollWrite();
        return 1;
    }

    /**
     * Will read the socket only if RX queue is empty.
     * Normally, poll() needs to be executed first.
     */
    std::int16_t receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                         uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags) override
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

    bool hasReadyRx() const { return !rx_queue_.empty(); }
    bool hasReadyTx() const
    {
        return !tx_queue_.empty() && (frames_in_socket_tx_queue_ < max_frames_in_socket_tx_queue_);
    }

    std::int16_t configureFilters(const uavcan::CanFilterConfig* const filter_configs,
                                  const std::uint16_t num_configs) override
    {
        if (filter_configs == nullptr)
        {
            assert(0);
            return -1;
        }
        hw_filters_container_.clear();
        hw_filters_container_.resize(num_configs);

        for (unsigned i = 0; i < num_configs; i++)
        {
            const uavcan::CanFilterConfig& fc = filter_configs[i];
            hw_filters_container_[i].can_id   = fc.id   & uavcan::CanFrame::MaskExtID;
            hw_filters_container_[i].can_mask = fc.mask & uavcan::CanFrame::MaskExtID;
            if (fc.id & uavcan::CanFrame::FlagEFF)
            {
                hw_filters_container_[i].can_id |= CAN_EFF_FLAG;
            }
            if (fc.id & uavcan::CanFrame::FlagRTR)
            {
                hw_filters_container_[i].can_id |= CAN_RTR_FLAG;
            }
            if (fc.mask & uavcan::CanFrame::FlagEFF)
            {
                hw_filters_container_[i].can_mask |= CAN_EFF_FLAG;
            }
            if (fc.mask & uavcan::CanFrame::FlagRTR)
            {
                hw_filters_container_[i].can_mask |= CAN_RTR_FLAG;
            }
        }

        return 0;
    }

    /**
     * SocketCAN emulates the CAN filters in software, so the number of filters is virtually unlimited.
     * This method returns a constant value.
     */
    static constexpr unsigned NumFilters = 8;
    std::uint16_t getNumFilters() const override { return NumFilters; }


    /**
     * Returns total number of errors of each kind detected since the object was created.
     */
    std::uint64_t getErrorCount() const override
    {
        std::uint64_t ec = 0;
        for (auto& kv : errors_) { ec += kv.second; }
        return ec;
    }

    /**
     * Returns number of errors of each kind in a map.
     */
    const decltype(errors_) & getErrors() const { return errors_; }

    int getFileDescriptor() const { return fd_; }

    /**
     * Open and configure a CAN socket on iface specified by name.
     * @param iface_name String containing iface name, e.g. "can0", "vcan1", "slcan0"
     * @return Socket descriptor or negative number on error.
     */
    static int openSocket(const std::string& iface_name)
    {
        errno = 0;

        const int s = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (s < 0)
        {
            return s;
        }

        class RaiiCloser
        {
            int fd_;
        public:
            RaiiCloser(int filedesc) : fd_(filedesc)
            {
                assert(fd_ >= 0);
            }
            ~RaiiCloser()
            {
                if (fd_ >= 0)
                {
                    UAVCAN_TRACE("SocketCAN", "RaiiCloser: Closing fd %d", fd_);
                    (void)::close(fd_);
                }
            }
            void disarm() { fd_ = -1; }
        } raii_closer(s);

        // Detect the iface index
        auto ifr = ::ifreq();
        if (iface_name.length() >= IFNAMSIZ)
        {
            errno = ENAMETOOLONG;
            return -1;
        }
        (void)std::strncpy(ifr.ifr_name, iface_name.c_str(), iface_name.length());
        if (::ioctl(s, SIOCGIFINDEX, &ifr) < 0 || ifr.ifr_ifindex < 0)
        {
            return -1;
        }

        // Bind to the specified CAN iface
        {
            auto addr = ::sockaddr_can();
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;
            if (::bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
            {
                return -1;
            }
        }

        // Configure
        {
            const int on = 1;
            // Timestamping
            if (::setsockopt(s, SOL_SOCKET, SO_TIMESTAMP, &on, sizeof(on)) < 0)
            {
                return -1;
            }
            // Socket loopback
            if (::setsockopt(s, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &on, sizeof(on)) < 0)
            {
                return -1;
            }
            // Non-blocking
            if (::fcntl(s, F_SETFL, O_NONBLOCK) < 0)
            {
                return -1;
            }
        }

        // Validate the resulting socket
        {
            int socket_error = 0;
            ::socklen_t errlen = sizeof(socket_error);
            (void)::getsockopt(s, SOL_SOCKET, SO_ERROR, reinterpret_cast<void*>(&socket_error), &errlen);
            if (socket_error != 0)
            {
                errno = socket_error;
                return -1;
            }
        }

        raii_closer.disarm();
        return s;
    }
};

/**
 * Multiplexing container for multiple SocketCAN sockets.
 * Uses ppoll() for multiplexing.
 *
 * When an interface becomes down/disconnected while the node is running,
 * the driver will silently exclude it from the IO loop and continue to run on the remaining interfaces.
 * When all interfaces become down/disconnected, the driver will throw @ref AllIfacesDownException
 * from @ref SocketCanDriver::select().
 * Whether a certain interface is down can be checked with @ref SocketCanDriver::isIfaceDown().
 */
class SocketCanDriver : public uavcan::ICanDriver
{
    class IfaceWrapper : public SocketCanIface
    {
        bool down_ = false;

    public:
        IfaceWrapper(const SystemClock& clock, int fd) : SocketCanIface(clock, fd) { }

        void updateDownStatusFromPollResult(const ::pollfd& pfd)
        {
            assert(pfd.fd == this->getFileDescriptor());
            if (!down_ && (pfd.revents & POLLERR))
            {
                int error = 0;
                ::socklen_t errlen = sizeof(error);
                (void)::getsockopt(pfd.fd, SOL_SOCKET, SO_ERROR, reinterpret_cast<void*>(&error), &errlen);

                down_ = error == ENETDOWN || error == ENODEV;

                UAVCAN_TRACE("SocketCAN", "Iface %d is dead; error %d", this->getFileDescriptor(), error);
            }
        }

        bool isDown() const { return down_; }
    };

    const SystemClock& clock_;
    std::vector<std::unique_ptr<IfaceWrapper>> ifaces_;

public:
    /**
     * Reference to the clock object shall remain valid.
     */
    explicit SocketCanDriver(const SystemClock& clock)
        : clock_(clock)
    {
        ifaces_.reserve(uavcan::MaxCanIfaces);
    }

    /**
     * This function may return before deadline expiration even if no requested IO operations become possible.
     * This behavior makes implementation way simpler, and it is OK since libuavcan can properly handle such
     * early returns.
     * Also it can return more events than were originally requested by uavcan, which is also acceptable.
     */
    std::int16_t select(uavcan::CanSelectMasks& inout_masks,
                        const uavcan::CanFrame* (&)[uavcan::MaxCanIfaces],
                        uavcan::MonotonicTime blocking_deadline) override
    {
        // Detecting whether we need to block at all
        bool need_block = (inout_masks.write == 0);    // Write queue is infinite
        for (unsigned i = 0; need_block && (i < ifaces_.size()); i++)
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
            ::pollfd pollfds[uavcan::MaxCanIfaces] = {};
            unsigned num_pollfds = 0;
            IfaceWrapper* pollfd_index_to_iface[uavcan::MaxCanIfaces] = { };

            for (unsigned i = 0; i < ifaces_.size(); i++)
            {
                if (!ifaces_[i]->isDown())
                {
                    pollfds[num_pollfds].fd = ifaces_[i]->getFileDescriptor();
                    pollfds[num_pollfds].events = POLLIN;
                    if (ifaces_[i]->hasReadyTx() || (inout_masks.write & (1U << i)))
                    {
                        pollfds[num_pollfds].events |= POLLOUT;
                    }
                    pollfd_index_to_iface[num_pollfds] = ifaces_[i].get();
                    num_pollfds++;
                }
            }

            // This is where we abort when the last iface goes down
            if (num_pollfds == 0)
            {
                throw AllIfacesDownException();
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
            const int res = ::ppoll(pollfds, num_pollfds, &ts, nullptr);
            if (res < 0)
            {
                return res;
            }

            // Handling poll output
            for (unsigned i = 0; i < num_pollfds; i++)
            {
                pollfd_index_to_iface[i]->updateDownStatusFromPollResult(pollfds[i]);

                const bool poll_read  = pollfds[i].revents & POLLIN;
                const bool poll_write = pollfds[i].revents & POLLOUT;
                pollfd_index_to_iface[i]->poll(poll_read, poll_write);
            }
        }

        // Writing the output masks
        inout_masks = uavcan::CanSelectMasks();
        for (unsigned i = 0; i < ifaces_.size(); i++)
        {
            if (!ifaces_[i]->isDown())
            {
                inout_masks.write |= std::uint8_t(1U << i);     // Always ready to write if not down
            }
            if (ifaces_[i]->hasReadyRx())
            {
                inout_masks.read |= std::uint8_t(1U << i);      // Readability depends only on RX buf, even if down
            }
        }

        // Return value is irrelevant as long as it's non-negative
        return ifaces_.size();
    }

    SocketCanIface* getIface(std::uint8_t iface_index) override
    {
        return (iface_index >= ifaces_.size()) ? nullptr : ifaces_[iface_index].get();
    }

    std::uint8_t getNumIfaces() const override { return ifaces_.size(); }

    /**
     * Adds one iface by name. Will fail if there are @ref MaxIfaces ifaces registered already.
     * @param iface_name E.g. "can0", "vcan1"
     * @return Negative on error, interface index on success.
     * @throws uavcan_linux::Exception.
     */
    int addIface(const std::string& iface_name)
    {
        if (ifaces_.size() >= uavcan::MaxCanIfaces)
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
            ifaces_.emplace_back(new IfaceWrapper(clock_, fd));
        }
        catch (...)
        {
            (void)::close(fd);
            throw;
        }

        UAVCAN_TRACE("SocketCAN", "New iface '%s' fd %d", iface_name.c_str(), fd);

        return ifaces_.size() - 1;
    }

    /**
     * Returns false if the specified interface is functioning, true if it became unavailable.
     */
    bool isIfaceDown(std::uint8_t iface_index) const
    {
        return ifaces_.at(iface_index)->isDown();
    }
};

}
