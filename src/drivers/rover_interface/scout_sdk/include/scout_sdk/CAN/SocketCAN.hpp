#pragma once

#include <px4_platform_common/px4_config.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>

#include <sys/time.h>
#include <sys/socket.h>

#include <nuttx/can.h>

#include "can_frame.h"

namespace scoutsdk
{
/// RX Frame with its reception timestamp.
struct RxFrame {
	uint64_t timestamp_usec;

	/// The actual CAN frame data.
	CANFrame frame;
};

/// TX Frame with its transmission deadline.
struct TxFrame {
	uint64_t tx_deadline_usec;

	/// The actual CAN frame data.
	CANFrame frame;
};


class SocketCAN
{
public:
	/// Creates a SocketCAN socket for corresponding iface can_iface_name
	/// Also sets up the message structures required for socketcanTransmit & socketcanReceive
	/// can_fd determines to use CAN FD frame when is 1, and classical CAN frame when is 0
	/// The return value is 0 on success and -1 on error
	int Init(const char *const can_iface_name, const uint32_t can_bitrate);

	/// Close socket connection
	int Close()
	{
		return ::close(_fd);
	}

	/// Send a Frame to the SocketInstance socket
	/// This function is blocking
	/// The return value is number of bytes transferred, negative value on error.
	int16_t SendFrame(const TxFrame &txframe, int timeout_ms = 0);

	/// Receive a Frame from the SocketInstance socket
	/// This function is blocking
	/// The return value is number of bytes received, negative value on error.
	int16_t ReceiveFrame(RxFrame *rxf);

private:

	int               _fd{-1};
	bool              _can_fd{false};

	//// Send msg structure
	struct iovec       _send_iov {};
	struct canfd_frame _send_frame {};
	struct msghdr      _send_msg {};
	struct cmsghdr     *_send_cmsg {};
	struct timeval     *_send_tv {};  /* TX deadline timestamp */
	uint8_t            _send_control[sizeof(struct cmsghdr) + sizeof(struct timeval)] {};

	//// Receive msg structure
	struct iovec       _recv_iov {};
	struct canfd_frame _recv_frame {};
	struct msghdr      _recv_msg {};
	struct cmsghdr     *_recv_cmsg {};
	uint8_t            _recv_control[sizeof(struct cmsghdr) + sizeof(struct timeval)] {};
};
} // namespace scoutsdk
