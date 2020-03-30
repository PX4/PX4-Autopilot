/****************************************************************************
 * examples/canard/canard_main.c
 *
 *   Copyright (C) 2016 ETH Zuerich. All rights reserved.
 *   Author: Matthias Renner <rennerm@ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <canard.h>
#include <canard_dsdl.h>

#include <sched.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <net/if.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <poll.h>

#include <nuttx/can.h>
#include <netpacket/can.h>

#include "o1heap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Application constants */

#define APP_VERSION_MAJOR                        1
#define APP_VERSION_MINOR                        0
#define APP_NODE_NAME                            CONFIG_EXAMPLES_LIBCANARDV1_APP_NODE_NAME

#define UAVCAN_NODE_HEALTH_OK                    0
#define UAVCAN_NODE_HEALTH_WARNING               1
#define UAVCAN_NODE_HEALTH_ERROR                 2
#define UAVCAN_NODE_HEALTH_CRITICAL              3

#define UAVCAN_NODE_MODE_OPERATIONAL             0
#define UAVCAN_NODE_MODE_INITIALIZATION          1

#define UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE   ((3015 + 7) / 8)
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE 0xee468a8121c46a9e
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_ID        1

#define UNIQUE_ID_LENGTH_BYTES                   16

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Arena for memory allocation, used by the library */

static uint8_t canard_memory_pool[CONFIG_EXAMPLES_LIBCANARDV1_NODE_MEM_POOL_SIZE];

static uint8_t unique_id[UNIQUE_ID_LENGTH_BYTES] = {
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01
};

/* Node status variables */

static uint8_t node_health = UAVCAN_NODE_HEALTH_OK;
static uint8_t node_mode = UAVCAN_NODE_MODE_INITIALIZATION;
static bool g_canard_daemon_started;

static uint8_t my_message_transfer_id;  // Must be static or heap-allocated to retain state between calls.

struct pollfd fd;
int s; /* can raw socket */

/* rcv msg */
struct iovec recv_iov;
struct msghdr recv_msg;
struct canfd_frame recv_frame;
char ctrlmsg[sizeof(struct cmsghdr) + sizeof(struct timeval)];

#define O1_HEAP_SIZE 4096

O1HeapInstance *my_allocator;
static uint8_t uavcan_heap[O1_HEAP_SIZE]
__attribute__((aligned(O1HEAP_ALIGNMENT)));

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: memAllocate
 *
 * Description:
 *
 ****************************************************************************/
static void *memAllocate(CanardInstance *const ins, const size_t amount)
{
	(void) ins;
	return o1heapAllocate(my_allocator, amount);
}

/****************************************************************************
 * Name: memFree
 *
 * Description:
 *
 ****************************************************************************/

static void memFree(CanardInstance *const ins, void *const pointer)
{
	(void) ins;
	o1heapFree(my_allocator, pointer);
}

/****************************************************************************
 * Name: getMonotonicTimestampUSec
 *
 * Description:
 *
 ****************************************************************************/
uint64_t getMonotonicTimestampUSec(void)
{
	struct timespec ts;

	memset(&ts, 0, sizeof(ts));

	if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
		abort();
	}

	return ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}

/****************************************************************************
 * Name: process1HzTasks
 *
 * Description:
 *   This function is called at 1 Hz rate from the main loop.
 *
 ****************************************************************************/

void process1HzTasks(CanardInstance *ins, uint64_t timestamp_usec)
{

	CanardMicrosecond transmission_deadline = getMonotonicTimestampUSec() + 1000 * 10;

	const CanardTransfer transfer = {
		.timestamp_usec = transmission_deadline,      // Zero if transmission deadline is not limited.
		.priority       = CanardPriorityNominal,
		.transfer_kind  = CanardTransferKindMessage,
		.port_id        = 1234,                       // This is the subject-ID.
		.remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
		.transfer_id    = my_message_transfer_id,
		.payload_size   = 47,
		.payload        = "\x2D\x00" "Sancho, it strikes me thou art in great fear.",
	};

	++my_message_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
	int32_t result = canardTxPush(ins, &transfer);

	if (result < 0) {
		// An error has occurred: either an argument is invalid or we've ran out of memory.
		// It is possible to statically prove that an out-of-memory will never occur for a given application if the
		// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
		fprintf(stderr, "Transmit error %d\n", result);
	}
}

static uint32_t pleaseTransmit(const CanardFrame *txf)
{
	struct iovec iov;
	struct canfd_frame sockcan_frame;

	sockcan_frame.can_id = txf->extended_can_id;
	sockcan_frame.can_id |= CAN_EFF_FLAG;
	sockcan_frame.len = txf->payload_size;
	memcpy(&sockcan_frame.data, txf->payload, txf->payload_size);

	iov.iov_base = &sockcan_frame;
	iov.iov_len  = sizeof(sockcan_frame);

	uint8_t control[sizeof(struct cmsghdr) + sizeof(struct timeval)];
	memset(&control, 0x00, sizeof(control));

	struct msghdr msg;
	msg.msg_iov    = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = &control;
	msg.msg_controllen = sizeof(control);

	struct cmsghdr *cmsg = CMSG_FIRSTHDR(&msg);
	cmsg->cmsg_level = SOL_CAN_RAW;
	cmsg->cmsg_type = CAN_RAW_TX_DEADLINE;
	cmsg->cmsg_len = sizeof(struct timeval);
	struct timeval *tv = (struct timeval *)CMSG_DATA(cmsg);
	tv->tv_usec = txf->timestamp_usec % 1000000ULL;
	tv->tv_sec = (txf->timestamp_usec - tv->tv_usec) / 1000000ULL;

	const int res = sendmsg(s, &msg, 0);

	if (res <= 0) {
		if (errno == ENOBUFS || errno == EAGAIN) {  // Writing is not possible atm, not an error
			return 0;
		}

		return res;
	}

	if (res != sizeof(sockcan_frame)) {
		return -1;
	}

	return 1;
}

static void processReceivedTransfer(CanardTransfer *receive)
{
	printf("Received transfer remote_node_id %d transfer_id: %d payload size: %d\n",
	       receive->remote_node_id, receive->transfer_id, receive->payload_size);

}

/****************************************************************************
 * Name: processTxRxOnce
 *
 * Description:
 *   Transmits all frames from the TX queue, receives up to one frame.
 *
 ****************************************************************************/

void processTxRxOnce(CanardInstance *ins, int timeout_msec)
{
	/* Transmitting */


	for (const CanardFrame *txf = NULL; (txf = canardTxPeek(ins)) != NULL;) { // Look at the top of the TX queue.
		if (txf->timestamp_usec > getMonotonicTimestampUSec()) { // Check if the frame has timed out.
			if (pleaseTransmit(txf) == 0) {           // Send the frame. Redundant interfaces may be used here.
				break;                             // If the driver is busy, break and retry later.
			}
		}

		canardTxPop(ins);                         // Remove the frame from the queue after it's transmitted.
		ins->memory_free(ins, (CanardFrame *)txf); // Deallocate the dynamic memory afterwards.
	}


	/* Poll receive */
	if (poll(&fd, 1, timeout_msec) <= 0) {
		return;
	}

	/* Receiving */

	int32_t result = recvmsg(s, &recv_msg, 0);

	if (result < 0) {
		if (errno == ENETDOWN) {
			fprintf(stderr, "%s: interface down\n", CONFIG_EXAMPLES_LIBCANARDV1_DEV);
		}

		return;
	}

	CanardFrame received_frame;

	received_frame.extended_can_id = recv_frame.can_id & CAN_EFF_MASK;
	received_frame.payload_size = recv_frame.len;
	memcpy(received_frame.payload, recv_frame.data, recv_frame.len);

	struct cmsghdr *cmsg = CMSG_FIRSTHDR(&recv_msg);

	if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMP) {
		struct timeval tv;
		memcpy(&tv, CMSG_DATA(cmsg), sizeof(tv));
		received_frame.timestamp_usec = tv.tv_sec * 1000000ULL + tv.tv_usec;
	}

	CanardTransfer receive;
	result = canardRxAccept(ins,
				&received_frame, // The CAN frame received from the bus.
				0,               // If the transport is not redundant, use 0.
				&receive);

	if (result < 0) {
		// An error has occurred: either an argument is invalid or we've ran out of memory.
		// It is possible to statically prove that an out-of-memory will never occur for a given application if
		// the heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
		// Reception of an invalid frame is NOT an error.
		fprintf(stderr, "Receive error %d\n", result);

	} else if (result == 1) {
		processReceivedTransfer(&receive);  // A transfer has been received, process it.
		ins->memory_free(ins, (void *)receive.payload); // Deallocate the dynamic memory afterwards.

	} else {
		printf("RX canard %d\r\n", result);
		// Nothing to do.
		// The received frame is either invalid or it's a non-last frame of a multi-frame transfer.
		// Reception of an invalid frame is NOT reported as an error because it is not an error.
	}


}


static int create_can_socket()
{
	int s; /* can raw socket */
	struct sockaddr_can addr;
	struct canfd_frame frame;
	struct ifreq ifr;

	/* open socket */
	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("socket");
		return -1;
	}

	strncpy(ifr.ifr_name, CONFIG_EXAMPLES_LIBCANARDV1_DEV, IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);

	if (!ifr.ifr_ifindex) {
		perror("if_nametoindex");
		return -1;
	}

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	const int on = 1;
	/* RX Timestamping */

	if (setsockopt(s, SOL_SOCKET, SO_TIMESTAMP, &on, sizeof(on)) < 0) {
		return -1;
	}

	/* NuttX Feature: Enable TX deadline when sending CAN frames
	 * When a deadline occurs the driver will remove the CAN frame
	 */

	if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_TX_DEADLINE, &on, sizeof(on)) < 0) {
		return -1;
	}

	if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof(on)) < 0) {
		return -1;
	}

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return -1;
	}

	/* setup poll fd */
	fd.fd = s;
	fd.events = POLLIN;

	/* setup recv msg structure */
	recv_iov.iov_base = &recv_frame;
	recv_msg.msg_iov = &recv_iov;
	recv_msg.msg_iovlen = 1;
	recv_msg.msg_control = &ctrlmsg;
	recv_iov.iov_len = sizeof(recv_frame);
	recv_msg.msg_controllen = sizeof(ctrlmsg);
	recv_msg.msg_flags = 0;

	return s;
}

/****************************************************************************
 * Name: canard_daemon
 *
 * Description:
 *
 ****************************************************************************/

static int canard_daemon(int argc, char *argv[])
{

	int errval = 0;
	int ret;

	my_allocator = o1heapInit(&uavcan_heap, O1_HEAP_SIZE, NULL, NULL);

	if (my_allocator == NULL) {
		printf("o1heapInit failed with size %d\n", O1_HEAP_SIZE);
		return;
	}

	CanardInstance ins = canardInit(&memAllocate, &memFree);
	ins.mtu_bytes = CANARD_MTU_CAN_FD;  // Defaults to 64 (CAN FD);
	ins.node_id   = 2;

	/* Open the CAN device for reading */
	s = create_can_socket();

	if (s < 0) {
		printf("canard_daemon: ERROR: open %s failed: %d\n",
		       CONFIG_EXAMPLES_LIBCANARDV1_DEV, errno);
		errval = 2;
		goto errout_with_dev;
	}


	printf("canard_daemon: canard initialized\n");
	printf("start node (ID: %d Name: %s)\n", ins.node_id,
	       APP_NODE_NAME);

	CanardRxSubscription heatbeat_subscription;
	(void) canardRxSubscribe(&ins,
				 CanardTransferKindMessage,  // Indicate that we want service responses.
				 32085,                         // The Service-ID to subscribe to.
				 1024,                        // The maximum payload size (max DSDL object size).
				 CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
				 &heatbeat_subscription);

	CanardRxSubscription my_subscription;
	(void) canardRxSubscribe(&ins,
				 CanardTransferKindResponse,  // Indicate that we want service responses.
				 123,                         // The Service-ID to subscribe to.
				 1024,                        // The maximum payload size (max DSDL object size).
				 CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
				 &my_subscription);


	g_canard_daemon_started = true;
	uint64_t next_1hz_service_at = getMonotonicTimestampUSec();

	for (;;) {
		processTxRxOnce(&ins, 10);

		const uint64_t ts = getMonotonicTimestampUSec();

		if (ts >= next_1hz_service_at) {
			next_1hz_service_at += 1000000;
			process1HzTasks(&ins, ts);
		}
	}

errout_with_dev:

	g_canard_daemon_started = false;
	printf("canard_daemon: Terminating!\n");
	fflush(stdout);
	return errval;
}


/****************************************************************************
 * Name: canard_main
 *
 * Description:
 *
 ****************************************************************************/

int uavcannode_v1_main(int argc, FAR char *argv[])
{
	int ret;

	printf("canard_main: Starting canard_daemon\n");

	if (g_canard_daemon_started) {
		printf("canard_main: receive and send task already running\n");
		return EXIT_SUCCESS;
	}

	ret = task_create("canard_daemon", CONFIG_EXAMPLES_LIBCANARDV1_DAEMON_PRIORITY,
			  CONFIG_EXAMPLES_LIBCANARDV1_NODE_MEM_POOL_SIZE, canard_daemon,
			  NULL);

	if (ret < 0) {
		int errcode = errno;
		printf("canard_main: ERROR: Failed to start canard_daemon: %d\n",
		       errcode);
		return EXIT_FAILURE;
	}

	printf("canard_main: canard_daemon started\n");
	return EXIT_SUCCESS;
}
