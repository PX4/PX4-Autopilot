/*
 * uorb_converter.c
 *
 *  Created on: Apr 10, 2020
 *      Author: hovergames
 */

#include <string.h>

#include "uorb_converter.h"


/****************************************************************************
 * Private Data
 ****************************************************************************/
int uorb_sub_fd;
struct vehicle_gps_position_s att;
orb_advert_t att_pub;
px4_pollfd_struct_t fds[1];
int error_counter;
CanardInstance *canard_ins;
int my_message_transfer_id;


void uorbConverterInit(CanardInstance *ins)
{
	canard_ins = ins;

	/* subscribe to the uORB topic */
	uorb_sub_fd = orb_subscribe(ORB_ID(vehicle_gps_position));

	orb_set_interval(uorb_sub_fd, 200);

	/* advertise uORB topic */
	memset(&att, 0, sizeof(att));
	att_pub = orb_advertise(ORB_ID(vehicle_gps_position), &att);

	/* one could wait for multiple topics with this technique, just using one here */
	fds[0].fd = uorb_sub_fd;
	fds[0].events = POLLIN;

	error_counter = 0;
	my_message_transfer_id = 0;
}

void uorbProcessSub(int timeout_msec)
{
	/* wait for subscriber update of 1 file descriptor for timeout_msec ms */
	int poll_ret = px4_poll(fds, 1, timeout_msec);

	/* handle the poll result */
	if (poll_ret == 0) {
		/* this means none of our providers is giving us data */
	} else if (poll_ret < 0) {
		/* this is seriously bad - should be an emergency */
		if (error_counter < 10 || error_counter % 50 == 0) {
			/* use a counter to prevent flooding (and slowing us down) */
			PX4_ERR("ERROR return value from poll(): %d", poll_ret);
		}

		error_counter++;

	} else {
		if (fds[0].revents & POLLIN) {
			/* obtained data for the first file descriptor */
			struct vehicle_gps_position_s raw;
			/* copy sensors raw data into local buffer */
			orb_copy(ORB_ID(vehicle_gps_position), uorb_sub_fd, &raw);

			CanardMicrosecond transmission_deadline = getMonotonicTimestampUSec() + 1000 * 10;

			const CanardTransfer transfer = {
				.timestamp_usec = transmission_deadline,      // Zero if transmission deadline is not limited.
				.priority       = CanardPriorityNominal,
				.transfer_kind  = CanardTransferKindMessage,
				.port_id        = PORT_ID,                       // This is the subject-ID.
				.remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
				.transfer_id    = my_message_transfer_id,
				.payload_size   = TOPIC_SIZE,
				.payload        = &raw,
			};

			/* TODO uORB parsing and DSDL conversion */

			++my_message_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
			int32_t result = canardTxPush(canard_ins, &transfer);

			if (result < 0) {
				// An error has occurred: either an argument is invalid or we've ran out of memory.
				// It is possible to statically prove that an out-of-memory will never occur for a given application if the
				// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
				PX4_ERR("canardTxPush error %d", result);
			}

			PX4_INFO("Recevied data from uORB topic");
		}
	}
}


void uorbProcessPub(CanardTransfer *pub_msg)
{

	if (pub_msg->port_id == PORT_ID) {
		PX4_INFO("Publishing UAVCAN data to uORB topic");

		/* TODO uORB parsing and DSDL conversion */
		memcpy(&att, pub_msg->payload, pub_msg->payload_size);

		orb_publish(ORB_ID(vehicle_gps_position), att_pub, &att);
	}

}
