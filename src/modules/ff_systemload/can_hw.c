#include "can_hw.h"

#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <string.h>

#include <board_config.h>
#include <nuttx/can/can.h>

#ifdef CONFIG_ARCH_CHIP_STM32
#include "stm32.h"
#endif
#include "stm32_can.h"

#include <drivers/drv_hrt.h>

//#define CAN_HW_VERBOSE_INFO

int hw_CAN_receive_nonblocking(can_handle_s *handle);
int hw_CAN_transmit_nonblocking(can_handle_s *handle);
void printMessage(can_message_s *message);

can_hw_returncode_e hw_CAN_transmit(can_handle_s *handle, uint32_t timeout)
{
#ifdef CAN_HW_VERBOSE_INFO
	PX4_INFO("Transmit:");

	printMessage(&handle->pTxMsg);
#endif

	if (hw_CAN_transmit_nonblocking(handle) == 0) {
		return CAN_HW_OK;

	} else {
		return CAN_HW_FAIL;
	}

	//don't do blocking timeout for now.
}

can_hw_returncode_e hw_CAN_receive(can_handle_s *handle, uint32_t timeout)
{
#ifdef CAN_HW_VERBOSE_INFO
	PX4_INFO("Receive");
#endif

	uint32_t timestamp = (uint32_t)hrt_absolute_time(); //in microseconds
	uint32_t elapsed_time;

	while (((elapsed_time = (uint32_t)hrt_absolute_time()) - timestamp) < (timeout * 1000)) {
		if (hw_CAN_receive_nonblocking(handle) == 0) { //received message successfully
#ifdef CAN_HW_VERBOSE_INFO
			PX4_INFO("Success");
#endif
			return CAN_HW_OK;
		}

#ifdef CAN_HW_VERBOSE_INFO
		PX4_INFO("time: %d", elapsed_time);
#endif
	}

#ifdef CAN_HW_VERBOSE_INFO
	PX4_INFO("Timed out");
#endif
	memset((void *)&handle->pRxMsg, 0, sizeof(can_message_s));

	return CAN_HW_FAIL;
}

can_hw_returncode_e hw_CAN_init(can_handle_s *handle)
{
	PX4_INFO("initializing CAN");

	//struct can_dev_s *can_dev_p = NULL;
	//int canPort = 1;

	//can_dev_p = stm32_caninitialize(canPort);

	//if (can_dev_p == NULL) {
	//	PX4_ERR("ERROR:  Failed to initialize CAN interface. ");
	//	return CAN_HW_FAIL;
	//}

	//int can_instance_number = can_register(handle->instance, can_dev_p);

	//if (can_instance_number < 0) {
	//	PX4_ERR("ERROR: can_register failed: %d", can_instance_number);
	//	return CAN_HW_FAIL;
	//}

	int canFD = open(handle->instance, O_RDWR | O_NONBLOCK);
	handle->descriptor = canFD;

	if (canFD < 0) {
		PX4_INFO("Failed to open CAN device.");
		return CAN_HW_FAIL;
	}

	return CAN_HW_OK;
}

can_hw_returncode_e hw_CAN_deinit(can_handle_s *handle)
{
#ifdef CAN_HW_VERBOSE_INFO
	PX4_INFO("Closing descriptor %d", handle->descriptor);
#endif

	if (handle->descriptor < 0) {
		close(handle->descriptor);
		handle->descriptor = -1;
	}

	return CAN_HW_OK;
}

void printMessage(can_message_s *message)
{
	PX4_INFO("ID: %lx DLC: %ld RTR: %ld", message->StdId, message->DLC, message->RTR);
	// for (int j = 0; j<8; j++){
	// 	PX4_INFO("   Data[%d]: %x -- OK", j, message->Data[j]);
	// }
}

int hw_CAN_transmit_nonblocking(can_handle_s *handle)
{
	size_t msgsize;
	ssize_t bytes_written;

	struct can_msg_s nuttx_msg;
	nuttx_msg.cm_hdr.ch_dlc = handle->pTxMsg.DLC;
	nuttx_msg.cm_hdr.ch_rtr = handle->pTxMsg.RTR;
	nuttx_msg.cm_hdr.ch_id = handle->pTxMsg.StdId;

	for (uint8_t i = 0; i < handle->pTxMsg.DLC; i++) {
		nuttx_msg.cm_data[i] = handle->pTxMsg.Data[i];
	}

	msgsize = CAN_MSGLEN(nuttx_msg.cm_hdr.ch_dlc);

	bytes_written = write(handle->descriptor, &nuttx_msg, msgsize);

	if ((size_t)bytes_written != msgsize || bytes_written < 0) {
#ifdef CAN_HW_VERBOSE_INFO
		PX4_INFO("CAN Write Error: write(%d) returned %d", msgsize, bytes_written);
#endif
		return -1;

	} else {
#ifdef CAN_HW_VERBOSE_INFO
		PX4_INFO("CAN Write Success: write(%d) returned %d", msgsize, bytes_written);
#endif
		return 0;
	}
}

int hw_CAN_receive_nonblocking(can_handle_s *handle)
{
	size_t msgsize;
	ssize_t bytes_read;

	struct can_msg_s nuttx_msg; //nuttx message struct

	// Gets the maximum possible size of a CAN message - we will try to read up to this amount
	msgsize = sizeof(struct can_msg_s);

	bytes_read = read(handle->descriptor, &nuttx_msg, msgsize);

	if ((size_t)bytes_read < CAN_MSGLEN(0) || (size_t)bytes_read > msgsize || bytes_read < 0) {
#ifdef CAN_HW_VERBOSE_INFO
		PX4_INFO("ERROR: read(%d) returned %d ", msgsize, bytes_read);
#endif
		return -1;

	} else {
#ifdef CAN_HW_VERBOSE_INFO
		PX4_INFO("Receive:   ID: %4d DLC: %d -- OK", nuttx_msg.cm_hdr.ch_id, nuttx_msg.cm_hdr.ch_dlc);
		// for (int j = 0; j<nuttx_msg.cm_hdr.ch_dlc; j++){
		// 	PX4_INFO("     Data[%d]: %4d -- OK", j, nuttx_msg.cm_data[j]);
		// }
#endif

		handle->pRxMsg.StdId = nuttx_msg.cm_hdr.ch_id;
		handle->pRxMsg.DLC = nuttx_msg.cm_hdr.ch_dlc;
		handle->pRxMsg.RTR = nuttx_msg.cm_hdr.ch_rtr;

		for (uint8_t i = 0; i < nuttx_msg.cm_hdr.ch_dlc; i++) {
			handle->pRxMsg.Data[i] = nuttx_msg.cm_data[i];
		}

		return 0;
	}
}
