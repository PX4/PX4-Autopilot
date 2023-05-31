#ifndef __FF_SL_CAN_HW_H
#define __FF_SL_CAN_HW_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdbool.h>
#include <stdint.h>

typedef enum {
	CAN_HW_OK = 0,
	CAN_HW_FAIL = 1,
} can_hw_returncode_e;

typedef struct {
	uint32_t StdId;
	uint32_t ExtId;
	uint32_t IDE;
	uint32_t RTR;
	uint32_t DLC;
	uint8_t Data[8];
} can_message_s;

typedef struct {
	can_message_s pTxMsg; /*!< Pointer to transmit structure  */
	can_message_s pRxMsg; /*!< Pointer to reception structure */
	const char *instance; //file path
	int descriptor;
} can_handle_s;

#define CAN_ID_STD ((uint32_t)0x00000000U)  /*!< Standard Id */
#define CAN_ID_EXT ((uint32_t)0x00000004U)  /*!< Extended Id */

#define CAN_RTR_DATA ((uint32_t)0x00000000U)  /*!< Data frame */
#define CAN_RTR_REMOTE ((uint32_t)0x00000001U)  /*!< Remote frame */

can_hw_returncode_e hw_CAN_transmit(can_handle_s *handle, uint32_t timeout);
can_hw_returncode_e hw_CAN_receive(can_handle_s *handle, uint32_t timeout);
can_hw_returncode_e hw_CAN_init(can_handle_s *handle);
can_hw_returncode_e hw_CAN_deinit(can_handle_s *handle);

#ifdef __cplusplus
}
#endif

#endif
