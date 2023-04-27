#ifndef CAN_FRAME_H
#define CAN_FRAME_H

#include <stdint.h>
#include <unistd.h>

/// CAN data frame with support for 11-bit ID and extended 29-bit ID.
typedef struct {
	/// 11-bit or 29-bit extended ID. The bits above 29-th shall be zero.
	uint32_t can_id;

	/// The useful data in the frame. The length value is not to be confused with DLC!
	/// If the payload is empty (payload_size = 0), the payload pointer may be NULL.
	/// For RX frames: the library does not expect the lifetime of the pointee to extend beyond the point of return
	/// from the API function. That is, the pointee can be invalidated immediately after the frame has been processed.
	/// For TX frames: the frame and the payload are allocated within the same dynamic memory fragment, so their
	/// lifetimes are identical; when the frame is freed, the payload is invalidated.
	/// A more detailed overview of the dataflow and related resource management issues is provided in the API docs.
	size_t      payload_size;
	void *payload;
} CANFrame;

#endif /* CAN_FRAME_H */
