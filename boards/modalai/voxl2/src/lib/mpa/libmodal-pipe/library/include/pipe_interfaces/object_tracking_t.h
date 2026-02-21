#ifndef OBJECT_TRACKING_T_H
#define OBJECT_TRACKING_T_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////
// Object Tracking (OT)
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief      Common interface for object tracking (OT)
 *
 *             Applications that track objects or regioins of interest in video
 *             use this interface to supply tracking information to other system
 *             components.
 */


/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream.
 *
 */
#include "magic_number.h"
#ifndef OT_MAGIC_NUMBER
#error "OT_MAGIC_NUMBER not defined!"
#endif

/**
 * This is the data structure that object trackers should make available to clients
 * on the data pipe.
 *
 */
typedef struct object_tracking_data_t
{
    uint32_t magic_number;         ///< Unique 32-bit number used to signal the beginning of a OT packet while parsing a data stream.
    float obj_loc_yaw_rad;      ///< positive value indicates object is on the right side of the frame
    float obj_loc_pitch_rad;    ///< positive value indicates object is in the upper half of the frame
    int32_t n_sequence;         ///< number of sequential detections of this object, set to -1 if nothing is detected, reset to 1 on first detection of new object
    int64_t timestamp_ns;       ///< timestamp of the detection, usually halway through the camera exposure
    char camera_frame[32];      ///< name of camera frame where this object was detected
    int32_t reserved1;
    int32_t reserved2;
    int32_t reserved3;
    int32_t reserved4;
} __attribute__((packed)) object_tracking_data_t;

/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync.
 *
 * Note this is NOT the size of the pipe which can hold much more. This is just
 * the read buffer size allocated on the heap into which data from the pipe is
 * read.
 */
#define OT_RECOMMENDED_READ_BUF_SIZE (sizeof(object_tracking_data_t) * 16)

/**
 * We recommend VIO servers use a 64k pipe size. Clients can increase this
 * buffer if they wish. 64K is also the Linux Kernel default pipe size.
 */
#define OT_RECOMMENDED_PIPE_SIZE (64 * 1024)

/**
 * @brief      Use this to simultaneously validate that the bytes from a pipe
 *             contains valid data, find the number of valid packets
 *             contained in a single read from the pipe, and cast the raw data
 *             buffer as a object_tracking_data_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate a object_tracking_data_t array separate from the pipe read buffer.
 *             The data can be read straight out of the pipe read buffer, much
 *             like reading data directly out of a mavlink_message_t message.
 *
 *             However, this does mean the user should finish processing this
 *             data before returning the pipe data callback which triggers a new
 *             read() from the pipe.
 *
 * @param[in]  data       pointer to pipe read data buffer
 * @param[in]  bytes      number of bytes read into that buffer
 * @param[out] n_packets  number of valid packets received
 *
 * @return     Returns the same data pointer provided by the first argument, but
 *             cast to an object_tracking_data_t* struct for convenience. If there was an
 *             error then NULL is returned and n_packets is set to 0
 */
object_tracking_data_t *pipe_validate_ot_data_t(char *data, int bytes, int *n_packets);

#ifdef __cplusplus
}
#endif

#endif