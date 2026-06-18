#ifndef POSE_4DOF_T_H
#define POSE_4DOF_T_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////
// 4DOF pose
////////////////////////////////////////////////////////////////////////////////

/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream.
 */
#include "magic_number.h"
#ifndef POSE_4DOF_MAGIC_NUMBER
#error "POSE_4DOF_MAGIC_NUMBER not defined!"
#endif

    /**
     * 4DOF pose (position and yaw)
     *
     * This is used to describe the position and orientation of the drone body in
     * fixed frame where roll and pitch are not needed for the fixed-frame pipe
     * input to voxl-vision-px4.
     *
     * Data is sent in with the position as the position of the center of mass of
     * the drone's body with respect to the fixed coordinate frame. The fixed
     * coordinate frame does not have to be north-aligned but should keep with the
     * NED convention of x pointing forward, Y to the right, and Z down.
     *
     * Yaw follows the right hand rule, e.g. positive yaw indicates rotation of the
     * body to the right about the Z axis. Yaw should be in +-PI
     *
     * This packet is 44 bytes long
     */
    typedef struct pose_4dof_t
    {
        uint32_t magic_number; ///< Unique 32-bit number used to signal the beginning of a packet while parsing a data stream.
        int64_t timestamp_ns;  ///< timestamp in apps-proc clock-monotonic
        double p[3];           ///< meters
        double yaw;            ///< radians, between +- PI
    } __attribute__((packed)) pose_4dof_t;

/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync. Here we use 23 packets which is
 * perhaps more than necessary but only takes a little under 1kB of memory
 * which is minimal.
 *
 * Note this is NOT the size of the pipe which can hold much more. This is just
 * the read buffer size allocated on the heap into which data from the pipe is
 * read.
 */
#define POSE_4DOF_RECOMMENDED_READ_BUF_SIZE (sizeof(pose_4dof_t) * 23)

/**
 * We recommend pose servers use a 64k pipe size. This means every client would
 * get their own buffer of 49 seconds of data at 30hz. Clients can
 * increase this buffer if they wish.
 * 64K is also the Linux Kernel default pipe size.
 */
#define POSE_4DOF_RECOMMENDED_PIPE_SIZE (64 * 1024)

    /**
     * @brief      Use this to simultaneously validate that the bytes from a pipe
     *             contains valid data, find the number of valid packets
     *             contained in a single read from the pipe, and cast the raw data
     *             buffer as a pose_4dof_t* for easy access.
     *
     *             This does NOT copy any data and the user does not need to
     *             allocate a pose_4dof_t array separate from the pipe read buffer.
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
     *             cast to an pose_4dof_t* struct for convenience. If there was an
     *             error then NULL is returned and n_packets is set to 0
     */
    pose_4dof_t *pipe_validate_pose_4dof_t(char *data, int bytes, int *n_packets);

#ifdef __cplusplus
}
#endif

#endif