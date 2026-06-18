#ifndef POSE_VEL_6DOF2_T_H
#define POSE_VEL_6DOF2_T_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////
// 6DOF pose with velocity
////////////////////////////////////////////////////////////////////////////////

/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream.
 */
#include "magic_number.h"
#ifndef POSE_VEL_6DOF2_MAGIC_NUMBER
#error "POSE_VEL_6DOF2_MAGIC_NUMBER not defined!"
#endif

// length of parent and child frame strings, matched libvoxl-common-config
// extrinsic frame fields in vcc_extrinsic_t
#define PV6DOF2_STRLEN  64

/**
 * Position and velocity in 6DOF, this is basically a stripped down VIO packet.
 *
 * This is how voxl-vision-hub publishes the position and velocity of the drone
 * body in both local and fixed frame. Units are in m, m/s, and rad/s
 *
 * Will also be used for voxl-state-estimator in the future
 */
typedef struct pose_vel_6dof2_t
{
    uint32_t magic_number;          ///< Unique 32-bit number used to signal the beginning of a packet while parsing a data stream.
    int64_t timestamp_ns;           ///< Timestamp in clock_monotonic system time of the provided pose.
    char parent[PV6DOF2_STRLEN];    ///< Name of parent frame
    char child[PV6DOF2_STRLEN];     ///< Name of child frame
    float T_child_wrt_parent[3];    ///< Translation of the child with respect to parent frame in meters.
    float R_child_to_parent[3][3];  ///< Rotation matrix from child to parent frame.
    float v_child_wrt_parent[3];    ///< Velocity of the child with respect to the parent frame.
    float w_child_wrt_child[3];     ///< Angular velocity of the child frmae about its X Y and Z axes respectively. Essentially filtered gyro values with internal biases applied.
    int8_t quality;                 ///< -1 for invalid, good data ranges from 0 to 100 based on sensor or estimator confidence
    int8_t reserved1;
    int8_t reserved2;
    int8_t reserved3;
} __attribute__((packed)) pose_vel_6dof2_t;

/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync.
 *
 * Note this is NOT the size of the pipe which can hold much more. This is just
 * the read buffer size allocated on the heap into which data from the pipe is
 * read.
 */
#define POSE_6DOF2_RECOMMENDED_READ_BUF_SIZE (sizeof(pose_vel_6dof2_t) * 32)

/**
 * We recommend pose servers use a 64k pipe size.
 * 64K is also the Linux Kernel default pipe size.
 */
#define POSE_6DOF2_RECOMMENDED_PIPE_SIZE (64 * 1024)

/**
 * @brief      Use this to simultaneously validate that the bytes from a pipe
 *             contains valid data, find the number of valid packets
 *             contained in a single read from the pipe, and cast the raw data
 *             buffer as a pose_vel_6dof_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate a pose_vel_6dof_t array separate from the pipe read buffer.
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
 *             cast to an pose_vel_6dof_t* struct for convenience. If there was an
 *             error then NULL is returned and n_packets is set to 0
 */
pose_vel_6dof2_t *pipe_validate_pose_vel_6dof2_t(char *data, int bytes, int *n_packets);

#ifdef __cplusplus
}
#endif

#endif
