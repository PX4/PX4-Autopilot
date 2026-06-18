#ifndef TAG_DETECTION_T_H
#define TAG_DETECTION_T_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <modal_pipe_defines.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>

#ifdef CTYPESGEN
#pragma pack(1)
#endif

////////////////////////////////////////////////////////////////////////////////
// Apriltag/aruco Detection
////////////////////////////////////////////////////////////////////////////////

/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream.
 */
#include "magic_number.h"
#ifndef TAG_DETECTION_MAGIC_NUMBER
#error "TAG_DETECTION_MAGIC_NUMBER not defined!"
#endif

/**
 * A tag can be flagged by the user as fixed, static, or dynamic.
 *
 * "fixed": The tag is at a known location in space as described by the
 * T_tag_wrt_fixed vector and R_tag_to_fixed rotation matrix. These fixed tags
 * are used by voxl-vision-px4 for apriltag relocalization.
 *
 * "static": A static tag can be trusted to be static (not moving) but does not
 * have a known location. For example, a landing pad.
 *
 * "dynamic": A dynamic tag can be expected to be in motion such as an object to
 * be tracked or followed dynamically.
 *
 * If a tag is detected that has not been listed by the user as a known tag
 * in /etc/modalai/tag_locations.conf then it will be listed as unknown.
 */
#ifndef CTYPESGEN
#define TAG_LOCATION_TYPE_STRINGS {"unknown", "fixed", "static", "dynamic"}
#endif
#define N_TAG_LOCATION_TYPES 4

#define TAG_LOCATION_UNKNOWN 0
#define TAG_LOCATION_FIXED 1
#define TAG_LOCATION_STATIC 2
#define TAG_LOCATION_DYNAMIC 3

// max name length of a tag name
#define TAG_NAME_LEN 64

    /**
     * describes an apriltag, aruco, or similar detection. Provides the tag's position and rotation
     * relative to the camera that detected it.
     *
     * This packet is 252 bytes long.
     */
    typedef struct tag_detection_t
    {
        uint32_t magic_number;            ///< Unique 32-bit number used to signal the beginning of a VIO packet while parsing a data stream.
        int32_t id;                       ///< id number of the tag
        float size_m;                     ///< size of the tag in meters
        int64_t timestamp_ns;             ///< timestamp at the middle of the frame exposure in monotonic time
        char name[TAG_NAME_LEN];          ///< optional name of the tag
        int loc_type;                     ///< location type
        float T_tag_wrt_cam[3];           ///< location of the tag with respect to camera frame in meters.
        float R_tag_to_cam[3][3];         ///< rotation matrix from tag frame to camera frame
        float T_tag_wrt_fixed[3];         ///< only set if location type is LOCATION_FIXED
        float R_tag_to_fixed[3][3];       ///< only set if location type is LOCATION_FIXED
        char cam[MODAL_PIPE_MAX_DIR_LEN]; ///< camera pipe where the detection was made
        int reserved;                     ///< reserved field
    } __attribute__((packed)) tag_detection_t;

/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync. Here we use 16 packets because the
 * tag detection packet is 252-bytes long and this givea a buffer just under 4k
 *
 * Note this is NOT the size of the pipe which can hold more. This is just the
 * read buffer size allocated on the heap into which data from the pipe is read.
 */
#define TAG_DETECTION_RECOMMENDED_READ_BUF_SIZE (sizeof(tag_detection_t) * 16)

/**
 * We recommend tof servers use a pipe size of 64kB which is also the Linux
 * default pipe size. This allows 260 apriltag detections to be stored in the
 * pipe before losing data.
 */
#define TAG_DETECTION_RECOMMENDED_PIPE_SIZE (64 * 1024)

    /**
     * @brief      Use this to simultaneously validate that the bytes from a pipe
     *             contains valid data, find the number of valid packets
     *             contained in a single read from the pipe, and cast the raw data
     *             buffer as an tag_detection_t* for easy access.
     *
     *             This does NOT copy any data and the user does not need to
     *             allocate anapriltag_data_t array separate from the pipe read buffer.
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
     *             cast to an tag_detection_t* struct for convenience. If there was an
     *             error then NULL is returned and n_packets is set to 0
     */
    tag_detection_t *pipe_validate_tag_detection_t(char *data, int bytes, int *n_packets);

    /**
     * @brief      convert a tag location type id number to string
     *
     * For example TAG_LOCATION_FIXED will return the string "fixed"
     *
     * @param[in]  i     location type id, e.g. TAG_LOCATION_FIXED
     *
     * @return     const char8 string of the format
     */
    const char *pipe_tag_location_type_to_string(int i);

#ifdef __cplusplus
}
#endif

#endif