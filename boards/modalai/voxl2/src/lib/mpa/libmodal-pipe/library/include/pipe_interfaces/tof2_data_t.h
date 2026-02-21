#ifndef TOF2_DATA_T_H
#define TOF2_DATA_T_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////
// TOF2 this is the replacement for the now-deprecated tof_data_t
// the second gen PMD TOF sensor has a larger resolution. This packet type
// supports both old and new sensors. They are differentiated by the resolution.
////////////////////////////////////////////////////////////////////////////////

/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream.
 */
#include "magic_number.h"
#ifndef TOF2_MAGIC_NUMBER
#error "TOF2_MAGIC_NUMBER not defined!"
#endif

#define MPA_MAX_TOF2_WIDTH 240
#define MPA_MAX_TOF2_HEIGHT 180
#define MPA_MAX_TOF2_SIZE (MPA_MAX_TOF2_WIDTH * MPA_MAX_TOF2_HEIGHT)

    /*
     * this packet is big enough for both gen tof sensors. when using the first
     * gen (A65) sensor, the ends of each array should be ignored
     */
    typedef struct tof2_data_t
    {
        uint32_t magic_number; ///< Unique 32-bit number used to signal the beginning of a packet while parsing a data stream.
        int64_t timestamp_ns;  ///< timestamp in nanoseconds
        int16_t width;         ///< 224 for first gen (A65), 240 for second gen (LIOW2)
        int16_t height;        ///< 172 for first gen (A65), 180 for second gen (LIOW2)
        int16_t reserved1;
        int16_t reserved2;
        float points[MPA_MAX_TOF2_SIZE][3];     ///< Point cloud (x,y,z in meters)
        float noises[MPA_MAX_TOF2_SIZE];        ///< noise value for each point (meters)
        uint8_t grayValues[MPA_MAX_TOF2_SIZE];  ///< IR grayvalue for each point
        uint8_t confidences[MPA_MAX_TOF2_SIZE]; ///< Confidence value for each point
    } __attribute__((packed)) tof2_data_t;

/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync. Here we use 4 packets because the
 * tof data packet is massive and we wont be expecting to get them at more than
 * 15-30 hz
 *
 * Note this is NOT the size of the pipe which can hold more. This is just
 * the read buffer size allocated on the heap into which data from the pipe is
 * read.
 */
#define TOF2_RECOMMENDED_READ_BUF_SIZE (sizeof(tof2_data_t) * 4)

/**
 * We recommend tof servers use a pipe size of 64 mb. This allows 6 seconds
 * of tof data at 15 hz to be stored
 */
#define TOF2_RECOMMENDED_PIPE_SIZE (1024 * 1024 * 64)

    /**
     * @brief      Use this to simultaneously validate that the bytes from a pipe
     *             contains valid data, find the number of valid packets
     *             contained in a single read from the pipe, and cast the raw data
     *             buffer as a tof2_data_t* for easy access.
     *
     *             This does NOT copy any data and the user does not need to
     *             allocate a tof_data_t array separate from the pipe read buffer.
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
     *             cast to an tof_data_t* struct for convenience. If there was an
     *             error then NULL is returned and n_packets is set to 0
     */
    tof2_data_t *pipe_validate_tof2_data_t(char *data, int bytes, int *n_packets);

#ifdef __cplusplus
}
#endif

#endif