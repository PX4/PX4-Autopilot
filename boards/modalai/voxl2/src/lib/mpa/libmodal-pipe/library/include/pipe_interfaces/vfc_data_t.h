#ifndef VFC_DATA_T_H
#define VFC_DATA_T_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////
// VFC
////////////////////////////////////////////////////////////////////////////////

/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream. If this were to be cast as a float it would have a
 * value of 5.7x10^13 which is an impossible value for translations/rotation
 * readings making it unique as an identifier.
 *
 */
#include "magic_number.h"
#ifndef VFC_MAGIC_NUMBER
#error "VFC_MAGIC_NUMBER not defined!"
#endif

    /**
     * This is the data structure that voxl flight controller (VFC) makes available
     * to indicate current status.
     */
    typedef struct vfc_data_t
    {
        uint32_t magic_number; ///< Unique 32-bit number used to signal the beginning of a VFC status packet while parsing a data stream.
        bool altitude_ok;
        bool flow_ok;
        bool position_ok;
        float backtrack_seconds;
        bool backtrack_active;
        int8_t desired_submode;
        int8_t actual_submode;
    } __attribute__((packed)) vfc_data_t;

/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync.
 *
 * Note this is NOT the size of the pipe which can hold much more. This is just
 * the read buffer size allocated on the heap into which data from the pipe is
 * read.
 */
#define VFC_RECOMMENDED_READ_BUF_SIZE (sizeof(vfc_data_t) * 32)

    /**
     * @brief      Use this to simultaneously validate that the bytes from a pipe
     *             contains valid data, find the number of valid packets
     *             contained in a single read from the pipe, and cast the raw data
     *             buffer as a vfc_data_t* for easy access.
     *
     *             This does NOT copy any data and the user does not need to
     *             allocate a vfc_data_t array separate from the pipe read buffer.
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
     *             cast to an vfc_data_t* struct for convenience. If there was an
     *             error then NULL is returned and n_packets is set to 0
     */
    vfc_data_t *pipe_validate_vfc_data_t(char *data, int bytes, int *n_packets);

#ifdef __cplusplus
}
#endif

#endif