#ifndef CRSF_RAW_T_H
#define CRSF_RAW_T_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////
// CRSF RAW DATA
////////////////////////////////////////////////////////////////////////////////

/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream.
 *
 */
#include "magic_number.h"
#ifndef CRSF_RAW_MAGIC_NUMBER
#error "CRSF_RAW_MAGIC_NUMBER not defined!"
#endif

/**
 * This is the data structure that holds the raw CRSF frame data.
 * Total size: 80 bytes (4+8+1+64+3). Reserved bytes pad to 8-byte alignment.
 */
typedef struct crsf_raw_data_t
{
    uint32_t magic_number; ///< Unique 32-bit number used to signal the beginning of a CRSF raw packet.
    uint32_t len;          ///< Length of data
    int64_t timestamp_ns;  ///< Timestamp in nanoseconds
    uint8_t data[64];      ///< Raw CRSF frame data
    int32_t reserved_1;    ///< Reserved for future use
    int32_t reserved_2;    ///< Reserved for future use
} __attribute__((packed)) crsf_raw_data_t;


/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync.
 *
 * Note this is NOT the size of the pipe which can hold more. This is just the
 * read buffer size allocated on the heap into which data from the pipe is read.
 */
#define CRSF_RAW_RECOMMENDED_READ_BUF_SIZE (sizeof(crsf_raw_data_t) * 32)

/**
 * 64KB is the Linux default pipe size, more than enough for this data
 */
#define CRSF_RAW_RECOMMENDED_PIPE_SIZE (64 * 1024)

crsf_raw_data_t *pipe_validate_crsf_raw_data_t(char *data, int bytes, int *n_packets);

#ifdef __cplusplus
}
#endif

#endif
