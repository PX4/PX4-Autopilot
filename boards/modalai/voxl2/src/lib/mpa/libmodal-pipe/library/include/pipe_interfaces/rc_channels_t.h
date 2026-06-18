#ifndef RC_CHANNELS_T_H
#define RC_CHANNELS_t_h

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////
// RC CHANNELS
////////////////////////////////////////////////////////////////////////////////

/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream.
 *
 */
#include "magic_number.h"
#ifndef RC_CHANNELS_MAGIC_NUMBER
#error "RC_CHANNELS_MAGIC_NUMBER not defined!"
#endif

/**
 * This is the data structure that holds the barometer data.
 */
typedef struct rc_channels_data_t
{
    uint32_t magic_number;     ///< Unique 32-bit number used to signal the beginning of a rc_channels packet.
    uint32_t timestamp_ns;     ///< [ns] Timestamp in ns
    uint16_t channels[18];     ///< [us] Array of up to 18 RC channel values
    uint8_t chancount;         ///< Total number of RC channels being received 
    uint8_t rssi;              ///< Receive signal strength indicator in device-dependent units/scale. Values: [0-254]
    
} __attribute__((packed)) rc_channels_data_t;


/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync.
 *
 * Note this is NOT the size of the pipe which can hold more. This is just the
 * read buffer size allocated on the heap into which data from the pipe is read.
 */
#define RC_CHANNELS_RECOMMENDED_READ_BUF_SIZE (sizeof(rc_channels_data_t) * 46)

/**
 * 64KB is the Linux default pipe size, which should be more than enough since
 *  RC_CHANNEL messages do not come out at a very high rate. 
 */
#define RC_CHANNELS_RECOMMENDED_PIPE_SIZE (64 * 1024)

rc_channels_data_t *pipe_validate_rc_channels_data_t(char *data, int bytes, int *n_packets);

#ifdef __cplusplus
}
#endif

#endif
