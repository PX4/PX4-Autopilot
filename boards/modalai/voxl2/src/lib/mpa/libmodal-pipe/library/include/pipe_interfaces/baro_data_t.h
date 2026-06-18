#ifndef BARO_DATA_T_H
#define BARO_DATA_T_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////
// BAROMETER
////////////////////////////////////////////////////////////////////////////////

/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream.
 *
 */
#include "magic_number.h"
#ifndef BARO_MAGIC_NUMBER
#error "BARO_MAGIC_NUMBER not defined!"
#endif

/**
 * This is the data structure that holds the barometer data.
 */
typedef struct baro_data_t
{
    uint32_t magic_number; ///< Unique 32-bit number used to signal the beginning of a Baro packet.
    float pressure_pa;     ///< Pressure in Pascals
    float temp_c;          ///< Temperature in Celsius
    float alt_amsl_m;      ///< best estiamte for amsl altitude based only on this sensor
    int64_t timestamp_ns;  ///< Timestamp in nanoseconds
    uint32_t reserved_1;   ///< two reserved spots
    uint32_t reserved_2;
} __attribute__((packed)) baro_data_t;


/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync.
 *
 * Note this is NOT the size of the pipe which can hold more. This is just the
 * read buffer size allocated on the heap into which data from the pipe is read.
 */
#define BARO_RECOMMENDED_READ_BUF_SIZE (sizeof(baro_data_t) * 32)

/**
 * 64KB is the Linux default pipe size, more than enough for this tiny packet
 */
#define BARO_RECOMMENDED_PIPE_SIZE (64 * 1024)

baro_data_t *pipe_validate_baro_data_t(char *data, int bytes, int *n_packets);

#ifdef __cplusplus
}
#endif

#endif
