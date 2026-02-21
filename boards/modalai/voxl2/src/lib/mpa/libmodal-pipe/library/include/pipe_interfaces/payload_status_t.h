#ifndef PAYLOAD_STATUS_T_H
#define PAYLOAD_STATUS_T_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////
// PAYLOAD STATUS
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief      Common interface for Payload Status
 *
 *             Applications that show payload status can use this interface
 */

/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream.
 *
 */
#include "magic_number.h"
#ifndef PAYLOAD_STATUS_MAGIC_NUMBER
#error "PAYLOAD_STATUS_MAGIC_NUMBER not defined!"
#endif


// some generic reserved payload types, field is 32-bits so feel free to use
// your own random number for your unique payload
// This list will likely continue to grow monotonically.
#define MPA_PAYL_TYPE_TEST_FIXTURE  1
#define MPA_PAYL_TYPE_DROPPER       2
#define MPA_PAYL_TYPE_SPEAKER       3
#define MPA_PAYL_TYPE_FLASHLIGHT    4
#define MPA_PAYL_TYPE_GENERIC_USB   5
#define MPA_PAYL_TYPE_GENERIC_FTDI  6
#define MPA_PAYL_TYPE_RESERVED_1    7
#define MPA_PAYL_TYPE_RESERVED_2    8
#define MPA_PAYL_TYPE_RESERVED_3    9
#define MPA_PAYL_TYPE_RESERVED_4    10

// Flags that can be set in the "State" bitmask. The field is 64 bits long so
// feel free to use unused bits in the upper end of the bit range for custom
// features. This list will likely continue to grow monotonically.
#define MPA_PAYL_FLAG_DISCONNECT  (1<<0)
#define MPA_PAYL_FLAG_ERROR       (1<<1)
#define MPA_PAYL_FLAG_INPUT_ERROR (1<<2)
#define MPA_PAYL_FLAG_STARTUP     (1<<3)
#define MPA_PAYL_FLAG_IDLE        (1<<4)
#define MPA_PAYL_FLAG_ARMED       (1<<5)
#define MPA_PAYL_FLAG_TRIGGERED   (1<<6)
#define MPA_PAYL_FLAG_OPEN        (1<<7)
#define MPA_PAYL_FLAG_CLOSED      (1<<8)
#define MPA_PAYL_FLAG_SST_30      (1<<9)
#define MPA_PAYL_FLAG_SST_60      (1<<10)
#define MPA_PAYL_FLAG_SST_90      (1<<11)
#define MPA_PAYL_FLAG_EN_IMP      (1<<12)
#define MPA_PAYL_FLAG_EN_FAR      (1<<13)
#define MPA_PAYL_FLAG_EN_NEAR     (1<<14)
#define MPA_PAYL_FLAG_ARMED_IMP   (1<<15)
#define MPA_PAYL_FLAG_ARMED_FAR   (1<<16)
#define MPA_PAYL_FLAG_ARMED_NEAR  (1<<17)



/**
 * This is the data structure that holds the payload status data.
 */
typedef struct payload_status_t
{
    uint32_t magic_number;  ///< Unique 32-bit number used to signal the beginning of a payload_status packet.
    uint32_t type;          ///< Payload type
    uint64_t timestamp_ns;  ///< Timestamp in ns
    uint64_t state;         ///< Bitmask of state flags
    char status[64];        ///< Status string
    int32_t pwm_state_1;    ///< Current output PWM signal in us. -1 = unknown. 0=GPIO_OFF, int32_max=GPIO_ON
    int32_t pwm_state_2;    ///< Current output PWM signal in us. -1 = unknown. 0=GPIO_OFF, int32_max=GPIO_ON
    int32_t reserved_1;     ///< Reserved for future use
    int32_t reserved_2;     ///< Reserved for future use
} __attribute__((packed)) payload_status_t;


/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync.
 *
 * Note this is NOT the size of the pipe which can hold more. This is just the
 * read buffer size allocated on the heap into which data from the pipe is read.
 */
#define PAYLOAD_STATUS_RECOMMENDED_READ_BUF_SIZE (sizeof(payload_status_t) * 32)

/**
 * 64KB is the Linux default pipe size, which should be more than enough since
 * payload status messages do not come out at a very high rate.
 */
#define PAYLOAD_STATUS_RECOMMENDED_PIPE_SIZE (64 * 1024)

#ifdef __cplusplus
}
#endif

#endif
