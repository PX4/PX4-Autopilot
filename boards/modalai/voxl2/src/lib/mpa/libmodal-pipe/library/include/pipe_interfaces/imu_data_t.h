#ifndef IMU_DATA_T_H
#define IMU_DATA_T_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <float.h>

////////////////////////////////////////////////////////////////////////////////
// IMU
////////////////////////////////////////////////////////////////////////////////

/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream.
 */
#include "magic_number.h"
#ifndef IMU_MAGIC_NUMBER
#error "IMU_MAGIC_NUMBER not defined!"
#endif

/**
 * If a device cannot read temperature or temperature reading is disabled then
 * IMU_INVALID_TEMPERATURE_VALUE should be present in the temp_c field to
 * indicate this.
 */
#define IMU_INVALID_TEMPERATURE_VALUE (FLT_MIN)

    /**
     * This is the data structure that imu servers should make available to clients
     * on the data pipe. (40 bytes long)
     */
    typedef struct imu_data_t
    {
        uint32_t magic_number; ///< Set to IMU_IMAGE_MAGIC_NUMBER for frame syncing
        float accl_ms2[3];     ///< XYZ acceleration in m/s^2
        float gyro_rad[3];     ///< XYZ gyro rotation in rad/s
        float temp_c;          ///< temp in C, IMU_INVALID_TEMPERATURE_VALUE if no thermometer present
        uint64_t timestamp_ns; ///< timestamp in nanoseconds, uses system clock_monotonic
    } __attribute__((packed)) imu_data_t;

/**
 * You don't have to use this read buffer size, but it is HIGHLY
 * recommended to use a multiple of the packet size so that you never read a
 * partial packet which would throw the reader out of sync. Here we use a nice
 * number of 400 packets which is perhaps more than necessary but only takes a
 * little under 16K of memory which is minimal.
 *
 * Note this is NOT the size of the pipe which can hold much more. This is just
 * the read buffer size allocated on the heap into which data from the pipe is
 * read.
 */
#define IMU_RECOMMENDED_READ_BUF_SIZE (sizeof(imu_data_t) * 400)

/**
 * We recommend IMU servers use a 128k pipe size. This means every client would
 * get their own buffer of over 3.2 seconds of IMU data at 1000hz. Clients can
 * increase this buffer if they wish. voxl-imu-server uses this as its default.
 */
#define IMU_RECOMMENDED_PIPE_SIZE (128 * 1024)

/**
 * @brief      Use this to simultaneously validate that the data from a pipe
 *             contains valid imu data, find the number of valid packets
 *             contained in a single read from the pipe, and cast the raw data
 *             buffer as an imu_data_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate an imu_data_t array separate from the pipe read buffer.
 *             The data can be read straight out of the pipe read buffer, much
 *             like reading data directly out of a mavlink_message_t message.
 *
 *             However, this does mean the user should finish processing this
 *             data before returning the pipe data callback which triggers a new
 *             read() from the pipe.
 *
 * @param[in]  data       pointer to pipe read data buffer
 * @param[in]  bytes      number of bytes read into that buffer
 * @param[out] n_packets  number of valid imu_data_t packets received
 *
 * @return     Returns the same data pointer provided by the first argument, but
 *             cast to an imu_data_t* struct for convenience. If there was an
 *             error then NULL is returned and n_packets is set to 0
 */
imu_data_t* pipe_validate_imu_data_t(char *data, int bytes, int *n_packets);

#ifdef __cplusplus
}
#endif

#endif
