#ifndef MAVLINK_MESSAGE_T_H
#define MAVLINK_MESSAGE_T_H

#ifndef CTYPESGEN

#ifdef __cplusplus
extern "C"
{
#endif

#include <c_library_v2/common/mavlink.h>

////////////////////////////////////////////////////////////////////////////////
// mavlink_message_t
//
// Functions to interface with a stream of mavlink_message_t structs. Much like
// our own data structs, mavlink_message_t structs are of a fixed length and
// contain a magic number. Therefore they can be send neatly over a pipe.
//
// While it is possible to send a normal encoded mavlink stream over a pipe,
// this is much easier and saves the CPU from needing to pack/unpack/checksum
// the stream and is therefore more efficient and easier to read. Afterall,
// programs only want access to the final mavlink_message_t in the end. The
// unpacking and decoding of mavlink messages from PX4 is accelerated on the
// SDSP on VOXL already. In userspace land we just send the final
// mavlink_message_t around via pipes.
////////////////////////////////////////////////////////////////////////////////



/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync. Here we use 56 packets which is
 * perhaps more than necessary but only takes a little under 4 pages of memory
 * which is minimal.
 *
 * mavlink_message_t structs are 291 bytes long
 *
 * Note this is NOT the size of the pipe which can hold much more. This is just
 * the read buffer size allocated on the heap into which data from the pipe is
 * read.
 */
#define MAVLINK_MESSAGE_T_RECOMMENDED_READ_BUF_SIZE (sizeof(mavlink_message_t) * 56)

/**
 * We recommend mavlink servers use the typical 1M pipe size for mavlink which
 * can contain up to 3603 messages.
 */
#define MAVLINK_MESSAGE_T_RECOMMENDED_PIPE_SIZE (1024 * 1024)

/**
 * @brief      Use this to simultaneously validate that the data from a pipe
 *             contains valid mavlink_message_t data, find the number of valid
 *             packets contained in a single read from the pipe, and cast the
 *             raw data buffer as a mavlink_message_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate a mavlink_message_t array separate from the pipe read
 *             buffer. The data can be read straight out of the pipe read
 *             buffer.
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
 *             cast to a mavlink_message_t* struct for convenience. If there was
 *             an error then NULL is returned and n_packets is set to 0
 */
mavlink_message_t *pipe_validate_mavlink_message_t(char *data, int bytes, int *n_packets);


#ifdef __cplusplus
}
#endif

#endif // CTYPESGEN

#endif
