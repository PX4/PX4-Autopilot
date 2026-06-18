/*******************************************************************************
 * Copyright 2025 ModalAI Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 ******************************************************************************/



#ifndef MODAL_PIPE_BUFFER_H
#define MODAL_PIPE_BUFFER_H


#ifdef EN_ION_BUF

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <pthread.h>
#include <modal_pipe_common.h>
#include <modal_pipe_interfaces.h>


#define GRALLOC_USAGE_PRIVATE_UNCACHED (UINT32_C(1) << 29)
#define ENCODER_USAGE  GRALLOC_USAGE_HW_VIDEO_ENCODER

#ifdef __cplusplus
#define MPA_ION_BUF_INITIALIZER mpa_ion_buf_t()
#else
#define MPA_ION_BUF_INITIALIZER {         \
    .magic_number = 0,                    \
    .buffer_id = -1,                       \
    .vaddress = NULL,                     \
    .fd = -1,                             \
    .consumerFlags = 0,                   \
    .size = 0,                            \
    .width = 0,                           \
    .height = 0,                          \
    .stride = 0,                          \
    .slice = 0,                           \
    .format = 0,                          \
    .reserved1 = 0,                       \
    .reserved2 = 0,                       \
    .reserved3 = 0,                       \
    .reserved4 = 0                        \
}
#endif

#define MPA_ION_BUF_POOL_MAX_SIZE 64

typedef struct mpa_ion_buf_pool_t {
	uint32_t		initialized;
	uint32_t		n_bufs;
	mpa_ion_buf_t	bufs[MPA_ION_BUF_POOL_MAX_SIZE];

    uint32_t        client_mask[MPA_ION_BUF_POOL_MAX_SIZE];     // one bit per client
    uint32_t        generation[MPA_ION_BUF_POOL_MAX_SIZE];      // sync with buf.generation
    uint8_t         producer_active[MPA_ION_BUF_POOL_MAX_SIZE]; //  0/1

    uint32_t		in_use[MPA_ION_BUF_POOL_MAX_SIZE];
	uint32_t		n_client_references[MPA_ION_BUF_POOL_MAX_SIZE];
    int64_t			time_sent_to_clients_ns[MPA_ION_BUF_POOL_MAX_SIZE];

	pthread_mutex_t	pool_mtx;
} mpa_ion_buf_pool_t;


////////////////////////////////////////////////////////////////////////////////
// General function for allocating buffer pools, mostly used by camera server
////////////////////////////////////////////////////////////////////////////////



/**
 * @brief      use GBM to predict how big an ION buffer needs to be for a certain
 * camera frame size and format.
 *
 * use before allocating buffers to see how big they need to be.
 *
 * @param[in]  h       height
 * @param[in]  w       width
 * @param[in]  format  see IMAGE_FORMAT_YUV422 etc in modal_pipe_interfaces.h
 *
 * @return     size in bytes on success, -1 on failure
 */
int32_t mpa_ion_buf_pool_predict_cam_frame_size(int h, int w, int format);


/**
 * @brief      allocate an entire pool of buffers
 *
 * @param      pool_ptr         The pool pointer
 * @param      n_bufs           The n bufs
 * @param      width            The image width
 * @param      height           The image height
 * @param      hal3_format      The image format
 * @param      gralloc_flags    The gralloc flags
 *
 * @return     0 on success, -1 on failure
 */
int mpa_ion_buf_pool_alloc_bufs(mpa_ion_buf_pool_t* pool_ptr, uint32_t n_bufs, uint32_t width,
                           		uint32_t height, uint32_t hal3_format, uint32_t gralloc_flags);


/**
 * @brief      deallocates all buffers in a pool, cleans up the pool's data
 *             like it is no longer going to be used
 * 
 * @param      pool_ptr         The pool pointer
 *
 * @return     0 on success, -1 on failure
 */
int mpa_ion_buf_pool_delete_bufs(mpa_ion_buf_pool_t* pool_ptr);


/**
 * @brief      frees all memory in a buffer pool and resets the pool struct back
 *             to zeros.
 *
 * @param      pool_ptr  pointer to your pool struct to free
 *
 * @return     0 on success, -1 on failure
 */
int mpa_ion_buf_pool_free(mpa_ion_buf_pool_t* pool_ptr);


/**
 * @brief      Finds a buffer match for the given id and decrements the
 *             client references for the buffer. If the client references is
 *             decremented to 0, sets the buffer as unused. 
 * 
 * @param      pool_ptr         The pool pointer
 * @param      id               The buffer id used to find the buffer
 * 
 * @return     0 on sucess, -1 on failure
 */
int mpa_ion_buf_flag_as_unused_by_id(mpa_ion_buf_pool_t* pool_ptr, int id);


/**
 * @brief     Validates the buffer_id and generation value then decrements 
 *            the client references, and toggles the client_mask for the 
 *            buffer. If the client references is decremented to 0, and the
 *            client mask is empty, sets the buffer as unused
 * 
 * @param     pool_ptr          The pool pointer
 * @param     msg               The msg that holds buffer and generation info  
 * 
 * @return    0 on sucess, -1 on failure
 */
int mpa_ion_buf_process_release_msg(mpa_ion_buf_pool_t* pool_ptr, ion_buf_release_msg_t msg);


/**
 * @brief      Finds a buffer match for the given address and decrements the
 *             client references for the buffer. If the client references is
 *             decremented to 0, sets the buffer as unused. 
 * 
 * @param      pool_ptr         The pool pointer
 * @param      vaddress         The address used to find the buffer
 * 
 * @return     0 on sucess, -1 on failure
 */
int mpa_ion_buf_flag_as_unused_by_vaddress(mpa_ion_buf_pool_t* pool_ptr, void* vaddress);


/**
 * @brief      Finds a buffer match for the given handle and decrements the
 *             client references for the buffer. If the client references is
 *             decremented to 0, sets the buffer as unused. 
 * 
 * @param      pool_ptr         The pool pointer
 * @param      handle           The handle used to find the buffer
 * 
 * @return     0 on sucess, -1 on failure
 */
int mpa_ion_buf_flag_as_unused_by_handle(mpa_ion_buf_pool_t* pool_ptr, buffer_handle_t* handle);


/**
 * @brief      Fetch the pointer to an ion buf either by address.
 *
 *             Be careful not to modify the data as it points to the original
 *             pool. This should be used to inspect the buffer data. For
 *             example, when HAL3 returns a camera frame it provides the
 *             vaddress in the callback. we can use that to look up which buffer
 *             from our pool was used.
 *
 * @param      pool_ptr  The pool pointer
 * @param      vaddress  The address used to find the buffer
 *
 * @return     buffer pointer on success, null on failure
 */
mpa_ion_buf_t* mpa_ion_buf_pool_get_buf_info_by_vaddress(mpa_ion_buf_pool_t* pool_ptr, void* vaddress);


/**
 * @brief      Fetch the pointer to an ion buf either by id.
 *
 *             Be careful not to modify the data as it points to the original
 *             pool. This should be used to inspect the buffer data. For
 *             example, when HAL3 returns a camera frame it provides the
 *             vaddress in the callback. we can use that to look up which buffer
 *             from our pool was used.
 *
 * @param      pool_ptr  The pool pointer
 * @param      id        The buffer id used to find the buffer
 *
 * @return     buffer pointer on success, null on failure
 */
mpa_ion_buf_t* mpa_ion_buf_pool_get_buf_info_by_id(mpa_ion_buf_pool_t* pool_ptr, int id);


/**
 * @brief      Fetch the pointer to an ion buf either by handle.
 *
 *             Be careful not to modify the data as it points to the original
 *             pool. This should be used to inspect the buffer data. For
 *             example, when HAL3 returns a camera frame it provides the
 *             vaddress in the callback. we can use that to look up which buffer
 *             from our pool was used.
 *
 * @param      pool_ptr  The pool pointer
 * @param      handle    The handle used to find the buffer
 *
 * @return     buffer pointer on success, null on failure
 */
mpa_ion_buf_t* mpa_ion_buf_pool_get_buf_info_by_handle(mpa_ion_buf_pool_t* pool_ptr, buffer_handle_t* handle);


/**
 * @brief      Returns the next available unused buffer for use.
 *
 *             It flags the buffer as used. The buffer will be
 *             flagged as used until a flag_as_unused* function is called.
 *
 * @param      pool_ptr  The pool pointer
 *
 * @return     pointer to the next available buffer or null on failure
 */
mpa_ion_buf_t* mpa_ion_buf_pool_claim_unused_buf(mpa_ion_buf_pool_t* pool_ptr);


/**
 * @brief      Returns the next available unused buffer for use.
 *
 *             It flags the buffer associated with this handle as used. 
 *             The buffer will be flagged as used until a flag_as_unused* 
 *             function is called.
 *
 * @param      pool_ptr  The pool pointer
 *
 * @return     pointer to the next available buffer handle or nullptr on failure
 */
buffer_handle_t* mpa_ion_buf_pool_claim_unused_handle(mpa_ion_buf_pool_t* pool_ptr);


/**
 * @brief      This function is used to decrement the client references value 
 *             for the matching buffer. If the client references becomes 0, 
 *             then we set that buffer to as unused, meaning it can be claimed
 *             for any new image
 * 
 * @param      pool_ptr  The pool pointer
 * @param      buf       The ion buffer
 *
 * @return     pointer to the next available buffer or -1 on failure
 */
int mpa_ion_buf_pool_mark_buf_unused(mpa_ion_buf_pool_t* pool_ptr, mpa_ion_buf_t buf);


/**
 *  @brief     This function is a predefined implementation of the control pipe
 *             callback function. It is used to automatically call 
 *             mpa_ion_buf_pool_mark_buf_unused() for buffers the client writes
 *             back to the control pipe to show they're not used anymore
 * 
 *             NOTE: implemented in server.c
 */
void _ion_buf_control_cb(int ch, char* data, int bytes, __attribute__((unused)) void* context);


/**
 * @brief      helper function to get the number of unused buffers in a pool
 *             that are ready to be claimed.
 *
 * @param      pool_ptr  The pool pointer
 *
 * @return     number of unused buffers or -1 on error
 */
int mpa_ion_buf_pool_get_num_unused(mpa_ion_buf_pool_t* pool_ptr);



////////////////////////////////////////////////////////////////////////////////
// Functions used by camera server to send ion buffers to clients
////////////////////////////////////////////////////////////////////////////////

/**
 * We recommend ion buffer servers use small pipes, a single 4k page is plenty.
 */
#define MPA_ION_BUF_RECOMMENDED_PIPE_SIZE   (4 * 1024)


#ifdef __cplusplus
}
#endif

#endif 

#endif // MODAL_PIPE_BUFFER_H
