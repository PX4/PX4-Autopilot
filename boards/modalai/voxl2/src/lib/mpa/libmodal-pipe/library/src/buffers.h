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

#ifndef PIPE_BUFFERS_H
#define PIPE_BUFFERS_H

#include <cstdint>
#include <modal_pipe_buffers.h>

/**
 * @brief Initialize the buffer allocator subsystem
 * 
 * @return 0 on success
 * @note This function will assert on failure
 */
int init_buffer_allocator(void);

/**
 * @brief close the device fd release system resources
 * 
 * @return 0 on success
 */
int shutdown_buffer_allocator(void);

/**
 * @brief Allocate a single buffer
 * 
 * @param buf Pointer to mpa_ion_buf_t structure to populate with buffer information
 * @param width Image width in pixels
 * @param height Image height in pixels  
 * @param hal3_format HAL3 pixel format (e.g., HAL_PIXEL_FORMAT_YCbCr_420_888)
 * @param gralloc_flags Gralloc usage flags for buffer allocation
 * 
 * @return 0 on success, -EINVAL on error
 * 
 */
int allocate_one_buffer(mpa_ion_buf_t* buf, int width, int height, uint32_t hal3_format, uint32_t gralloc_flags);

/**
 * @brief Release a previously allocated buffer
 * 
 * @param buf Pointer to mpa_ion_buf_t structure containing buffer to release
 * 
 * @return 0 on success
 */
int delete_one_buffer(mpa_ion_buf_t* buf);

#endif // PIPE_BUFFERS_H
