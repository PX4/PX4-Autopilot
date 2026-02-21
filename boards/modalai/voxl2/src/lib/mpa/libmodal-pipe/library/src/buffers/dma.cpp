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

#define LE_CAMERA
#include <cerrno>
#include <cstdint>
#include <fcntl.h>
#include <assert.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <unordered_map>
#include <linux/dma-heap.h>

#include <hardware/camera3.h>
#include <hardware/linuxembeddedgralloc.h>
#include <camx/camxformatutilexternal.h>

#include "../buffers.h"

#ifndef MSM_MEDIA_ALIGN
#define MSM_MEDIA_ALIGN(__sz, __align) (((__align) & ((__align) - 1)) ?\
    ((((__sz) + (__align) - 1) / (__align)) * (__align)) :\
    (((__sz) + (__align) - 1) & (~((__align) - 1))))
#endif

static int g_dev_fd;

static void* _get_uv_start_from_fmt(uint8_t* block_start, uint32_t aligned_width, uint32_t aligned_height,
        uint32_t fmt) {
    switch(fmt) {
        case HAL_PIXEL_FORMAT_YCBCR_420_888:
            return static_cast<void*>(block_start + (aligned_width * aligned_height));

        case HAL_PIXEL_FORMAT_BLOB:
        case HAL_PIXEL_FORMAT_RAW10:
        case HAL_PIXEL_FORMAT_RAW12:
            return nullptr;

        default:
            printf("Got unsupported format in %s, returning nullptr\n", __FUNCTION__);
            return nullptr;
    }
}

static const std::unordered_map<uint32_t, CamxPixelFormat> hal_to_camx_pix_fmt = {
    {HAL_PIXEL_FORMAT_BLOB,          CamxPixelFormat::CAMERA_PIXEL_FORMAT_BLOB},
    {HAL_PIXEL_FORMAT_RAW10,         CamxPixelFormat::CAMERA_PIXEL_FORMAT_RAW10},
    {HAL_PIXEL_FORMAT_RAW12,         CamxPixelFormat::CAMERA_PIXEL_FORMAT_RAW12},
    {HAL_PIXEL_FORMAT_RAW16,         CamxPixelFormat::CAMERA_PIXEL_FORMAT_RAW16},
    {HAL_PIXEL_FORMAT_YCBCR_420_888, CamxPixelFormat::CAMERA_PIXEL_FORMAT_MULTIPLANAR_FLEX}, // NV12
};

static bool _try_convert_hal_to_camx_pixel_format(uint32_t hal_format, CamxPixelFormat& camx_format) {
    auto it = hal_to_camx_pix_fmt.find(hal_format);
    if (it != hal_to_camx_pix_fmt.end()) {
        camx_format = it->second;
        return true;
    }
    return false;
}

static int _calculate_color_format_params(uint32_t hal_format, int width, int height, 
                                        int* stride, int* scanline, unsigned int* size) {
    switch (hal_format) {
        case HAL_PIXEL_FORMAT_RGBA_8888:
        {
            uint32_t bpp = 4;
            uint32_t plane_align = 4096;
            uint32_t stride_align = 128;
            uint32_t scanline_align = 32;

            *stride   = MSM_MEDIA_ALIGN(width * bpp, stride_align);
            *scanline = MSM_MEDIA_ALIGN(height, scanline_align);
            *size     = MSM_MEDIA_ALIGN((*stride) * (*scanline), plane_align);
            break;
        }
            
        default:
            return -ENOTSUP;  // Format not supported
    }

    return 0;
}

static int _calculate_camx_format_params(uint32_t hal_format, int width, int height,
                                       int* stride, int* scanline, unsigned int* size) {
    CamxPixelFormat cam_format;
    int res;
    
    // Try to convert HAL format to CamX format
    if (!_try_convert_hal_to_camx_pixel_format(hal_format, cam_format)) {
        return -ENOTSUP;  // Format not supported by CamX
    }
    
    // Get buffer size
    res = CamxFormatUtil_GetBufferSize(cam_format, width, height, size);
    if (res != 0) {
        printf("%s: failed to get size for CamX format: %d\n", __func__, res);
        return res;
    }

    // Only get size for blob types
    // stride and scanline should be width and height respectively
    if (hal_format == HAL_PIXEL_FORMAT_BLOB) {
        *stride = width;
        *scanline = height;
        return 0;
    }

    int plane_count;
    CamxPlaneType plane_types[CamxFormatUtilMaxNumPlanes] = {};
    res = CamxFormatUtil_GetPlaneTypes(cam_format, plane_types, &plane_count);
    if (res != 0) {
        printf("%s: failed to get plane types for CamX format: %d\n", __func__, res);
        return res;
    }

    res = CamxFormatUtil_GetStrideInBytes(cam_format, plane_types[0], width, stride);
    if (res != 0) {
        printf("%s: failed to get stride for CamX format: %d\n", __func__, res);
        return res;
    }

    res = CamxFormatUtil_GetScanline(cam_format, plane_types[0], height, scanline);
    if (res != 0) {
        printf("%s: failed to get scanline for CamX format: %d\n", __func__, res);
        return res;
    }

    return 0;
}

int init_buffer_allocator() {
    g_dev_fd = open("/dev/dma_heap/qcom,system", O_RDONLY | O_CLOEXEC);
    assert(g_dev_fd >= 0);
    return 0;
}

int shutdown_buffer_allocator() {
    if (g_dev_fd >= 0) {
        close(g_dev_fd);
    }
    return 0;
}

int allocate_one_buffer(mpa_ion_buf_t* buf, int width, int height, uint32_t hal3_format, uint32_t gralloc_flags) {
    int res;
    int stride = width;
    int scanline = height;
    unsigned int size = 0;

    /*
    * We must leverage the CamxFormatUtils API in order to get the pixel format information to create
    * the buffer. The Camx API does not support color pixel formats, so we must calculate
    * that ourselves.
    */
    res = _calculate_camx_format_params(hal3_format, width, height, &stride, &scanline, &size);
    if (res != 0) {
        // Camx failed, this likely means we have a color fmt but it could also be a true failure
        res = _calculate_color_format_params(hal3_format, width, height, &stride, &scanline, &size);
    }

    if (res != 0) {
        // Camx API failed and we don't have a color format
        printf("Unsupported HAL format: 0x%08X\n", hal3_format);
        return -EINVAL;
    } 

    // Allocate DMA buffer
    struct dma_heap_allocation_data alloc_data;
    alloc_data.fd = 0;
    alloc_data.len = (unsigned long)size;
    alloc_data.fd_flags = O_RDWR | O_CLOEXEC;
    alloc_data.heap_flags = 0;

    res = ioctl(g_dev_fd, DMA_HEAP_IOCTL_ALLOC, &alloc_data);
    if (0 != res) {
        printf("%s: Unable to allocate DMA buffer: %d\n", __func__, res);
        return -EINVAL;
    }

    // Map the buffer
    void* buf_addr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, alloc_data.fd, 0);
    if (buf_addr == NULL) {
        int errsv = errno;
        printf("Unable to mmap buffer object: %s (errno=%d)\n", strerror(errsv), errsv);
        close(alloc_data.fd);
        return -EINVAL;
    }

    // Fill buffer structure
    buf->vaddress = buf_addr;
    buf->uvHead   = _get_uv_start_from_fmt((uint8_t*)buf_addr, stride, scanline, hal3_format);
    buf->size     = size;
    buf->width    = width;
    buf->height   = height;
    buf->stride   = stride;
    buf->slice    = scanline;
    buf->fd       = alloc_data.fd;

    // Create private handle
    private_handle_t *priv_handle = new private_handle_t(alloc_data.fd, size, (int)gralloc_flags, 0, hal3_format, width, height);
    priv_handle->width = stride;
    priv_handle->height = scanline;

    buf->handle = static_cast<native_handle_t*>(priv_handle);

    return 0;
}

int delete_one_buffer(mpa_ion_buf_t* buf) {
    // printf("Unmapping buffer: vaddr=%p, size=%zu bytes\n", buf->vaddress, buf->size);
    munmap(buf->vaddress, buf->size);
    close(buf->fd);
    delete buf->handle;

    return 0;
}
