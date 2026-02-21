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

#include <gbm.h>
#include <gbm_priv.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdio.h>      // for fprintf
#include <stdlib.h>     // for fprintf
#include <sys/socket.h>
#include <assert.h>
#include <utility>
#include <unordered_map>

#include <camera/CameraMetadata.h>
#include <libhardware/gralloc_priv.h>
#include <cutils/native_handle.h>

#include <modal_pipe_buffers.h>
#include <modal_pipe_server.h>

#include "../buffers.h"


int g_dev_fd;
static struct gbm_device* g_gbm_dev;

int init_buffer_allocator(void)
{
    g_dev_fd = open("/dev/dri/card0", O_RDWR);
    if (g_dev_fd < 0) {
        printf("Unable to open /dev/dri/card0, falling back to /dev/ion\n");

        g_dev_fd = open("/dev/ion", O_RDWR);
        if (g_dev_fd < 0) {
            printf("Opening /dev/ion also failed\n");
        }
    }
    assert(g_dev_fd >= 0);

    g_gbm_dev = gbm_create_device(g_dev_fd);
    assert(g_gbm_dev != NULL);

    return 0;
}

int shutdown_buffer_allocator(void)
{
    if (g_gbm_dev != NULL){
        gbm_device_destroy(g_gbm_dev);
        close(g_dev_fd);
    }

    return 0;
}

// typedef struct {
//     int32_t gralloc_usage_flag;
//     int32_t gbm_usage_flag;
// } gralloc_to_gbm_usage_pair;

static const std::unordered_map<uint32_t, uint32_t> gralloc_usage_flag_map =
{
    {GRALLOC_USAGE_HW_CAMERA_ZSL,      0                              },
    // {GRALLOC_USAGE_PRIVATE_ALLOC_UBWC, GBM_BO_USAGE_UBWC_ALIGNED_QTI}, // Uncomment when available
    {GRALLOC_USAGE_PRIVATE_UNCACHED,   GBM_BO_USAGE_UNCACHED_QTI      },
    {GRALLOC_USAGE_PROTECTED,          GBM_BO_USAGE_PROTECTED_QTI     },
    {GRALLOC_USAGE_SW_READ_OFTEN,      GBM_BO_USAGE_CPU_READ_QTI      },
    {GRALLOC_USAGE_SW_WRITE_OFTEN,     GBM_BO_USAGE_CPU_WRITE_QTI     },
    {GRALLOC_USAGE_HW_VIDEO_ENCODER,   GBM_BO_USAGE_VIDEO_ENCODER_QTI },
    {GRALLOC_USAGE_HW_FB,              0                              },
    {GRALLOC_USAGE_HW_TEXTURE,         0                              },
    {GRALLOC_USAGE_HW_RENDER,          GBM_BO_USAGE_HW_RENDERING_QTI  },
    {GRALLOC_USAGE_HW_COMPOSER,        GBM_BO_USAGE_HW_COMPOSER_QTI   },
    {GRALLOC_USAGE_HW_CAMERA_READ,     GBM_BO_USAGE_CAMERA_READ_QTI   },
    {GRALLOC_USAGE_HW_CAMERA_WRITE,    GBM_BO_USAGE_CAMERA_WRITE_QTI  }
};

// #define GRALLOC_USAGE_FLAG_MAP_SIZE (sizeof(gralloc_usage_flag_map) / sizeof(gralloc_to_gbm_usage_pair))

static int32_t gralloc_flags_to_gbm(int32_t gralloc_flags)
{
    uint32_t output = 0;
    for (auto const& pair : gralloc_usage_flag_map) {
        if (gralloc_flags & pair.first) {
            output |= pair.second;
        }
    }

    // if we have encoder, then remove camera read/write flags
    // for some reason, camera read/write flags cause GBM to get confused and
    // allocated wrongly-sized buffers in certain cases. this is also documented
    // inside QMMF. if we leave these flags in while also having video encoder
    // usage, then camx will crash because it's expecting larger buffers than
    // GBM will allocate.
    if (output & GBM_BO_USAGE_VIDEO_ENCODER_QTI) {
        // printf("Found video encoder usage, removing camera read/write usage\n");
        output = output & ~(GBM_BO_USAGE_CAMERA_READ_QTI | GBM_BO_USAGE_CAMERA_WRITE_QTI);

        // sanity check because I'm paranoid
        if (output & GBM_BO_USAGE_CAMERA_WRITE_QTI || output & GBM_BO_USAGE_CAMERA_READ_QTI) {
            printf("Attempted to remove camera read/write, but still present\n");
        }
    }

    return output;
}


// typedef struct {
//     uint32_t hal3_format_flag;
//     uint32_t gbm_format_flag;
// } hal_to_gbm_format_pair;


static const std::unordered_map<uint32_t, uint32_t> hal_format_flag_map = 
{
  {HAL_PIXEL_FORMAT_BGRA_8888,               GBM_FORMAT_BGRA8888              },
  {HAL_PIXEL_FORMAT_RGB_565,                 GBM_FORMAT_RGB565                },
  {HAL_PIXEL_FORMAT_RGB_888,                 GBM_FORMAT_RGB888                },
  {HAL_PIXEL_FORMAT_RGBA_1010102,            GBM_FORMAT_RGBA1010102           },
  {HAL_PIXEL_FORMAT_RGBA_8888,               GBM_FORMAT_RGBA8888              },
  {HAL_PIXEL_FORMAT_RGBX_8888,               GBM_FORMAT_RGBX8888              },

  {HAL_PIXEL_FORMAT_BLOB,                    GBM_FORMAT_BLOB                  },
//   {HAL_PIXEL_FORMAT_RAW8,                    0},
  {HAL_PIXEL_FORMAT_RAW10,                   GBM_FORMAT_RAW10                 },
  {HAL_PIXEL_FORMAT_RAW12,                   GBM_FORMAT_RAW12                 },
  {HAL_PIXEL_FORMAT_RAW16,                   GBM_FORMAT_RAW16                 },

  {HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED,  GBM_FORMAT_IMPLEMENTATION_DEFINED},

//  {HAL_PIXEL_FORMAT_NV12_ENCODEABLE,         GBM_FORMAT_NV12_ENCODEABLE},
//  {HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS,      GBM_FORMAT_YCbCr_420_SP_VENUS},
//  {HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC, GBM_FORMAT_YCbCr_420_SP_VENUS_UBWC},

  {HAL_PIXEL_FORMAT_YCBCR_420_888,           GBM_FORMAT_YCbCr_420_888         },
  {HAL_PIXEL_FORMAT_YCBCR_422_SP,            GBM_FORMAT_YCbCr_422_SP          },
  {HAL_PIXEL_FORMAT_YCBCR_422_I,             0                                },
  {HAL_PIXEL_FORMAT_YCRCB_420_SP,            GBM_FORMAT_YCrCb_420_SP          },
  {HAL_PIXEL_FORMAT_YV12,                    0                                },
  {HAL_PIXEL_FORMAT_YCBCR_422_888,           0                                },
//  {HAL_PIXEL_FORMAT_NV21_ZSL,                GBM_FORMAT_NV21_ZSL},
};


// typedef struct {
//     uint32_t gbm_format;
//     std::string str;
// } gbm_format_to_str_pair;

static const std::unordered_map<uint32_t, const char*> gbm_format_str_map = {
       {GBM_FORMAT_BGRA8888              , "GBM_FORMAT_BGRA8888"},
       {GBM_FORMAT_RGB565                , "GBM_FORMAT_RGB565"},
       {GBM_FORMAT_RGB888                , "GBM_FORMAT_RGB888"},
       {GBM_FORMAT_RGBA1010102           , "GBM_FORMAT_RGBA1010102"},
       {GBM_FORMAT_RGBA8888              , "GBM_FORMAT_RGBA8888"},
       {GBM_FORMAT_RGBX8888              , "GBM_FORMAT_RGBX8888"},
       {GBM_FORMAT_BLOB                  , "GBM_FORMAT_BLOB"},
       {GBM_FORMAT_RAW10                 , "GBM_FORMAT_RAW10"},
       {GBM_FORMAT_RAW12                 , "GBM_FORMAT_RAW12"},
       {GBM_FORMAT_RAW16                 , "GBM_FORMAT_RAW16"},
       {GBM_FORMAT_IMPLEMENTATION_DEFINED, "GBM_FORMAT_IMPLEMENTATION_DEFINED"},
       {GBM_FORMAT_NV12_ENCODEABLE       , "GBM_FORMAT_NV12_ENCODEABLE"},
       {GBM_FORMAT_YCbCr_420_SP_VENUS    , "GBM_FORMAT_YCbCr_420_SP_VENUS"},
       {GBM_FORMAT_YCbCr_420_888         , "GBM_FORMAT_YCbCr_420_888"},
       {GBM_FORMAT_YCbCr_422_SP          , "GBM_FORMAT_YCbCr_422_SP"},
       {GBM_FORMAT_YCrCb_420_SP          , "GBM_FORMAT_YCrCb_420_SP"},
};

// #define GBM_FORMAT_STR_MAP_SIZE  (sizeof(gbm_format_str_map)  / sizeof(gbm_format_to_str_pair))
// #define HAL_FORMAT_FLAG_MAP_SIZE (sizeof(hal_format_flag_map) / sizeof(hal_to_gbm_format_pair))

static uint32_t hal3_format_to_gbm(uint32_t hal3_format)
{
    for (auto const& pair : hal_format_flag_map) {
        if (hal3_format == pair.first) {
            return pair.second;
        }
    }

    printf("Got unrecognized hal3 format in %s, defaulting to GBM_FORMAT_IMPLEMENTATION_DEFINED\n", __FUNCTION__);
    return GBM_FORMAT_IMPLEMENTATION_DEFINED;
}


static void* getUVStartFromFmt(uint8_t* block_start, uint32_t aligned_width, uint32_t aligned_height,
        uint32_t fmt) {
    switch(fmt) {
        case HAL_PIXEL_FORMAT_YCBCR_420_888:
            return static_cast<void*>(block_start + (aligned_width * aligned_height));

        case HAL_PIXEL_FORMAT_BLOB:
        case HAL_PIXEL_FORMAT_RAW10:
        case HAL_PIXEL_FORMAT_RAW12:
        case HAL_PIXEL_FORMAT_RAW16:
        case HAL_PIXEL_FORMAT_RGBA_8888:
            return nullptr;

        default:
            printf("Got unsupported format in %s, returning nullptr\n", __FUNCTION__);
            return nullptr;
    }
}


int allocate_one_buffer(mpa_ion_buf_t* buf, int width, int height, uint32_t hal3_format, uint32_t gralloc_flags)
{
    uint32_t gbm_flags  = gralloc_flags_to_gbm(gralloc_flags);
    uint32_t gbm_format = hal3_format_to_gbm(hal3_format);

    //workaround for 12bit RAW image mapping to gpu not working correctly, causing gpu page fault
    uint32_t alloc_height = height;
    if (gbm_format == GBM_FORMAT_RAW12){
        alloc_height = height+1;
    }
    //workaround end

    struct gbm_bo* bo = gbm_bo_create(g_gbm_dev, width, alloc_height, gbm_format, gbm_flags);
    if(bo == NULL){
        printf("Could not create GBM buffer object\n");
        return -EINVAL;
    }

    uint32_t stride;
    // workaround/HACK: for ov7251, the driver always returns RAW8 data, but we need to request RAW10 from hal3
    // HAL3 will write RAW8 data into the given buffer as though there was no stride, therefore we will ignore stride
    // and overwrite it with width in that specific case
    if (hal3_format == HAL_PIXEL_FORMAT_RAW10 && width == 640 && height == 480) {
        stride = width;
    }
    else {
        stride = gbm_bo_get_stride(bo);
    }

    uint32_t aligned_h;
    int rval = gbm_perform(GBM_PERFORM_GET_BO_ALIGNED_HEIGHT, bo, &aligned_h);
    if (rval != 0) {
        printf("Could not get buffer stride: error=%d\n", rval);
        return -EINVAL;
    }

    // Note to reader: gbm_priv also contains the action
    // GBM_PERFORMGET_BO_ALIGNED_WIDTH. However, qmmf just uses stride, so we
    // will emulate that behavior here. Experimentally, it seems as though
    // stride >= aligned_width always. leaving the following for reference.
    uint32_t aligned_w = 0;
    rval = gbm_perform(GBM_PERFORM_GET_BO_ALIGNED_WIDTH, bo, &aligned_w);

    size_t size;
    rval = gbm_perform(GBM_PERFORM_GET_BO_SIZE, bo, &size);
    if (rval != 0) {
        printf("Could not get buffer size: error=%d\n", rval);
        return -EINVAL;
    }

    int fd = gbm_bo_get_fd(bo);
    if (fd < 0) {
        int errsv = errno;
        printf("Unable to get fd from bo: %s (errno=%d)\n", strerror(errsv), errsv);
        return -EINVAL;
    }

    // map the allocated buffer into our address space
    void* buf_addr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (buf_addr == NULL) {
        int errsv = errno;
        printf("Unable to mmap buffer object: %s (errno=%d)\n", strerror(errsv), errsv);
        return -EINVAL;
    }

    // char* format_str;
    // for (int i = 0; i < GBM_FORMAT_STR_MAP_SIZE; i++) {
    //     if (gbm_format == gbm_format_str_map[i].gbm_format) {
    //         format_str = gbm_format_str_map[i].str;
    //     }
    // }

    // printf("Allocated BO with width=%u height=%u stride=%u aligned_w=%u aligned_h=%u size=%u flags=0x%x format=%s fd=%u vaddr=%p\n",
    //         width, height, stride, aligned_w, aligned_h, size, gralloc_flags, format_str, fd, buf_addr);

    buf->vaddress = buf_addr;
    buf->uvHead   = getUVStartFromFmt((uint8_t*) buf_addr, stride, aligned_h, hal3_format);
    buf->size     = size;
    buf->width    = width;
    buf->height   = height;
    buf->stride   = stride;
    buf->slice    = aligned_h;
    buf->source   = BUFFER_SOURCE_LOCAL;
    buf->bo       = bo;
    buf->fd       = fd;
    buf->format = gbm_bo_get_format(bo);
    buf->consumerFlags = gbm_flags;

    // construct a native_handle_t from our bo
    private_handle_t* priv_handle = new private_handle_t(fd, size, gralloc_flags);
    // private_handle_t is a superclass of native_handle_t which initializes the
    // trailing VLA correctly
    buf->handle       = static_cast<native_handle_t*>(priv_handle);

    return 0;
}

int delete_one_buffer(mpa_ion_buf_t* buf)
{
    munmap(buf->vaddress, buf->size);
    gbm_bo_destroy(buf->bo);
    close(buf->fd);

    return 0;
}
