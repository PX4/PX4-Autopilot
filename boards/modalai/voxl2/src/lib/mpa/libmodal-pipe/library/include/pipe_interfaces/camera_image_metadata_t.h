#ifndef CAMERA_IMAGE_METADATA_T_H
#define CAMERA_IMAGE_METADATA_T_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

////////////////////////////////////////////////////////////////////////////////
// Camera and Images
////////////////////////////////////////////////////////////////////////////////

/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream.
 */
#include "magic_number.h"
#ifndef CAMERA_MAGIC_NUMBER
#error "CAMERA_MAGIC_NUMBER not defined!"
#endif

// Common image formats for use by a camera server. This is not an exhaustive
// list and custom values not included here can be used as long as the server
// and client both agree on the image format.
#define IMAGE_FORMAT_RAW8 0 // 8-bit gray image, used for tracking camera
#define IMAGE_FORMAT_NV12 1
#define IMAGE_FORMAT_STEREO_RAW8 2 // 8-bit gray, two sequential images
#define IMAGE_FORMAT_H264 3
#define IMAGE_FORMAT_H265 4
#define IMAGE_FORMAT_RAW16 5 // 16-bit image, for disparity maps or HDR gray images
#define IMAGE_FORMAT_NV21 6  // Android NV21 format from hal3
#define IMAGE_FORMAT_JPG 7
#define IMAGE_FORMAT_YUV422 8 // Standard YUV422 with YUYV mapping scheme
#define IMAGE_FORMAT_YUV420 9
#define IMAGE_FORMAT_RGB 10         // 24-bits per pixel
#define IMAGE_FORMAT_FLOAT32 11     // 32-bit float per pixel, for depth map
#define IMAGE_FORMAT_STEREO_NV21 12 // Android NV21 format from hal3, two sequential images
#define IMAGE_FORMAT_STEREO_RGB 13  // 24-bits per pixel, two sequential images
#define IMAGE_FORMAT_YUV422_UYVY 14 // YUV422 with alternate UYVY mapping scheme
#define IMAGE_FORMAT_STEREO_NV12 15
#define IMAGE_FORMAT_RAW10 16 // 10-bit image, in MIPI 10 bit format
#define IMAGE_FORMAT_RAW12 17 // 12-bit image, in MIPI 12 bit format
#define IMAGE_FORMAT_RAW14 18 // 14-bit image, in MIPI 14 bit format
    // NOTE: when updating this list, also update pipe_image_format_to_string() in
    // src/interfaces.c to print the new name

    /**
     * The metadata for the camera image. One of these is sent before every frame
     */
    typedef struct camera_image_metadata_t
    {
        uint32_t magic_number; ///< set to CAMERA_MAGIC_NUMBER
        int64_t timestamp_ns;  ///< timestamp in apps-proc clock-monotonic of beginning of exposure
        int32_t frame_id;      ///< iterator from 0++ starting from first frame when server starts on boot
        int16_t width;         ///< image width in pixels
        int16_t height;        ///< image height in bytes
        int32_t size_bytes;    ///< size of the image, for stereo this is the size of both L&R together
        int32_t stride;        ///< bytes per row
        int32_t exposure_ns;   ///< exposure in nanoseconds
        int16_t gain;          ///< ISO gain (100, 200, 400, etc..)
        int16_t format;        ///< raw8, nv12, etc
        int16_t framerate;     ///< expected framerate hz
        int16_t reserved;      ///< extra reserved bytes
    } __attribute__((packed)) camera_image_metadata_t;

    /**
     * @brief      convert an image format id number to string
     *
     * For example IMAGE_FORMAT_RAW8 will return the string "RAW8"
     *
     * @param[in]  i     image format id, e.g. IMAGE_FORMAT_RAW8
     *
     * @return     const char8 string of the format
     */
    const char *pipe_image_format_to_string(int i);

#ifdef __cplusplus
}
#endif

#endif
