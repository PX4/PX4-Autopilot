#ifndef POINT_CLOUD_METADATA_T_H
#define POINT_CLOUD_METADATA_T_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////
// Point Cloud
////////////////////////////////////////////////////////////////////////////////

/**
 * Points clouds are sent similar to camera frames with a metadata struct
 * followed by a sequence of points in float[3] format whose length is
 * determined by the metadata struct.
 *
 * Since each point is 3 floats in XYZ, the payload length will be 12*n_points
 * long.
 *
 * Use the point-cloud helper when initializing the modal pipe client to let
 * the helper pick apart the metadata and read in the right amount of data.
 */

/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream.
 */
#include "magic_number.h"
#ifndef POINT_CLOUD_MAGIC_NUMBER
#error "POINT_CLOUD_MAGIC_NUMBER not defined!"
#endif

#define POINT_CLOUD_FORMAT_FLOAT_XYZ 0     // 12-bytes per point, float, XYZ
#define POINT_CLOUD_FORMAT_FLOAT_XYZC 1    // 16-bytes per point, float, XYZ followed by confidence float
#define POINT_CLOUD_FORMAT_FLOAT_XYZRGB 2  // 15-bytes per point, float, XYZ followed by 8-bit RGB
#define POINT_CLOUD_FORMAT_FLOAT_XYZCRGB 3 // 19-bytes per point, float, XYZ followed by confidence float and 8-bit RGB
#define POINT_CLOUD_FORMAT_FLOAT_XY 4      //  8-bytes per point, float, XY
#define POINT_CLOUD_FORMAT_FLOAT_XYC 5     // 12-bytes per point, float, XY  followed by confidence float

    typedef struct point_cloud_metadata_t
    {
        uint32_t magic_number; ///< Unique 32-bit number used to signal the beginning of a packet while parsing a data stream.
        int64_t timestamp_ns;  ///< timestamp in nanoseconds
        uint32_t n_points;     ///< number of points following the metadata struct
        uint32_t format;       ///< point cloud format
        uint32_t id;           ///< optional id, meaning is denoted by individual servers
        char server_name[32];  ///< optional server name, to specify the source of this pointcloud
        uint32_t reserved;     ///< reserved bytes
    } __attribute__((packed)) point_cloud_metadata_t;

    const char *pipe_point_cloud_format_to_string(int i);

    /**
     * @brief      return the expected number of bytes of point cloud data that
     *             should follow a metadata struct in the stream.
     *
     *             different point cloud formats take up different numbers of bytes.
     *             For a metadata struct containing the number of bytes and format,
     *             this returns how many bytes are to be expects.
     *
     * @param[in]  meta  The metadata struct
     *
     * @return     expected number of bytes or -1 on error or invalid metadata
     *             struct.
     */
    int pipe_point_cloud_meta_to_size_bytes(point_cloud_metadata_t meta);

#ifdef __cplusplus
}
#endif

#endif