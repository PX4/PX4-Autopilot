/****************************************************************************
*
*   Copyright (c) 2015 PX4 Development Team. All rights reserved.
*      Author: Pavel Kirienko <pavel.kirienko@gmail.com>
*              David Sidrane <david_s5@usa.net>
*
****************************************************************************/

#ifndef UAVCAN_POSIX_FIRMWARE_COMMON_HPP_INCLUDED
#define UAVCAN_POSIX_FIRMWARE_COMMON_HPP_INCLUDED

#include <cstdint>
#include <cstdbool>
#include <cstdio>
#include <cstring>
#include <cfcntl>
#include <cerrno>

#include <uavcan/protocol/file/Path.hpp>
#include "firmware_path.hpp"

namespace uavcan_posix
{
/**
 * Firmware file validation logic.
 */
class FirmwareCommon
{
public:

    typedef struct app_descriptor_t
    {
        uint8_t signature[sizeof(uint64_t)];
        uint64_t image_crc;
        uint32_t image_size;
        uint32_t vcs_commit;
        uint8_t major_version;
        uint8_t minor_version;
        uint8_t reserved[6];
    } app_descriptor_t;

    app_descriptor_t descriptor;

    int getFileInfo(const char *path)
    {
        enum { MaxChunk  = (512 / sizeof(uint64_t)) };
        int rv = -ENOENT;
        uint64_t chunk[MaxChunk];
        int fd = open(path, O_RDONLY);

        if (fd >= 0)
        {
            app_descriptor_t *pdescriptor = 0;

            while(!pdescriptor)
            {
                int len = read(fd, chunk, sizeof(chunk));

                if (len == 0)
                {
                    break;
                }

                if (len < 0)
                {
                    rv = -errno;
                    goto out_close;
                }

                uint64_t *p = &chunk[0];

                do
                {
                    if (*p == sig.ull)
                    {
                        pdescriptor = (app_descriptor_t *)p;
                        descriptor = *pdescriptor;
                        rv = 0;
                        break;
                    }
                }
                while(p++ <= &chunk[MaxChunk - (sizeof(app_descriptor_t) / sizeof(chunk[0]))]);
            }
        out_close:
            close(fd);
        }
        return rv;
    }


private:

#define APP_DESCRIPTOR_SIGNATURE_ID 'A', 'P', 'D', 'e', 's', 'c'
#define APP_DESCRIPTOR_SIGNATURE_REV '0', '0'
#define APP_DESCRIPTOR_SIGNATURE APP_DESCRIPTOR_SIGNATURE_ID, APP_DESCRIPTOR_SIGNATURE_REV

    union
    {
        uint64_t ull;
        char text[sizeof(uint64_t)];
    } sig = {
        .text = {APP_DESCRIPTOR_SIGNATURE}
    };

};
}

#endif // Include guard
