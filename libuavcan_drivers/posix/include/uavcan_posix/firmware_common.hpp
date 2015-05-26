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
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <cerrno>

#include <uavcan/protocol/file/Path.hpp>
#include "firmware_path.hpp"

namespace uavcan_posix
{
/**
 * Firmware file validation logic.
 * TODO Rename - FirmwareCommon is a bad name as it doesn't reflect the purpose of this class.
 * TODO Returning value via member variable is not a proper way of doing it. Probably the whole class should
 *      be replaced with a static function.
 */
class FirmwareCommon
{
    static uavcan::uint64_t getAppDescriptorSignature()
    {
        uavcan::uint64_t ull = 0;
        std::memcpy(&ull, "APDesc00", 8);
        return ull;
    }

public:
    struct AppDescriptor
    {
        uint8_t signature[sizeof(uavcan::uint64_t)];
        uint64_t image_crc;
        uint32_t image_size;
        uint32_t vcs_commit;
        uint8_t major_version;
        uint8_t minor_version;
        uint8_t reserved[6];
    };

    AppDescriptor descriptor;

    int getFileInfo(const char* path)
    {
        using namespace std;

        const unsigned MaxChunk = 512 / sizeof(uint64_t);

        const uint64_t signature = getAppDescriptorSignature();

        int rv = -ENOENT;
        uint64_t chunk[MaxChunk];
        int fd = open(path, O_RDONLY);

        if (fd >= 0)
        {
            AppDescriptor* pdescriptor = NULL;

            while (pdescriptor == NULL)
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

                uint64_t* p = &chunk[0];

                do
                {
                    if (*p == signature)
                    {
                        pdescriptor = (AppDescriptor*) p;
                        descriptor = *pdescriptor;
                        rv = 0;
                        break;
                    }
                }
                while (p++ <= &chunk[MaxChunk - (sizeof(AppDescriptor) / sizeof(chunk[0]))]);
            }

        out_close:
            (void)close(fd);
        }
        return rv;
    }
};

}

#endif // Include guard
