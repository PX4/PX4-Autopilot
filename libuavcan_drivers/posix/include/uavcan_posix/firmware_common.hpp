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

namespace uavcan_posix
{
/**
 * Firmware version checking logic.
 * Refer to @ref FirmwareUpdateTrigger for details.
 */
class FirmwareCommon
{
    /* The folder where the files will be copied and Read from */

    static const char* get_cache_dir_()
    {
        return "c";
    }

public:

    enum { MaxBasePathLength = 128 };

    /**
     * This type is used for the base path
     */
    typedef uavcan::MakeString<MaxBasePathLength>::Type BasePathString;

    /**
     * Maximum length of full path including / the file name
     */
    enum { MaxPathLength =  uavcan::protocol::file::Path::FieldTypes::path::MaxSize + MaxBasePathLength };

    /**
     * This type is used internally for the full path to file
     */
    typedef uavcan::MakeString<MaxPathLength>::Type PathString;

    static char getPathSeparator() { return static_cast<char>(uavcan::protocol::file::Path::SEPARATOR); }

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

    /**
     * This method Is used to get the app_descriptor_t from a firmware image
     *
     * @param path                      Complete path to the file
     *
     * @param add_separator             Complete path with trailing separator
     *
     *
     * @return                          0 if the app_descriptor_t was found and out_descriptor
     *                                  has been updated.
     *                                  Otherwise -errno is returned
     *                                  -ENOENT Signature was not found.
     */

    static PathString getFirmwareCachePath(const PathString& p, bool add_separator = true)
    {
        PathString r;
        r = p;
        r.push_back(getPathSeparator());
        r += get_cache_dir_();
        if (add_separator) {
            r.push_back(getPathSeparator());
        }
        return r;
    }


    static BasePathString getFirmwareCachePath(const BasePathString& p, bool add_separator = true)
    {
        BasePathString r;
        r = p;
        r.push_back(getPathSeparator());
        r += get_cache_dir_();
        if (add_separator) {
            r.push_back(getPathSeparator());
        }
        return r;
    }

    int getFileInfo(PathString& path)
    {
        enum { MaxChunk  = (512 / sizeof(uint64_t)) };
        int rv = -ENOENT;
        uint64_t chunk[MaxChunk];
        int fd = open(path.c_str(), O_RDONLY);

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


    /**
     * Creates the Directories were the files will be stored
     */

    static int create_fw_paths(FirmwareCommon::BasePathString& path)
    {
        using namespace std;

        int rv = -uavcan::ErrInvalidParam;

        if (path.size() > 0)
        {
            if (path.back() == FirmwareCommon::getPathSeparator())
            {
                path.pop_back();
            }

            rv = 0;
            struct stat sb;
            if (stat(path.c_str(), &sb) != 0 || !S_ISDIR(sb.st_mode))
            {
                rv = mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
            }

            PathString cache = getFirmwareCachePath(PathString(path.c_str()), false);
            if (rv >= 0  && (stat(cache.c_str(), &sb) != 0 || !S_ISDIR(sb.st_mode)))
            {
                rv = mkdir(cache.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
            }

            if (rv >= 0 )
            {
                path.push_back(FirmwareCommon::getPathSeparator());
                if ((path.size() + uavcan::protocol::file::Path::FieldTypes::path::MaxSize) >
                    FirmwareCommon::MaxPathLength)
                {
                    rv = -uavcan::ErrInvalidConfiguration;
                }
            }
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
