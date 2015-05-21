/****************************************************************************
*
*   Copyright (c) 2015 PX4 Development Team. All rights reserved.
*      Author: Pavel Kirienko <pavel.kirienko@gmail.com>
*              David Sidrane <david_s5@usa.net>
*
****************************************************************************/

#ifndef UAVCAN_POSIX_FIRMWARE_PATH_HPP_INCLUDED
#define UAVCAN_POSIX_FIRMWARE_PATH_HPP_INCLUDED

#include <cstring>

#include <uavcan/protocol/file/Path.hpp>

namespace uavcan_posix
{
/**
 * Firmware Path Management.
 */
class FirmwarePath
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

    BasePathString base_path_;
    BasePathString cache_path_;

    /**
     * This type is used internally for the full path to file
     */
    typedef uavcan::MakeString<MaxPathLength>::Type PathString;

    static char getPathSeparator() { return static_cast<char>(uavcan::protocol::file::Path::SEPARATOR); }


    BasePathString& getFirmwareBasePath()
    {
        return base_path_;
    }

    void setFirmwareBasePath(const char * path)
    {
        base_path_ = path;
    }

    BasePathString& getFirmwareCachePath()
    {
        return cache_path_;
    }

    void setFirmwareCachePath(const char *path)
    {
        cache_path_ = path;

    }


    static void addSlash(BasePathString& path)
    {

        if (path.back() != getPathSeparator())
        {
            path.push_back(getPathSeparator());
        }
    }

    static void removeSlash(BasePathString& path)
    {

        if (path.back() == getPathSeparator())
        {
            path.pop_back();
        }

    }

    /**
     * Creates the Directories were the files will be stored
     */

    /* This is directory structure is in support of a workaround
     * for the issues that FirmwareFilePath is 40
     *
     *  It creates a path structure:
     *
     *
     *    +---(base_path)
     *        +-c                           <----------- Files are cashed here.
     */

    int CreateFwPaths(const char *base_path)
    {
        using namespace std;
        int rv = -uavcan::ErrInvalidParam;

        if (base_path)
        {

            int len = strlen(base_path);

            if (len > 0 && len < base_path_.MaxSize)
            {

                setFirmwareBasePath(base_path);
                removeSlash(getFirmwareBasePath());
                const char *path = getFirmwareBasePath().c_str();

                setFirmwareCachePath(path);
                addSlash(cache_path_);
                cache_path_ += get_cache_dir_();

                rv = 0;
                struct stat sb;
                if (stat(path, &sb) != 0 || !S_ISDIR(sb.st_mode))
                {
                    rv = mkdir(path, S_IRWXU | S_IRWXG | S_IRWXO);
                }

                path = getFirmwareCachePath().c_str();

                if (rv == 0  && (stat(path, &sb) != 0 || !S_ISDIR(sb.st_mode)))
                {
                    rv = mkdir(path, S_IRWXU | S_IRWXG | S_IRWXO);
                }

                addSlash(getFirmwareBasePath());
                addSlash(getFirmwareCachePath());


                if (rv >= 0 )
                {
                    if ((getFirmwareCachePath().size() + uavcan::protocol::file::Path::FieldTypes::path::MaxSize) >
                        MaxPathLength)
                    {
                        rv = -uavcan::ErrInvalidConfiguration;
                    }
                }
            }
        }
        return rv;
    }

    int init(const char *path)
    {
        return CreateFwPaths(path);
    }

};

}

#endif // Include guard
