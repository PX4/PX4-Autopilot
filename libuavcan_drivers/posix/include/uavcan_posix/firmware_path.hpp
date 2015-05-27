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
 * This class manages the sole copies of the cache path and the base path of the fw
 * This prevent having to have large stack allocations to manipulate the 2 paths.
 * It ensures that the paths have the proper slash as endings for concatenation.
 * It also provides the creation of the folders  and encapsulates the paths to
 */
class FirmwarePath
{
public:
    enum { MaxBasePathLength = 128 };

    /**
     * This type is used for the base path
     */
    typedef uavcan::MakeString<MaxBasePathLength>::Type BasePathString;

    /**
     * Maximum length of full path including / the file name
     */
    enum { MaxPathLength = uavcan::protocol::file::Path::FieldTypes::path::MaxSize + MaxBasePathLength };

    static char getPathSeparator()
    {
        return static_cast<char>(uavcan::protocol::file::Path::SEPARATOR);
    }

    /**
     * This type is used internally for the full path to file
     */
    typedef uavcan::MakeString<MaxPathLength>::Type PathString;



private:

    BasePathString base_path_;
    BasePathString cache_path_;

    /**
     * The folder where the files will be copied and read from
     */
    static const char* getCacheDir() { return "c"; }

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

    void setFirmwareBasePath(const char* path)
    {
        base_path_ = path;
    }

    void setFirmwareCachePath(const char* path)
    {
        cache_path_ = path;
    }

public:
    const BasePathString& getFirmwareBasePath() const { return base_path_; }

    const BasePathString& getFirmwareCachePath() const { return cache_path_; }

    /**
     * Creates the Directories were the files will be stored
     *
     * This is directory structure is in support of a workaround
     * for the issues that FirmwareFilePath is 40
     *
     *  It creates a path structure:
     *    +---(base_path)
     *        +-c                           <----------- Files are cached here.
     */
    int createFwPaths(const char* base_path)
    {
        using namespace std;
        int rv = -uavcan::ErrInvalidParam;

        if (base_path)
        {
            const int len = strlen(base_path);

            if (len > 0 && len < base_path_.MaxSize)
            {
                setFirmwareBasePath(base_path);
                removeSlash(base_path_);
                const char* path = getFirmwareBasePath().c_str();

                setFirmwareCachePath(path);
                addSlash(cache_path_);
                cache_path_ += getCacheDir();

                rv = 0;
                struct stat sb;
                if (stat(path, &sb) != 0 || !S_ISDIR(sb.st_mode))
                {
                    rv = mkdir(path, S_IRWXU | S_IRWXG | S_IRWXO);
                }

                path = getFirmwareCachePath().c_str();

                if (rv == 0 && (stat(path, &sb) != 0 || !S_ISDIR(sb.st_mode)))
                {
                    rv = mkdir(path, S_IRWXU | S_IRWXG | S_IRWXO);
                }

                addSlash(base_path_);
                addSlash(cache_path_);

                if (rv >= 0)
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
};

}

#endif // Include guard
