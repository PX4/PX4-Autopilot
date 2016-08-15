/****************************************************************************
*
*   Copyright (c) 2015 PX4 Development Team. All rights reserved.
*      Author: Pavel Kirienko <pavel.kirienko@gmail.com>
*              David Sidrane <david_s5@usa.net>
*
****************************************************************************/

#ifndef UAVCAN_POSIX_FIRMWARE_VERSION_CHECKER_HPP_INCLUDED
#define UAVCAN_POSIX_FIRMWARE_VERSION_CHECKER_HPP_INCLUDED

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <cerrno>
#include <dirent.h>

#include <uavcan/protocol/firmware_update_trigger.hpp>

// TODO Get rid of the macro
#if !defined(DIRENT_ISFILE) && defined(DT_REG)
# define DIRENT_ISFILE(dtype)  ((dtype) == DT_REG)
#endif

namespace uavcan_posix
{
/**
 * Firmware version checking logic.
 * Refer to @ref FirmwareUpdateTrigger for details.
 */
class FirmwareVersionChecker : public uavcan::IFirmwareVersionChecker
{
    enum { FilePermissions = 438 }; ///< 0o666

    enum { MaxBasePathLength = 128 };

    /**
     * This type is used for the base path
     */
    typedef uavcan::MakeString<MaxBasePathLength>::Type BasePathString;

    /**
     * Maximum length of full path including / the file name
     */
    enum { MaxPathLength = uavcan::protocol::file::Path::FieldTypes::path::MaxSize + MaxBasePathLength };

    /**
     * This type is used internally for the full path to file
     */
    typedef uavcan::MakeString<MaxPathLength>::Type PathString;

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

    int copyIfNot(const char* srcpath, const char* destpath)
    {
        using namespace std;

        // Does the file exist
        int rv = 0;
        int dfd = open(destpath, O_RDONLY, 0);

        if (dfd >= 0)
        {
            // Close it and exit 0
            (void)close(dfd);
        }
        else
        {
            uint8_t buffer[512];

            dfd = open(destpath, O_WRONLY | O_CREAT, FilePermissions);
            if (dfd < 0)
            {
                rv = -errno;
            }
            else
            {
                int sfd = open(srcpath, O_RDONLY, 0);
                if (sfd < 0)
                {
                    rv = -errno;
                }
                else
                {
                    ssize_t size = 0;
                    do
                    {
                        size = ::read(sfd, buffer, sizeof(buffer));
                        if (size != 0)
                        {
                            if (size < 0)
                            {
                                rv = -errno;
                            }
                            else
                            {
                                rv = 0;
                                ssize_t remaining = size;
                                ssize_t total_written = 0;
                                ssize_t written = 0;
                                do
                                {
                                    written = write(dfd, &buffer[total_written], remaining);
                                    if (written < 0)
                                    {
                                        rv = -errno;
                                    }
                                    else
                                    {
                                        total_written += written;
                                        remaining -=  written;
                                    }
                                }
                                while (written > 0 && remaining > 0);
                            }
                        }
                    }
                    while (rv == 0 && size != 0);

                    (void)close(sfd);
                }
                (void)close(dfd);
            }
        }
        return rv;
    }

    struct AppDescriptor
    {
        uavcan::uint8_t signature[sizeof(uavcan::uint64_t)];
        uavcan::uint64_t image_crc;
        uavcan::uint32_t image_size;
        uavcan::uint32_t vcs_commit;
        uavcan::uint8_t major_version;
        uavcan::uint8_t minor_version;
        uavcan::uint8_t reserved[6];
    };

    static int getFileInfo(const char* path, AppDescriptor& descriptor)
    {
        using namespace std;

        const unsigned MaxChunk = 512 / sizeof(uint64_t);

        uint64_t signature = 0;
        std::memcpy(&signature, "APDesc00", 8);

        int rv = -ENOENT;
        uint64_t chunk[MaxChunk];
        int fd = open(path, O_RDONLY);

        if (fd >= 0)
        {
            AppDescriptor* pdescriptor = UAVCAN_NULLPTR;

            while (pdescriptor == UAVCAN_NULLPTR)
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
                        pdescriptor = reinterpret_cast<AppDescriptor*>(p); // FIXME TODO This breaks strict aliasing
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

protected:
    /**
     * This method will be invoked when the class obtains a response to GetNodeInfo request.
     *
     * @param node_id                   Node ID that this GetNodeInfo response was received from.
     *
     * @param node_info                 Actual node info structure; refer to uavcan.protocol.GetNodeInfo for details.
     *
     * @param out_firmware_file_path    The implementation should return the firmware image path via this argument.
     *                                  Note that this path must be reachable via uavcan.protocol.file.Read service.
     *                                  Refer to @ref FileServer and @ref BasicFileServer for details.
     *
     * @return                          True - the class will begin sending update requests.
     *                                  False - the node will be ignored, no request will be sent.
     */
    virtual bool shouldRequestFirmwareUpdate(uavcan::NodeID,
                                             const uavcan::protocol::GetNodeInfo::Response& node_info,
                                             FirmwareFilePath& out_firmware_file_path)
    {
        using namespace std;

        /* This is a work  around for two issues.
         *  1) FirmwareFilePath is 40
         *  2) OK using is using 32 for max file names.
         *
         *  So for the file:
         *    org.pixhawk.px4cannode-v1-0.1.59efc137.uavcan.bin
         *    +---fw
         *        +-c                           <----------- Files are cashed here.
         *          +--- px4cannode-v1.59efc137.bin <------  A Firmware file
         *        +---org.pixhawk.px4cannode-v1 <---------- node_info.name
         *        +---1.0 <-------------------------------- node_info.name's hardware_version.major,minor
         *            + - px4cannode-v1.59efc137.bin  <---- A well known file must match the name
         *                                                   in the root fw folder, so if it does not exist
         *                                                   it is copied up MUST BE < 32 Characters
         */
        bool rv = false;

        char fname_root[MaxBasePathLength + 1];
        int n = snprintf(fname_root, sizeof(fname_root), "%s%s/%d.%d",
                         getFirmwareBasePath().c_str(),
                         node_info.name.c_str(),
                         node_info.hardware_version.major,
                         node_info.hardware_version.minor);

        if (n > 0 && n < (int)sizeof(fname_root) - 2)
        {
            DIR* const fwdir = opendir(fname_root);

            fname_root[n++] = getPathSeparator();
            fname_root[n++] = '\0';

            if (fwdir != UAVCAN_NULLPTR)
            {
                struct dirent* pfile = UAVCAN_NULLPTR;
                while ((pfile = readdir(fwdir)) != UAVCAN_NULLPTR)
                {
                    if (DIRENT_ISFILE(pfile->d_type))
                    {
                        // Open any bin file in there.
                        if (strstr(pfile->d_name, ".bin") != UAVCAN_NULLPTR)
                        {
                            PathString full_src_path = fname_root;
                            full_src_path += pfile->d_name;

                            PathString full_dst_path = getFirmwareCachePath().c_str();
                            full_dst_path += pfile->d_name;

                            // ease the burden on the user
                            int cr = copyIfNot(full_src_path.c_str(), full_dst_path.c_str());

                            // We have a file, is it a valid image
                            AppDescriptor descriptor;

                            std::memset(&descriptor, 0, sizeof(descriptor));

                            if (cr == 0 && getFileInfo(full_dst_path.c_str(), descriptor) == 0)
                            {
                                volatile AppDescriptor descriptorC = descriptor;
                                descriptorC.reserved[1]++;

                                if (node_info.software_version.image_crc == 0 ||
                                    (node_info.software_version.major == 0 && node_info.software_version.minor == 0) ||
                                    descriptor.image_crc != node_info.software_version.image_crc)
                                {
                                    rv = true;
                                    out_firmware_file_path = pfile->d_name;
                                }
                                break;
                            }
                        }
                    }
                }
                (void)closedir(fwdir);
            }
        }
        return rv;
    }

    /**
     * This method will be invoked when a node responds to the update request with an error. If the request simply
     * times out, this method will not be invoked.
     * Note that if by the time of arrival of the response the node is already removed, this method will not be called.
     *
     * SPECIAL CASE: If the node responds with ERROR_IN_PROGRESS, the class will assume that further requesting
     *               is not needed anymore. This method will not be invoked.
     *
     * @param node_id                   Node ID that returned this error.
     *
     * @param error_response            Contents of the error response. It contains error code and text.
     *
     * @param out_firmware_file_path    New firmware path if a retry is needed. Note that this argument will be
     *                                  initialized with old path, so if the same path needs to be reused, this
     *                                  argument should be left unchanged.
     *
     * @return                          True - the class will continue sending update requests with new firmware path.
     *                                  False - the node will be forgotten, new requests will not be sent.
     */
    virtual bool shouldRetryFirmwareUpdate(uavcan::NodeID,
                                           const uavcan::protocol::file::BeginFirmwareUpdate::Response&,
                                           FirmwareFilePath&)
    {
        // TODO: Limit the number of attempts per node
        return true;
    }

public:
    const BasePathString& getFirmwareBasePath() const { return base_path_; }

    const BasePathString& getFirmwareCachePath() const { return cache_path_; }

    static char getPathSeparator()
    {
        return static_cast<char>(uavcan::protocol::file::Path::SEPARATOR);
    }

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

    const char* getFirmwarePath() const
    {
        return getFirmwareCachePath().c_str();
    }
};
}

#endif // Include guard
