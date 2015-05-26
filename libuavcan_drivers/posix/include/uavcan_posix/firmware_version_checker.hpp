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

#include "firmware_common.hpp"

namespace uavcan_posix
{
/**
 * Firmware version checking logic.
 * Refer to @ref FirmwareUpdateTrigger for details.
 */
class FirmwareVersionChecker : public uavcan::IFirmwareVersionChecker
{
    enum { FilePermissions = 438 }; ///< 0o666

    FirmwarePath* paths_;

    int copyIfNot(const char* srcpath, const char* destpath)
    {
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
                    ssize_t size;
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

                                if (size != write(dfd, buffer, size))
                                {
                                    rv = -errno;
                                }
                            }
                        }
                    }
                    while (rv == 0 && size);

                    (void)close(sfd);
                }
                (void)close(dfd);
            }
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
    virtual bool shouldRequestFirmwareUpdate(uavcan::NodeID node_id,
                                             const uavcan::protocol::GetNodeInfo::Response& node_info,
                                             FirmwareFilePath& out_firmware_file_path)
    {
        /* This is a work  around for two issues.
         *  1) FirmwareFilePath is 40
         *  2) OK using is using 32 for max file names.
         *
         *  So for the file:
         *    org.pixhawk.px4cannode-v1-0.1.59efc137.uavcan.bin
         *    +---fw
         *        +-c                           <----------- Files are cashed here.
         *          +--- 59efc137.bin           <----------  A unknown Firmware file
         *        +---org.pixhawk.px4cannode-v1 <---------- node_info.name
         *        +---1.0 <-------------------------------- node_info.name's hardware_version.major,minor
         *            + - 59efc137.bin          <----------- A well known file must match the name
         *                                                   in the root fw folder, so if it does not exist
         *                                                   it is copied up
         */
        bool rv = false;

        char fname_root[FirmwarePath::MaxBasePathLength + 1];
        int n = snprintf(fname_root, sizeof(fname_root), "%s%s/%d.%d",
                         paths_->getFirmwareBasePath().c_str(),
                         node_info.name.c_str(),
                         node_info.hardware_version.major,
                         node_info.hardware_version.minor);

        if (n > 0 && n < (int) sizeof(fname_root) - 2)
        {
            DIR* const fwdir = opendir(fname_root);

            fname_root[n++] = FirmwarePath::getPathSeparator();
            fname_root[n++] = '\0';

            if (fwdir != NULL)
            {
                struct dirent* pfile = NULL;
                while ((pfile = readdir(fwdir)) != NULL)
                {
                    // TODO: This is not POSIX compliant
                    if (DIRENT_ISFILE(pfile->d_type))
                    {
                        // Open any bin file in there.
                        if (strstr(pfile->d_name, ".bin") != NULL)
                        {
                            FirmwarePath::PathString full_src_path = fname_root;
                            full_src_path += pfile->d_name;

                            FirmwarePath::PathString full_dst_path = paths_->getFirmwareCachePath().c_str();
                            full_dst_path += pfile->d_name;

                            // ease the burden on the user
                            int cr = copyIfNot(full_src_path.c_str(), full_dst_path.c_str());

                            // We have a file, is it a valid image
                            FirmwareCommon fw;

                            if (cr == 0 && fw.getFileInfo(full_dst_path.c_str()) == 0)
                            {
                                if (node_info.software_version.image_crc == 0 ||
                                    (node_info.software_version.major == 0 && node_info.software_version.minor == 0) ||
                                    fw.descriptor.image_crc != node_info.software_version.image_crc)
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
    FirmwareVersionChecker()
        : paths_(NULL)
    { }

    /**
     * Initializes the Firmware File back-end storage by passing a paths object
     * that maintains the directory where files will be stored.
     */
    int init(FirmwarePath& paths)
    {
        paths_ = &paths;
        return 0;
    }

    const char* getFirmwarePath() const
    {
        return paths_->getFirmwareCachePath().c_str();
    }
};

}

#endif // Include guard
