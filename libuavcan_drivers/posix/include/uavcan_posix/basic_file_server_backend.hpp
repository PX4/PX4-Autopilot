/****************************************************************************
*
*   Copyright (c) 2015 PX4 Development Team. All rights reserved.
*      Author: Pavel Kirienko <pavel.kirienko@gmail.com>
*              David Sidrane <david_s5@usa.net>
*
****************************************************************************/

#ifndef UAVCAN_POSIX_BASIC_FILE_SERVER_BACKEND_HPP_INCLUDED
#define UAVCAN_POSIX_BASIC_FILE_SERVER_BACKEND_HPP_INCLUDED

#include <sys/stat.h>
#include <cstdio>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>

#include <uavcan/data_type.hpp>
#include <uavcan/protocol/file/Error.hpp>
#include <uavcan/protocol/file/EntryType.hpp>
#include <uavcan/protocol/file/Read.hpp>
#include <uavcan/protocol/file_server.hpp>

#include "firmware_common.hpp"

namespace uavcan_posix
{
/**
 * This interface implements a POSIX compliant IFileServerBackend interface
 */
class BasicFileSeverBackend : public uavcan::IFileServerBackend
{

    enum { FilePermissions = 438 };   ///< 0o666

public:
    /**
     *
     * Back-end for uavcan.protocol.file.GetInfo.
     * Implementation of this method is required.
     * On success the method must return zero.
     */
    __attribute__((optimize("O0")))
    virtual int16_t getInfo(const Path& path, uint64_t& out_crc64, uint32_t& out_size, EntryType& out_type)
    {

        int rv = uavcan::protocol::file::Error::INVALID_VALUE;

        if (path.size() > 0)
        {

            FirmwareCommon fw;
            fw.getFileInfo(path.c_str());
            out_crc64 = fw.descriptor.image_crc;
            out_size = fw.descriptor.image_size;
            // todo Using fixed flag FLAG_READABLE until we add file permission checks to return actual value.
            out_type.flags = uavcan::protocol::file::EntryType::FLAG_FILE |
                             uavcan::protocol::file::EntryType::FLAG_READABLE;;
            rv = 0;
        }
        return rv;
    }

    /**
     * Back-end for uavcan.protocol.file.Read.
     * Implementation of this method is required.
     * @ref inout_size is set to @ref ReadSize; read operation is required to return exactly this amount, except
     * if the end of file is reached.
     * On success the method must return zero.
     */
    __attribute__((optimize("O0")))
    virtual int16_t read(const Path& path, const uint32_t offset, uint8_t* out_buffer, uint16_t& inout_size)
    {

        int rv = uavcan::protocol::file::Error::INVALID_VALUE;

        if (path.size() > 0)
        {


            int fd = open(path.c_str(), O_RDONLY);

            if (fd < 0)
            {
                rv = errno;
            }
            else
            {

                if (::lseek(fd, offset, SEEK_SET) < 0)
                {
                    rv = errno;
                }
                else
                {
                    //todo uses a read at offset to fill on EAGAIN
                    ssize_t len  = ::read(fd, out_buffer, inout_size);

                    if (len <  0)
                    {
                        rv = errno;
                    }
                    else
                    {

                        inout_size =  len;
                        rv = 0;
                    }
                }
                close(fd);
            }
        }
        return rv;
    }

    BasicFileSeverBackend() { }




};

}

#endif // Include guard
