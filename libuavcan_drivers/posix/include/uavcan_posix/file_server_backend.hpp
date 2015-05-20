/****************************************************************************
*
*   Copyright (c) 2015 PX4 Development Team. All rights reserved.
*      Author: Pavel Kirienko <pavel.kirienko@gmail.com>
*              David Sidrane <david_s5@usa.net>
*
****************************************************************************/

#ifndef UAVCAN_POSIX_FILE_SERVER_BACKEND_HPP_INCLUDED
#define UAVCAN_POSIX_FILE_SERVER_BACKEND_HPP_INCLUDED

#include <sys/stat.h>
#include <cstdio>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>

#include <uavcan/data_type.hpp>
#include <uavcan/protocol/file/EntryType.hpp>
#include <uavcan/protocol/file/Read.hpp>
#include <uavcan/protocol/file_server.hpp>

namespace uavcan_posix
{
/**
 * This interface implements a POSIX compliant IFileServerBackend interface
 */
class FileSeverBackend : public uavcan::IFileServerBackend
{
  /**
   * Maximum length of full path including / the file name
   */
  enum { MaxPathLength =  Path::MaxSize + 128 };

  enum { FilePermissions = 438 };     ///< 0o666

  /**
   * This type is used for the path
   */
  typedef uavcan::MakeString<MaxPathLength>::Type PathString;

  PathString base_path;

public:
  /**
   *
   * Backend for uavcan.protocol.file.GetInfo.
   * Implementation of this method is required.
   * On success the method must return zero.
   */
    virtual int16_t getInfo(const Path& path, uint64_t& out_crc64, uint32_t& out_size, EntryType& out_type)
    {

      enum { MaxBufferLength = 256 };

      int rv = -uavcan::ErrInvalidParam;
      if (path.size()) {

        rv = -uavcan::ErrFailure;

        PathString root_path = base_path.c_str();
        root_path += path;

        uint32_t size = 0;
        int fd = open(path.c_str(), O_RDONLY);
        if (fd >= 0)
        {
            uavcan::DataTypeSignatureCRC crc;
            ssize_t len;

            uint8_t bytes[MaxBufferLength];

            do {

              len = ::read(fd, bytes, MaxBufferLength);

              if (len <  0) {
                  return rv;
              }

              if (len >  0) {
                size += len;
                crc.add(bytes, len);
              }

            } while(len);
            close(fd);
            out_crc64 = crc.get();
            out_size = size;
            EntryType t;
            t.flags = uavcan::protocol::file::EntryType::FLAG_FILE;
            out_type = t;
            rv = 0;
        }
      }
      return rv;
    }

    /**
     * Backend for uavcan.protocol.file.Read.
     * Implementation of this method is required.
     * @ref inout_size is set to @ref ReadSize; read operation is required to return exactly this amount, except
     * if the end of file is reached.
     * On success the method must return zero.
     */
      virtual int16_t read(const Path& path, const uint32_t offset, uint8_t* out_buffer, uint16_t& inout_size)
      {

          int rv = -uavcan::ErrInvalidParam;

          if (path.size()) {

            rv = -uavcan::ErrFailure;

            PathString root_path = base_path.c_str();
            root_path += path;

            int fd = open(path.c_str(), O_RDONLY);

            if (fd >= 0)
            {
                if (::lseek(fd, offset, SEEK_SET) >= 0) {

                    ssize_t len  = ::read(fd, out_buffer, inout_size);
                    close(fd);

                  if (len <  0) {
                      return rv;
                  }
                  inout_size =  len;
                  rv = 0;
                }
            }
          }
        return rv;
      }

    /**
     * Initializes the file serrver based backend storage by passing a path to
     * the directory where files will be stored.
     */
    int init(const PathString& path)
    {
        using namespace std;

        int rv = -uavcan::ErrInvalidParam;

        if (path.size() > 0)
        {
            base_path = path.c_str();

            if (base_path.back() == '/')
            {
                base_path.pop_back();
            }

            rv = 0;
            struct stat sb;
            if (stat(base_path.c_str(), &sb) != 0 || !S_ISDIR(sb.st_mode))
            {
                rv = mkdir(base_path.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
            }
            if (rv >= 0 )
            {
                base_path.push_back('/');
                if ((base_path.size() + Path::MaxSize) > MaxPathLength)
                {
                    rv = -uavcan::ErrInvalidConfiguration;
                }
            }
        }
        return rv;
    }
};

}

#endif // Include guard
