/****************************************************************************
*
*   Copyright (c) 2015 PX4 Development Team. All rights reserved.
*      Author: Pavel Kirienko <pavel.kirienko@gmail.com>
*              David Sidrane <david_s5@usa.net>
*
****************************************************************************/

#pragma once

#include <sys/stat.h>
#include <stdio.h>
#include <cstddef>
#include <cstdlib>
#include <cfcntl>
#include <cstring>
#include <cerrno>

#include <uavcan/protocol/dynamic_node_id_server/storage_backend.hpp>

namespace uavcan_posix
{
namespace dynamic_node_id_server
{
/**
 * This interface implements a POSIX compliant IStorageBackend interface
 */
class FileStorageBackend : public uavcan::dynamic_node_id_server::IStorageBackend
{
    /**
     * Maximum length of full path including / and key max
     */

    enum { MaxPathLength = 128 };


    /**
     * This type is used for the path
     */
    typedef uavcan::Array<uavcan::IntegerSpec<8, uavcan::SignednessUnsigned, uavcan::CastModeTruncate>,
                          uavcan::ArrayModeDynamic, MaxPathLength> PathString;


    PathString base_path;

public:

    FileStorageBackend() { }


    virtual String get(const String& key) const
    {
        using namespace std;
        PathString path = base_path.c_str();
        path += key;
        String value;
        int fd = open(path.c_str(), O_RDONLY);
        if (fd >= 0)
        {
            char buffer[MaxStringLength + 1];
            memset(buffer, 0, sizeof(buffer));
            int len = read(fd, buffer, MaxStringLength);
            close(fd);
            if (len > 0)
            {
                for (int i = 0; i < len; i++)
                {
                    if (buffer[i] == ' ' || buffer[i] == '\n' || buffer[i] == '\r' )
                    {
                        buffer[i] = '\0';
                        break;
                    }
                }
                value = buffer;
            }
        }
        return value;

    }

    virtual void set(const String& key, const String& value)
    {
        using namespace std;
        PathString path = base_path.c_str();
        path += key;
        int fd = open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC);
        if (fd >= 0)
        {
            write(fd, value.c_str(), value.size());
            close(fd);
        }
    }

    /**
     * Initializes the File based back end storage by passing to a path to
     * the directory where the key named files will be stored.
     * This the return result should be 0 on success.
     * If it is -ErrInvalidConfiguration then the the path name is too long to
     * Accommodate the trailing slash and max key length;
     *
     */

    int init(const PathString & path)
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
            if (rv >= 0 ) {
              base_path.push_back('/');
              if ((base_path.size() + MaxStringLength) > MaxPathLength)
              {
                  rv = -uavcan::ErrInvalidConfiguration;
              }
            }
        }
        return rv;

    }

};

}
}
