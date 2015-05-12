/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/protocol/dynamic_node_id_server/event.hpp>
#include <time.h>

namespace uavcan_posix
{
namespace dynamic_node_id_server
{
/**
 * This interface implements a POSIX compliant file based IEventTracer interface
 */
class FileEventTracer : public uavcan::dynamic_node_id_server::IEventTracer
{
    /**
     * Maximum length of full path to log file
     */

    enum { MaxPathLength = 128, FormatBufferLength = 64 };


    /**
     * This type is used for the path
     */
    typedef uavcan::Array<uavcan::IntegerSpec<8, uavcan::SignednessUnsigned, uavcan::CastModeTruncate>,
                          uavcan::ArrayModeDynamic, MaxPathLength> PathString;


    PathString path_;


public:

    FileEventTracer() { }

    virtual void onEvent(uavcan::dynamic_node_id_server::TraceCode code, uavcan::int64_t argument)
    {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        int fd = open(path_.c_str(), O_WRONLY | O_CREAT | O_APPEND);
        if (fd >= 0 )
        {
            char buffer[FormatBufferLength + 1];
            int n =   snprintf(buffer, FormatBufferLength, "%d.%ld,%d,%lld\n", ts.tv_sec, ts.tv_nsec, code, argument);
            write(fd, buffer, n);
            close(fd);
        }
    }
    /**
     * Initializes the File based event trace
     *
     */

    int init(const PathString & path)
    {
        using namespace std;

        int rv = -uavcan::ErrInvalidParam;

        if (path.size() > 0)
        {
            path_ = path.c_str();
            rv = open(path_.c_str(), O_WRONLY | O_CREAT | O_TRUNC);
            close(rv);
        }
        return rv;
    }

};

}
}
