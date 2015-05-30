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
#include <uavcan/data_type.hpp>

namespace uavcan_posix
{
/**
 * This interface implements a POSIX compliant IFileServerBackend interface
 */
class BasicFileSeverBackend : public uavcan::IFileServerBackend
{
    enum { FilePermissions = 438 };   ///< 0o666


protected:

    class FDCacheBase {

    public:
        FDCacheBase() { }
        virtual ~FDCacheBase() { }

        virtual int open(const char *path, int oflags)
        {
            using namespace std;

            return ::open(path, oflags);
        }

        virtual int close(int fd, bool done = true)
        {
            using namespace std;

            return ::close(fd);
        }

    };

    FDCacheBase fallback_;

    class FDCache : public FDCacheBase {

        enum { MaxAgeSeconds = 3 };

        class FDCacheItem {

            friend FDCache;
            FDCacheItem *next_;
            time_t last_access_;
            int fd_;
            int oflags_;
            const char *path_;

        public:

            enum { InvalidFD = -1 };

            FDCacheItem() :
                next_(0),
                last_access_(0),
                fd_(InvalidFD),
                oflags_(0),
                path_(0)
            { }

            FDCacheItem(int fd, const char * path, int oflags) :
                next_(0),
                last_access_(0),
                fd_(fd),
                oflags_(oflags),
                path_(strdup(path))
            {

            }

            ~FDCacheItem()
            {
                if (valid())
                {
                    delete path_;
                }
            }

            inline bool valid()
            {
                return path_ != 0;
            }

            inline int getFD()
            {
                return fd_;
            }

            inline time_t getAccess()
            {
                return last_access_;
            }

            time_t acessed()
            {
                last_access_ = time(0);
                return getAccess();
            }

            void expire()
            {
                last_access_ = 0;
            }

            bool expired()
            {
                return 0 == last_access_ || (time(0) - last_access_) > MaxAgeSeconds;
            }

            int compare(const char * path, int oflags)
            {
                return (oflags_ == oflags && 0 == ::strcmp(path, path_)) ? 0 : 1;
            }

            int compare(int fd)
            {
                return fd_ == fd ? 0 : 1;
            }

        };

        FDCacheItem* head_;

        FDCacheItem* find(const char *path, int oflags)
        {
            for(FDCacheItem* pi = head_; pi; pi = pi->next_)
            {
                if (0 == pi->compare(path, oflags))
                {
                    return pi;
                }
            }
            return 0;
        }

        FDCacheItem* find(int fd)
        {
            for(FDCacheItem* pi = head_; pi; pi = pi->next_)
            {
                if(0 == pi->compare(fd))
                {
                    return pi;
                }
            }
            return 0;
        }


        FDCacheItem* add(FDCacheItem* pi)
        {
            pi->next_ = head_;
            head_ = pi;
            pi->acessed();
            return pi;
        }

        void removeExpired(FDCacheItem** pi)
        {
            while (*pi)
            {
                if ((*pi)->expired())
                {
                    FDCacheItem* next = (*pi)->next_;
                    (void)FDCacheBase::close((*pi)->fd_);
                    delete(*pi);
                    *pi = next;
                    continue;
                }
                pi = &(*pi)->next_;
            }
        }

        void remove(FDCacheItem* pi, bool done)
        {
            if (done)
            {
                pi->expire();
            }
            removeExpired(&head_);
        }


        void clear()
        {
            FDCacheItem* tmp;
            for(FDCacheItem* pi = head_; pi; pi = tmp)
            {
                tmp = pi->next_;
                (void)FDCacheBase::close(pi->fd_);
                delete pi;
            }
        }


    public:

        FDCache() :
            head_(0)
        { }

        virtual ~FDCache()
        {
            clear();
        }

        virtual int open(const char *path, int oflags)
        {
            int fd = FDCacheItem::InvalidFD;

            FDCacheItem *pi = find(path, oflags);

            if (pi != 0)
            {
                pi->acessed();
            }
            else
            {
                fd = FDCacheBase::open(path, oflags);
                if (fd < 0)
                {
                    return fd;
                }

                /* Allocate and clone path */

                pi = new FDCacheItem(fd, path, oflags);

                /* Allocation worked but check clone */

                if (pi && !pi->valid())
                {

                    /* Allocation worked but clone or path failed */

                    delete pi;
                    pi = 0;
                }

                if (pi == 0)
                {
                    /*
                     * If allocation fails no harm just can not cache it
                     * return open fd
                     */

                    return fd;
                }
                /* add new */
                add(pi);
            }
            return pi->getFD();
        }


        virtual int close(int fd, bool done)
        {
            FDCacheItem *pi = find(fd);
            if (pi == 0)
            {
                /*
                 * If not found just close it
                 */

                return FDCacheBase::close(fd);
            }
            remove(pi, done);
            return 0;
        }

    };


    FDCacheBase *fdcache_;

    FDCacheBase&  getFDCache()
    {
        if (fdcache_ == 0)
        {
            fdcache_ = new FDCache();

            if (fdcache_ == 0)
            {
                fdcache_ = &fallback_;

            }
        }
        return *fdcache_;
    }

    /**
     * Back-end for uavcan.protocol.file.GetInfo.
     * Implementation of this method is required.
     * On success the method must return zero.
     */
    virtual int16_t getInfo(const Path& path, uint64_t& out_size, EntryType& out_type)
    {

        int rv = uavcan::protocol::file::Error::INVALID_VALUE;

        if (path.size() > 0)
        {
            using namespace std;


            struct stat sb;

            rv = stat(path.c_str(), &sb);

            if (rv < 0)
            {
                rv = errno;
            }
            else
            {

                rv = 0;
                out_size = sb.st_size;
                out_type.flags = uavcan::protocol::file::EntryType::FLAG_READABLE;
                if (S_ISDIR(sb.st_mode))
                {
                    out_type.flags |= uavcan::protocol::file::EntryType::FLAG_DIRECTORY;
                }
                else if (S_ISREG(sb.st_mode))
                {
                    out_type.flags |= uavcan::protocol::file::EntryType::FLAG_FILE;
                }
                // TODO Using fixed flag FLAG_READABLE until we add file permission checks to return actual value.
            }
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
    virtual int16_t read(const Path& path, const uint64_t offset, uint8_t* out_buffer, uint16_t& inout_size)
    {
        int rv = uavcan::protocol::file::Error::INVALID_VALUE;

        if (path.size() > 0)
        {
            FDCacheBase& cache = getFDCache();
            int fd = cache.open(path.c_str(), O_RDONLY);

            if (fd < 0)
            {
                rv = errno;
            }
            else
            {
                rv = ::lseek(fd, offset, SEEK_SET);

                ssize_t len = 0;

                if (rv < 0)
                {
                    rv = errno;
                }
                else
                {
                    // TODO use a read at offset to fill on EAGAIN
                    len = ::read(fd, out_buffer, inout_size);

                    if (len < 0)
                    {
                        rv = errno;
                    }
                    else
                    {
                        rv = 0;
                    }
                }

                (void)cache.close(fd, rv != 0 || len != inout_size);
                inout_size = len;
            }
        }
        return rv;
    }

public:

    BasicFileSeverBackend() :
        fdcache_(0)
    {
    }

    ~BasicFileSeverBackend()
    {
        if (fdcache_ != &fallback_)
        {
            delete fdcache_;
            fdcache_ = 0;
        }
    }
};

}

#endif // Include guard
