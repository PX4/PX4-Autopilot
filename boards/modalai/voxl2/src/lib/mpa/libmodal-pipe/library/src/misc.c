/*******************************************************************************
 * Copyright 2025 ModalAI Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 ******************************************************************************/


#define _GNU_SOURCE     // for nftw
#include <stdio.h>      // for fprintf
#include <unistd.h>     // for read() & write()
#include <fcntl.h>      // for O_WRONLY & O_RDONLY
#include <limits.h>     // for PATH_MAX
#include <errno.h>
#include <sys/stat.h>   // for mkdir
#include <sys/types.h>  // for mode_t in mkdir
#include <ftw.h>        // for file tree walk
#include <time.h>

#include "misc.h"

/**
 * @brief      helper to make a new directory and all necessary parent
 *             directories. Returns success if folder(s) already exist.
 *
 *             This requires the directory string to contain a trailing '/'
 *             after the final directory. This is done to allow a full path to a
 *             file to be given and this function will only create the necessary
 *             directories.
 *
 *             For example:
 *
 *             _mkdir_recursive("/tmp/folder1/folder2/"
 *             _mkdir_recursive("/tmp/folder1/folder2/file1"
 *
 *             will BOTH create: /tmp/ /tmp/folder1/ and /tmp/folder1/folder2/
 *
 *             Neither will create a folder named "file1" as mkdir() would.
 *
 * @param[in]  dir   The directory string
 *
 * @return     0 on success, -1 on failure
 */
int _mkdir_recursive(const char* dir)
{
    char tmp[PATH_MAX];
    char* p = NULL;

    snprintf(tmp, sizeof(tmp),"%s",dir);
    for(p = tmp + 1; *p!=0; p++){
        if(*p == '/'){
            *p = 0;
            if(mkdir(tmp, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) && errno!=EEXIST){
                perror("ERROR calling mkdir");
                printf("tried to make %s\n", tmp);
                return -1;
            }
            *p = '/';
        }
    }
    return 0;
}


// only used by _remove_recursive
static int _unlink_cb(const char *fpath, __attribute__((unused))const struct stat *sb, __attribute__((unused))int typeflag, __attribute__((unused))struct FTW *ftwbuf)
{
    int rv = remove(fpath);
    if(rv) perror(fpath);
    return rv;
}


/**
 * @brief      equivalent to rm -rf
 *
 * @param      path  The path to remove
 *
 * @return     0 on success, -1 on failure
 */
int _remove_recursive(const char *path)
{
    return nftw(path, _unlink_cb, 64, FTW_DEPTH | FTW_PHYS);
}


int _exists(char* path)
{
    // file exists
    if(access(path, F_OK ) != -1 ) return 1;
    // file doesn't exist
    return 0;
}

int64_t _time_monotonic_ns(void)
{
    struct timespec ts;
    if(clock_gettime(CLOCK_MONOTONIC, &ts)){
        fprintf(stderr,"ERROR calling clock_gettime\n");
        return -1;
    }
    return (int64_t)ts.tv_sec*1000000000 + (int64_t)ts.tv_nsec;
}



// Old glibc was missing getrandom so for APQ8096 and 32-bit libs on QRB5165
// we have to define our own
#ifdef __arm__

    #  include <sys/syscall.h>
    #  include <errno.h>
    static int _my_getrandom(void *buf, size_t buflen)
    {
        if (buflen > 256) {
            errno = EIO;
            return -1;
        }
        return syscall(SYS_getrandom, buf, buflen, 0);
    }

#else

    #  include <sys/random.h>
    static int _my_getrandom(void *buf, size_t buflen)
    {
        return getrandom(buf, buflen, 0);
    }

# endif



// now use the platform-independent getrandom() to create a nice
// integer bounded in a range
int _random_number(int min_num, int max_num)
{
    int result = 0, low_num = 0, hi_num = 0;

    if (min_num < max_num)
    {
        low_num = min_num;
        hi_num = max_num + 1; // include max_num in output
    } else {
        low_num = max_num + 1; // include max_num in output
        hi_num = min_num;
    }

    // fetch 4 random bytes
    int n;
    _my_getrandom(&n, sizeof(int));

    // make sure it's a positive integer and scale to the desired range
    if(n<0) n *= -1;
    result = (n % (hi_num - low_num)) + low_num;

    return result;
}
