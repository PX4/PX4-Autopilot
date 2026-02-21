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

#ifndef PIPE_MISC_H
#define PIPE_MISC_H


#include <stdio.h>  // for fprintf
#include <stdint.h>

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
int _mkdir_recursive(const char* dir);


/**
 * @brief      equivalent to rm -r
 *
 * @param      path  The directory to remove
 *
 * @return     0 on success, -1 on failure
 */
int _remove_recursive(const char *path);


/**
 * wrapper for access() to check if a file exists to make code more readable
 */
int _exists(char* path);


// current clock time monotonic in nanoseconds
int64_t _time_monotonic_ns(void);


// fetch a random integer in a range (inclusive)
int _random_number(int min_num, int max_num);


#endif // PIPE_MISC_H
