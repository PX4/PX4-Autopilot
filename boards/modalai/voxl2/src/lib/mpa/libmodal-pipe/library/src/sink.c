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


#define _GNU_SOURCE
#include <stdio.h>  // for fprintf
#include <unistd.h> // for read() & write()
#include <errno.h>  // to check error in read() and write()
#include <fcntl.h>  // for O_WRONLY & O_RDONLY
#include <string.h> // for strlen()
#include <stdlib.h> // for malloc
#include <sys/stat.h>   // for mkfifo
#include <sys/types.h>  // for mkfifo
#include <signal.h>     // for pthread_kill
#include <pthread.h>

#include <modal_pipe_sink.h>
#include "misc.h"

// shorten these defines to improve code readability
#define N_CH        PIPE_SINK_MAX_CHANNELS
#define DIR_LEN     MODAL_PIPE_MAX_DIR_LEN
#define NAME_LEN    MODAL_PIPE_MAX_NAME_LEN
#define PATH_LEN    MODAL_PIPE_MAX_PATH_LEN



// struct to define the state of each channel
typedef struct sink_channel_t{
    int             running;            ///< set to 1 once a channel is running
    int             pipe_fd;            ///< file descriptor to read data from
    char            path[PATH_LEN];     ///< path of pipe so we can remove later
    char*           buf;                ///< optional read buffer on heap
    int             buf_len;            ///< length of read buffer
    pthread_t       helper_thread_id;   ///< optional helper thread id
    sink_simple_cb* cb_func;            ///< optional helper callback
    void*           cb_context;         ///< context pointer to send back to cb
} sink_channel_t;

// array of structs defining the state of each channel
static sink_channel_t c[N_CH];
// set to 1 to enable debug prints
static int en_debug;
// each channel gets a mutex to protect during init or cleanup
static pthread_mutex_t mtx[N_CH];



// dummy function to catch the USR1 signal
static void _sigusr_cb(__attribute__((unused)) int sig)
{
    if(en_debug) printf("sink helper thread received sigusr %d\n", sig);
    return;
}


// reset a channel to all 0's as if the program just started
static void _wipe_channel(int i)
{
    if(i<0 || i>=N_CH) return;
    memset(&c[i], 0, sizeof(sink_channel_t));
    return;
}


// simple helper thread
static void* _simple_helper_func(void* context)
{
    int ch = (long)context;
    int bytes_read;

    // catch the SIGUSR1 signal which we use to quit the blocking read
    struct sigaction sigusr_action = {.sa_handler=_sigusr_cb};
    sigaction(SIGUSR1, &sigusr_action, 0);

    // wait until running is set to 0. the read will return when it gets a signal
    // not much else to do since we also control opening and closing the pipe
    // unlike client and servers that have to be tolerant to each other.
    while(c[ch].running){
        bytes_read = read(c[ch].pipe_fd, c[ch].buf, c[ch].buf_len);
        if(c[ch].cb_func){
            c[ch].cb_func(ch, c[ch].buf, bytes_read, c[ch].cb_context);
        }
    }

    if(en_debug) printf("sink channel %d helper thread exiting cleanly\n", ch);
    return NULL;
}


int pipe_sink_create(int ch, const char* path, int flags, int pipe_size, int buf_len)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return -1;
    }
    if(c[ch].running){
        fprintf(stderr, "ERROR in %s, channel %d already running\n", __FUNCTION__, ch);
        return -1;
    }
    if((flags & SINK_FLAG_EN_SIMPLE_HELPER) && buf_len<1){
        fprintf(stderr, "ERROR in %s, buffer length should be >0 when enabling simple helper\n", __FUNCTION__);
        return -1;
    }
    if(flags & SINK_FLAG_EN_DEBUG_PRINTS){
        en_debug = 1;
    }

    // validate pipe size
    if(pipe_size < (4*1024)){
        fprintf(stderr, "WARNING in %s, requested pipe size less than 4k, using default of 1M\n", __FUNCTION__);
        pipe_size = 1024*1024;
    }
    if(pipe_size>(256*1024*1024)){
        fprintf(stderr, "WARNING in %s, trying to set default pipe size >256MiB probably won't work\n", __FUNCTION__);
    }

    // validity checking
    int pathlen = strlen(path);
    if(pathlen<1){
        fprintf(stderr, "ERROR in %s, empty path string provided\n", __FUNCTION__);
        return -1;
    }
    if(pathlen>=PATH_LEN){
        fprintf(stderr, "ERROR in %s, path string too long\n", __FUNCTION__);
        return -1;
    }
    if(path[pathlen-1]=='/'){
        fprintf(stderr, "ERROR in %s, path string can't end in '/'\n", __FUNCTION__);
        return -1;
    }

    // make any necessary parent directories
    if(_mkdir_recursive(path)){
        fprintf(stderr, "Error in %s making directory\n", __FUNCTION__);
        return -1;
    }

    // lock the mutex and make the new pipe
    pthread_mutex_lock(&mtx[ch]);
    if(mkfifo(path, 0666)){
        if(errno!=EEXIST){
            perror("failed to mkfifo");
            pthread_mutex_unlock(&mtx[ch]);
            return -1;
        }
    }

    // now open the new path
    int fd = open(path, O_RDWR);
    if(fd<0){
        perror("ERROR in pipe_sink_init_channel opening pipe");
        pthread_mutex_unlock(&mtx[ch]);
        return -1;
    }

    // set pipe size
    errno = 0;
    int new_size = fcntl(fd, F_SETPIPE_SZ, pipe_size);
    pthread_mutex_unlock(&mtx[ch]);

    // error check
    if(new_size<pipe_size){
        fprintf(stderr,"WARNING in %s, failed to set pipe size\n", __FUNCTION__);
        perror(" ");
        if(errno == EPERM){
            fprintf(stderr, "You may need to be root to make a pipe that big\n");
        }
    }

    // save information for later
    c[ch].pipe_fd = fd;
    strcpy(c[ch].path, path);

    // all initialized now!
    c[ch].running = 1;

    // start helper thread if requested
    if(flags & SINK_FLAG_EN_SIMPLE_HELPER){
        // allocate memory for the read buffer
        c[ch].buf = malloc(buf_len);
        c[ch].buf_len = buf_len;
        // start thread
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_create(&c[ch].helper_thread_id, &attr, _simple_helper_func, (void*)(long)ch);
        pthread_attr_destroy(&attr);
    }

    pthread_mutex_unlock(&mtx[ch]);
    return 0;
}


int pipe_sink_set_simple_cb(int ch, sink_simple_cb* cb, void* context)
{
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return -1;
    }
    pthread_mutex_lock(&mtx[ch]);
    c[ch].cb_context = context;
    c[ch].cb_func = cb;
    pthread_mutex_unlock(&mtx[ch]);
    return 0;
}


int pipe_sink_get_fd(int ch)
{
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return -1;
    }
    if(c[ch].pipe_fd<=0){
        fprintf(stderr, "ERROR in %s, channel not initialized yet\n", __FUNCTION__);
        return -1;
    }
    return c[ch].pipe_fd;
}


void pipe_sink_close(int ch)
{
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return;
    }

    // nothing to do if not running
    if(!c[ch].running) return;

    // signal to stop running and disable callback
    pthread_mutex_lock(&mtx[ch]);
    c[ch].running = 0;
    c[ch].cb_func = NULL;

    // if helper thread is running, quit it
    if(c[ch].helper_thread_id){
        // send signal to thread, this is just to make the blocking read quit
        pthread_kill(c[ch].helper_thread_id, SIGUSR1);

#ifdef __ANDROID__
        errno = pthread_join(c[ch].helper_thread_id, NULL);
#else
        // do a timed join, 1 second timeout
        struct timespec thread_timeout;
        clock_gettime(CLOCK_REALTIME, &thread_timeout);
        thread_timeout.tv_sec += 1;
        errno = pthread_timedjoin_np(c[ch].helper_thread_id, NULL, &thread_timeout);
#endif
        if(errno==ETIMEDOUT){
            fprintf(stderr, "WARNING, %s timed out joining read thread\n", __FUNCTION__);
        }

        // free its read buffer
        free(c[ch].buf);
    }

    // clean everything up
    close(c[ch].pipe_fd);
    remove(c[ch].path);
    _wipe_channel(ch);
    pthread_mutex_unlock(&mtx[ch]);

    return;
}


void pipe_sink_close_all(void)
{
    for(int i=0; i<N_CH; i++) pipe_sink_close(i);
    return;
}


// DEPRECATED
int pipe_sink_init_channel(int ch, char* path, int flags, int buf_len)
{
    return pipe_sink_create(ch, path, flags, 1024*1024, buf_len);
}

// DEPRECATED
void pipe_sink_close_channel(int ch)
{
    return pipe_sink_close(ch);
}

