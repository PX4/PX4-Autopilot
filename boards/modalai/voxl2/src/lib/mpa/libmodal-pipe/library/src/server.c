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


#define _GNU_SOURCE     // for pthread_timedjoin_np and possibly other things
#include <stdio.h>      // for fprintf
#include <stdlib.h>     // for fprintf
#include <stdarg.h>     // for variadics
#include <unistd.h>     // for read() & write()
#include <errno.h>      // to check error in read() and write()
#include <sys/ioctl.h>
#include <fcntl.h>      // for O_WRONLY & O_RDONLY
#include <string.h>     // for strlen()
#include <sys/stat.h>   // for mkfifo()
#include <sys/types.h>  // for mkfifo()
#include <signal.h>     // for pthread_kill
#include <pthread.h>
#include <ftw.h>        // for file tree walk
#include <sys/socket.h> // for socket
#include <sys/un.h>  // for struct sockaddr_un
#include <modal_json.h>
#include <poll.h>
#include <sys/inotify.h>

#include <modal_pipe_server.h>
#include <modal_start_stop.h>
#include "misc.h"

// shorten these names to improve code readability
#define N_CH        PIPE_SERVER_MAX_CHANNELS
#define N_CLIENT    PIPE_SERVER_MAX_CLIENTS_PER_CH
#define DIR_LEN     MODAL_PIPE_MAX_DIR_LEN
#define NAME_LEN    MODAL_PIPE_MAX_NAME_LEN
#define PATH_LEN    MODAL_PIPE_MAX_PATH_LEN


// struct containing the full state of each channel
typedef struct server_channel_t{

    // this first section are things that are manipulated by internal functions
    // in normal operation. Most are protected by a mutex. If you add anything
    // here in the future, make sure it's cleanup up by _wipe_channel()!!
    int         running;                            ///< set to 1 once a channel is running
    int         claimed;                            ///< flag to indicate that the channel has been claimed, even if it is not yet running
    char        base_dir[DIR_LEN];                  ///< string containing base directory for each channel
    char        request_path[DIR_LEN+16];           ///< string containing request pipe path for each channel
    char        control_path[DIR_LEN+16];           ///< string containing control pipe path for each channel
    char        info_path[DIR_LEN+16];              ///< string containing info pipe path for each channel
    int         request_fd;                         ///< one request pipe per channel, store fd here
    int         control_fd;                         ///< one control pipe per channel, store fd here
    pthread_t   request_thread;                     ///< thread ID reading the request pipe
    pthread_t   monitor_thread;                     ///< thread ID monitoring for disconnected clients
    pthread_t   control_thread;                     ///< thread ID reading the control pipe
    int         control_thread_priority;            ///< 0-99
    int         n_clients;                          ///< number of clients in any state
    char        client_names[N_CLIENT][NAME_LEN];   ///< store name of each client
    int         client_sock_fd[N_CLIENT];           ///< client socket used by server to send fds
    int         flags;                              ///< copy of flags passed to server open function

    // normal writing and changing pipe size is protected on a per-client basis so
    // the server never gets hung up waiting for the main channel mutex
    pthread_mutex_t client_mtx[N_CLIENT];       ///< mutex to protect writing to individual clients
    int         data_fd[N_CLIENT];              ///< 2D array of all file descriptors for data pipes
    char        data_path[N_CLIENT][PATH_LEN];  ///< store path of each data pipe to check for duplicates
    int         client_state[N_CLIENT];         ///< keep track as clients connect and disconnect

#ifdef EN_ION_BUF
    // ion buffer pool reference used in control pipe callback to easily mark an ion buffer as unused
    mpa_ion_buf_pool_t* ion_pool_ref;
#endif

    // TODO disable clients being able to change this
    int         actual_pipe_size[N_CLIENT];     ///< actual pipe size as reported by the kernel for that client's data pipe
 
    // info as set by the user when creating the server pipe
    pipe_info_t info;
    cJSON*      info_json;

    // Support for encoded image frames which require a header and then an I frame to be sent when a client connects
    void *                  encode_header;                 ///< The header to be sent to each client when it connects
    camera_image_metadata_t encode_header_meta;            ///< Metadata for the header
    uint8_t                 accepting_p_frames[N_CLIENT];  ///< Keeps track of which clients are ready for p frames

    // everything below is set by the user as a configurable option before
    // creating the pipe. None of these things are wiped in _wipe_channel()
    int                     control_pipe_size;      // global variable, so starts at 0
    int                     control_read_buf_size;  // global variable, so starts at 0
    server_control_cb*      control_cb_func;
    server_request_cb*      request_cb_func;        // DEPRECATED
    server_connect_cb*      connect_cb_func;
    server_disconnect_cb*   disconnect_cb_func;
    void*                   control_cb_context;
    void*                   connect_cb_context;
    void*                   request_cb_context;     // DEPRECATED
    void*                   disconnect_cb_context;
} server_channel_t;


// array of structs defining the state of each channel
static server_channel_t c[N_CH];
// set to 1 to enable debug prints
#define en_debug (c[ch].flags & SERVER_FLAG_EN_DEBUG_PRINTS)

// each channel gets a mutex to protect from writing during init or cleanup
static pthread_mutex_t mtx[N_CH];
// One mutex to protect the claimed field of all channels
static pthread_mutex_t claim_mtx;

static int _disconnect_client(int ch, int client_id);

static void _safe_unclaim(int ch)
{
    pthread_mutex_lock(&claim_mtx);
    c[ch].claimed = 0;
    pthread_mutex_unlock(&claim_mtx);
    return;
}

// dummy function to catch the USR1 signal
static void _sigusr_cb(__attribute__((unused)) int sig)
{
    return;
}

// dummy function to catch sigpipe when client disconnects
static void _sigpipe_handler(__attribute__((unused)) int signum)
{
    return;
}


// reset a channel to all 0's as if the program just started
// preserve a few user-defined settings
static void _wipe_channel(int ch)
{
    if(ch<0 || ch>=N_CH) return;

    c[ch].running = 0;
    memset(c[ch].base_dir, 0, DIR_LEN);
    memset(c[ch].request_path, 0, DIR_LEN+16);
    memset(c[ch].control_path, 0, DIR_LEN+16);
    memset(c[ch].info_path, 0, DIR_LEN+16);
    c[ch].request_fd = 0;
    c[ch].control_fd = 0;
    c[ch].request_thread = 0;
    c[ch].monitor_thread = 0;
    c[ch].control_thread = 0;
    c[ch].control_thread_priority = 0;
    c[ch].n_clients = 0;
    memset(c[ch].data_fd, 0, sizeof(int)*N_CLIENT);
    memset(c[ch].data_path, 0, N_CLIENT*PATH_LEN);
    memset(c[ch].client_names, 0, N_CLIENT*NAME_LEN);
    memset(c[ch].client_state, 0, sizeof(int)*N_CLIENT);
    memset(&c[ch].info, 0, sizeof(pipe_info_t));

    if(c[ch].info_json!=NULL){

        cJSON_Delete(c[ch].info_json);
        c[ch].info_json = NULL;
    }

    if(c[ch].encode_header){
        free(c[ch].encode_header);
        memset(&c[ch].encode_header_meta, 0, sizeof(camera_image_metadata_t));
        memset(&c[ch].accepting_p_frames, 0, sizeof(c[ch].accepting_p_frames[0])*N_CLIENT);
    }

    // everything else in the struct can be left alone

    return;
}


static void* _request_listener_func(void* context)
{
    char buf[256];
    int ch = (long)context;
    int bytes_read;

    // catch the SIGUSR1 signal which we use to quit the blocking read
    struct sigaction sigusr_action = {.sa_handler=_sigusr_cb};
    sigaction(SIGUSR1, &sigusr_action, 0);

    while(c[ch].running){
        errno = 0;
        bytes_read = read(c[ch].request_fd, buf, sizeof(buf));
        if(bytes_read>0){
            // successful read, make a new data pipe
            int client_id = pipe_server_add_client(ch, buf);
            // also inform the server via callback if it's been set
            if(client_id>=0 && c[ch].request_cb_func) {
                c[ch].request_cb_func(ch, buf, bytes_read, client_id, c[ch].request_cb_context);
            }
        }
        else if(bytes_read==0){
            // return of 0 means nobody is on the other end of the pipe
            // just sleep waiting for a client to start
            usleep(100000);
        }
        else{
            // actual error occured
            if(errno==EINTR) break; // caught the interrupt signal (ctrl-c), exit loop
            perror("request listener read error:");
            usleep(500000); // must sleep or we go into loop when other end of the pipe is closed
        }
    }

    if(en_debug) printf("channel %d request thread closing\n", ch);
    return NULL;
}


// This is only called by the disconnect monitor thread. If inotify tells us
// that a client has disconnected and closed its pipe either intentionally
// or by crashing,
static int _disconnect_client_by_name(int ch, char* name)
{
    // search for the client
    for(int i = 0; i < N_CLIENT; ++i){

        // uninitialized slots are fine, shouldn't have a name to check
        if(c[ch].client_state[i] == CLIENT_UNINITIALIZED) continue;
        // disconnected slots are fine, the write call may have disconnected them already
        if(c[ch].client_state[i] == CLIENT_DISCONNECTED) continue;

        // connected clients can be checked by name
        if(strcmp(c[ch].client_names[i], name) == 0) {
            pthread_mutex_lock(&c[ch].client_mtx[i]);
            _disconnect_client(ch, i);
            pthread_mutex_unlock(&c[ch].client_mtx[i]);
        }
    }
    return 0;
}

// thread to monitor if clients disconnect. Failed writes to these clients
// will also trigger a disconnect but this works to prevent orphaned clients
// when no data is currently being written to the pipe.
static void* disconnect_monitor_thread_func(void* context)
{
    int ch = (long)context;

    // blocking inotify handle
    int fd = inotify_init();
    if(fd < 0){
        perror("inotify_init");
        exit(EXIT_FAILURE);
    }

    // Watch the file for close events.Clients open pipes read-only so we only
    // need to watch for close events on files not open for writing
    int wd = inotify_add_watch(fd, c[ch].base_dir, IN_CLOSE_NOWRITE);
    if(wd < 0){
        perror("inotify_add_watch");
        close(fd);
        return NULL;
    }

    // if a client has a lot of pipes open we are going to get a slew of inotify
    // events all in one read so allocate a decent buffer to capture them all.
    char buf[4096] __attribute__ ((aligned(__alignof__(struct inotify_event))));

    // keep servicing the inotify fd  until we quit. Blocking read() call will
    // return on SIGUSR1 when our process quits
    while (c[ch].running){
        // sit here on blocking read most of the time
        errno = 0;
        int length = read(fd, buf, sizeof(buf));

        // thread has likely got SIGUSR1 and is time to quit
        if(length <= 0 && errno != EAGAIN) break;

        // haven't seen this happen but seems like a safe check
        if(length % sizeof(struct inotify_event)){
            fprintf(stderr, "ERROR expected inotify read length as multiple of sizeof(struct inotify_event)\n");
            continue;
        }

        struct inotify_event* events = (struct inotify_event*)buf;
        int n = length/sizeof(struct inotify_event);
        for(int i=0; i<n; i++){
            if(!events[i].len) continue;
            if(events[i].mask & IN_ISDIR) continue;
            if(en_debug){
                printf("disconnecting client %s due to file closure\n", events[i].name);
            }
            _disconnect_client_by_name(ch, events[i].name);
        }
    }

    inotify_rm_watch(fd, wd);
    close(fd);
    if(en_debug) printf("channel %d disconnect monitor thread closing\n", ch);

    return NULL;
}


static void* _control_listener_func(void* context)
{
    int ch = (long)context;
    int bytes_read;

    // should never get here, creating the pipe should have set the read_buf_size
    // but check anyway since we are about to allocate memory
    if(c[ch].control_read_buf_size<=0){
        fprintf(stderr, "ERROR in control listener thread control read buf size must be nonzero\n");
        return NULL;
    }

    int buflen = c[ch].control_read_buf_size;
    char buf[buflen];

    // catch the SIGUSR1 signal which we use to quit the blocking read
    struct sigaction sigusr_action = {.sa_handler=_sigusr_cb};
    sigaction(SIGUSR1, &sigusr_action, 0);


    while(c[ch].running){

        if(c[ch].control_fd == 0){
            if(en_debug){
                fprintf(stderr, "channel %d helper tried to read from closed fd\n", ch);
            }
            break;
        }

        errno = 0;
        bytes_read = read(c[ch].control_fd, buf, buflen);
        buf[bytes_read] = 0;

        // might have closed pipe while reading, if so exit thread
        if(!c[ch].running){
            break;
        }

        if(bytes_read<=0){
            if(en_debug){
                printf("control pipe read returned %d\n", bytes_read);
                perror("errno=");
            }
            if(errno==EINTR) break; // caught the interrupt signal (ctrl-c), exit loop
        }
        else{
            // if we got a valid read and callback exists, execute it
            if(c[ch].control_cb_func){
                c[ch].control_cb_func(ch, buf, bytes_read, c[ch].control_cb_context);
            }
        }
    }

    if(en_debug) printf("channel %d control thread closing\n", ch);
    return NULL;
}


#ifdef EN_ION_BUF
void _ion_buf_control_cb(int ch, char* data, int bytes, __attribute__((unused)) void* context)
{
    ion_buf_release_msg_t* msgs = (ion_buf_release_msg_t*)data;
    int N = bytes / sizeof(ion_buf_release_msg_t);

    for(int i = 0; i < N; i++)
    {
        ion_buf_release_msg_t msg = msgs[i];
        int res = mpa_ion_buf_process_release_msg(c[ch].ion_pool_ref, msg);
        if (res) fprintf(stderr, "WARNING: Client %s failed to release ion buffer\n", c[ch].client_names[i]);
    }

    return;
}
#endif


static cJSON* _make_new_cjson_from_info(pipe_info_t info)
{
    cJSON* new_json;

    new_json = cJSON_CreateObject();
    if(new_json == NULL){
        fprintf(stderr, "ERROR: in %s, failed to make new cJSON object\n", __FUNCTION__);
        return NULL;
    }

    if(cJSON_AddStringToObject(new_json, "name", info.name)==NULL){
        fprintf(stderr, "ERROR: could not add name to JSON object\n");
        return NULL;
    }
    if(cJSON_AddStringToObject(new_json, "location", info.location)==NULL){
        fprintf(stderr, "ERROR: could not add location to JSON object\n");
        return NULL;
    }
    if(cJSON_AddStringToObject(new_json, "type", info.type)==NULL){
        fprintf(stderr, "ERROR: could not add type to JSON object\n");
        return NULL;
    }
    if(cJSON_AddStringToObject(new_json, "server_name", info.server_name)==NULL){
        fprintf(stderr, "ERROR: could not add server_name to JSON object\n");
        return NULL;
    }
    if(cJSON_AddNumberToObject(new_json, "size_bytes", info.size_bytes)==NULL){
        fprintf(stderr, "ERROR: could not add size_bytes to JSON object\n");
        return NULL;
    }
    if(cJSON_AddNumberToObject(new_json, "server_pid", info.server_pid)==NULL){
        fprintf(stderr, "ERROR: could not add server_pid to JSON object\n");
        return NULL;
    }

    return new_json;
}


int pipe_server_create(int ch, pipe_info_t info, int flags)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return -1;
    }

    // claim the channel before we get to setting it up so nothing else will
    // claim it in the mean time with pipe_client_get_next_available_channel()
    pthread_mutex_lock(&claim_mtx);
    int old_claim = c[ch].claimed;
    c[ch].claimed = 1;
    pthread_mutex_unlock(&claim_mtx);

    if(c[ch].running){
        fprintf(stderr, "ERROR in %s, channel %d already running\n", __FUNCTION__, ch);
        return -1;
    }

    // validate name length
    int i;
    for(i=0; i<MODAL_PIPE_MAX_NAME_LEN; i++){
        if(info.name[i]==0) break;
    }
    if(i>(MODAL_PIPE_MAX_NAME_LEN-1)){
        if(!old_claim) _safe_unclaim(ch);
        fprintf(stderr, "ERROR in %s, pipe name too long for channel %d\n", __FUNCTION__, ch);
        return -1;
    }

    // validate path length
    for(i=0; i<MODAL_PIPE_MAX_PATH_LEN; i++){
        if(info.location[i]==0) break;
    }
    if(i>(MODAL_PIPE_MAX_PATH_LEN-1)){
        if(!old_claim) _safe_unclaim(ch);
        fprintf(stderr, "ERROR in %s, pipe location too long for channel %d\n", __FUNCTION__, ch);
        return -1;
    }

    // check for empty or invalid name
    if(strlen(info.name)<=0){
        if(!old_claim) _safe_unclaim(ch);
        fprintf(stderr, "ERROR in %s, invalid pipe name: %s\n", __FUNCTION__, info.name);
        return -1;
    }
    if(strstr(info.name, "/") != NULL){
        if(!old_claim) _safe_unclaim(ch);
        fprintf(stderr, "ERROR in %s, pipe name can't contain a '/'\n", __FUNCTION__);
        return -1;
    }
    if(strstr(info.name, "unknown") != NULL){
        if(!old_claim) _safe_unclaim(ch);
        fprintf(stderr, "ERROR in %s, pipe name can't be 'unknown'\n", __FUNCTION__);
        return -1;
    }

    // clean up the pipe path location in case the user messed it up
    char dir[MODAL_PIPE_MAX_DIR_LEN];
    if(info.location[0]!='/'){
        // user either gave a bad location or no location at all, construct from name
        if(pipe_expand_location_string(info.name, dir)){
            fprintf(stderr, "ERROR in %s, invalid pipe name: %s\n", __FUNCTION__, info.name);
        }
    }
    else{
        // user gave both name and location, check location for validity
        if(pipe_expand_location_string(info.location, dir)){
            fprintf(stderr, "ERROR in %s, invalid pipe location: %s\n", __FUNCTION__, info.location);
        }
    }

    // save the cleaned up location back to info struct
    strcpy(info.location, dir);

    // set the PID
    info.server_pid = (int)getpid();

    // validate pipe size
    if(info.size_bytes < (4*1024)){
        fprintf(stderr, "WARNING in %s, requested pipe size less than 4k, using default of 1M\n", __FUNCTION__);
        info.size_bytes = 1024*1024;
    }
    if(info.size_bytes>(256*1024*1024)){
        fprintf(stderr, "WARNING in %s, trying to set default pipe size >256MiB probably won't work\n", __FUNCTION__);
    }

    // fix control pipe size to defaults if the user didn't set them
    if(c[ch].control_pipe_size<=0){
        c[ch].control_pipe_size = 64*1024;
    }
    if(c[ch].control_read_buf_size<=0){
        c[ch].control_read_buf_size = 1024;
    }

    // lock mutex before we start accessing the state
    pthread_mutex_lock(&mtx[ch]);

    // check if directory is already being used by another channel
    for(int i=0; i<N_CH; i++){
        if(strcmp(dir, c[i].base_dir)==0){
            if(!old_claim) _safe_unclaim(ch);
            fprintf(stderr,"ERROR in %s, %s already in use by channel %d\n", __FUNCTION__, dir, i);
            pthread_mutex_unlock(&mtx[ch]);
            return -1;
        }
    }

    // turn on debug prints if requested
    c[ch].flags = flags;

    // set up handler for sigpipe which occurs when client disconnects
    struct sigaction action;
    action.sa_handler = _sigpipe_handler;
    sigemptyset (&action.sa_mask);
    action.sa_flags = 0;
    sigaction (SIGPIPE, &action, NULL);

    // duplicate the directory into local mem to save for later
    // also make request and control paths
    strcpy(c[ch].base_dir, dir);
    strcpy(c[ch].request_path, dir);
    strcat(c[ch].request_path,"request");

    // make the directory for pipes to exist in
    if(_mkdir_recursive(c[ch].base_dir)){
        fprintf(stderr, "Error in %s making directory\n", __FUNCTION__);
        _wipe_channel(ch);
        pthread_mutex_unlock(&mtx[ch]);
        if(!old_claim) _safe_unclaim(ch);
        return -1;
    }

    // construct info JSON and write to file
    c[ch].info_json = _make_new_cjson_from_info(info);
    if(c[ch].info_json == NULL){
        fprintf(stderr,"ERROR in %s, failed to construct json\n", __FUNCTION__);
        _wipe_channel(ch);
        pthread_mutex_unlock(&mtx[ch]);
        if(!old_claim) _safe_unclaim(ch);
        return -1;
    }
    strcpy(c[ch].info_path, dir);
    strcat(c[ch].info_path,"info");
    if(json_write_to_file(c[ch].info_path, c[ch].info_json)){
        fprintf(stderr,"ERROR in %s, failed to write info json file\n", __FUNCTION__);
        _wipe_channel(ch);
        pthread_mutex_unlock(&mtx[ch]);
        return -1;
    }

    // make the request pipe
    errno = 0;
    if(mkfifo(c[ch].request_path, 0666)){
        if(errno!=EEXIST){
            perror("Error in pipe_server_create calling mkfifo");
            _wipe_channel(ch);
            pthread_mutex_unlock(&mtx[ch]);
            if(!old_claim) _safe_unclaim(ch);
            return -1;
        }
    }
    // opening read only will block until something else opens the other end
    // so open read-write to avoid this even though we don't write to it
    c[ch].request_fd = open(c[ch].request_path, O_RDWR);
    if(c[ch].request_fd<0){
        perror("Error in pipe_server_create opening request path");
        _wipe_channel(ch);
        pthread_mutex_unlock(&mtx[ch]);
        if(!old_claim) _safe_unclaim(ch);
        return -1;
    }

    // make the control pipe if enabled
    if(flags & SERVER_FLAG_EN_CONTROL_PIPE){
        // construct the string
        strcpy(c[ch].control_path,dir);
        strcat(c[ch].control_path,"control");
        // make pipe
        errno = 0;
        if(mkfifo(c[ch].control_path, 0666)){
            // if it already exists then don't worry
            if(errno!=EEXIST){
                perror("Error in pipe_server_create calling mkfifo");
                _wipe_channel(ch);
                pthread_mutex_unlock(&mtx[ch]);
                if(!old_claim) _safe_unclaim(ch);
                return -1;
            }
        }
        // opening read only will block until something else opens the other end
        // so open read-write to avoid this even though we don't write to it
        c[ch].control_fd = open(c[ch].control_path, O_RDWR);
        if(c[ch].control_fd<0){
            perror("Error in pipe_server_create opening control path");
            _wipe_channel(ch);
            pthread_mutex_unlock(&mtx[ch]);
            if(!old_claim) _safe_unclaim(ch);
            return -1;
        }

        // set the control pipe size
        errno = 0;
        int new_size = fcntl(c[ch].control_fd, F_SETPIPE_SZ, c[ch].control_pipe_size);

        // error check setting control pipe size
        if(new_size<c[ch].control_pipe_size){
            perror("ERROR failed to set control pipe size\n");
            if(errno == EPERM){
                fprintf(stderr, "You may need to be root to make a pipe that big\n");
            }
            _wipe_channel(ch);
            pthread_mutex_unlock(&mtx[ch]);
            if(!old_claim) _safe_unclaim(ch);
            return -1;
        }
    }



    // flag everything as initialized and start the request thread and monitor thread
    c[ch].info = info;
    c[ch].running = 1;
    pipe_pthread_create(&c[ch].monitor_thread, disconnect_monitor_thread_func, (void*)(long)ch, 0);
    pipe_pthread_create(&c[ch].request_thread, _request_listener_func, (void*)(long)ch, 0);

    // optionally start the control thread
    if(flags & SERVER_FLAG_EN_CONTROL_PIPE){
        pipe_pthread_create(&c[ch].control_thread, _control_listener_func, \
                                (void*)(long)ch, c[ch].control_thread_priority);
    }

    // finally unlock the mutex
    pthread_mutex_unlock(&mtx[ch]);

    return 0;
}

int pipe_server_get_next_available_channel()
{
    int ret = PIPE_ERROR_OTHER;

    pthread_mutex_lock(&claim_mtx);
    for(int ch = 0; ch < N_CH; ch++){
        if(!c[ch].claimed){
            c[ch].claimed = 1;
            ret = ch;
            break;
        }
    }
    pthread_mutex_unlock(&claim_mtx);

    return ret;
}

cJSON* pipe_server_get_info_json_ptr(int ch)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH);
        return NULL;
    }
    if(!c[ch].running){
        fprintf(stderr, "ERROR in %s, channel %d not initialized yet\n", __FUNCTION__, ch);
        return NULL;
    }
    return c[ch].info_json;
}


int pipe_server_update_info(int ch)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH);
        return -1;
    }
    if(!c[ch].running){
        fprintf(stderr, "ERROR in %s, channel %d not initialized yet\n", __FUNCTION__, ch);
        return -1;
    }
    if(json_write_to_file(c[ch].info_path, c[ch].info_json)){
        fprintf(stderr,"ERROR in %s, failed to write info json file\n", __FUNCTION__);
        return -1;
    }
    return 0;
}


int pipe_server_add_client(int ch, const char* name)
{
    int new_client_id = -1;

    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH);
        return -1;
    }
    if(!c[ch].running){
        fprintf(stderr, "ERROR in %s, channel %d not initialized yet\n", __FUNCTION__, ch);
        return -1;
    }
    if(name==NULL){
        fprintf(stderr, "ERROR in %s, recevied NULL pointer\n", __FUNCTION__);
        return -1;
    }

    // check the name length is within bounds
    int namelen = strlen(name);
    if(namelen >= NAME_LEN){
        fprintf(stderr, "ERROR in %s, name length is too long\n", __FUNCTION__);
        return -1;
    }

    // copy the string in and cleanup potential garbage at the end of the string
    char newname[NAME_LEN+1];
    strcpy(newname,name);
    for(int i=0;i<namelen;i++){
        if(newname[i]<32 || newname[i]>122){
            fprintf(stderr, "ERROR in %s, invalid character: %c at position %d in name\n", __FUNCTION__, newname[i], i);
            return -1;
        }
    }

    // starting to manipulate the channel struct now, lock the mutex first
    pthread_mutex_lock(&mtx[ch]);

    // check for duplicate name
    for(int i=0; i<c[ch].n_clients; i++){
        if( c[ch].client_state[i] == CLIENT_INITIALIZED ||
            c[ch].client_state[i] == CLIENT_CONNECTED)
        {
             if(strcmp(newname, c[ch].client_names[i])==0){
                fprintf(stderr, "WARNING not adding new client %s as it already exists\n", newname);
                pthread_mutex_unlock(&mtx[ch]);
                return -1;
            }
        }
    }

    // find the first empty slot in our array of 16 potential clients.
    // this slot may be brand new (uninitialized) or leftover from an old
    // and disconnected client in which case we can reuse the slot without
    // incrementing the "n_clients" field.
    int found_slot = 0;
    for(int i=0; i<N_CLIENT; i++){
        if( c[ch].client_state[i] == CLIENT_UNINITIALIZED ||
            c[ch].client_state[i] == CLIENT_DISCONNECTED){
            new_client_id = i;
            found_slot = 1;
            break;
        }
    }

    if(!found_slot){
        fprintf(stderr,"WARNING in pipe_server_add_client too many clients are trying to connect\n");
        fprintf(stderr,"maximum simultaneous clients on a single channel is %d\n", N_CLIENT);
        pthread_mutex_unlock(&mtx[ch]);
        return -1;
    }

    // construct the string.
    char full_path[PATH_LEN];
    int len = sprintf(full_path, "%s%s", c[ch].base_dir, newname);
    if(len<0){
        perror("ERROR in pipe_server_add_client constructing path:");
        pthread_mutex_unlock(&mtx[ch]);
        return -1;
    }
    len+=1; // add one to length to include the NULL character.

    // make the new pipe in the file system
    if(en_debug) printf("making new fifo %s\n", full_path);
    errno = 0;
    if(mkfifo(full_path, 0666)){
        if(errno!=EEXIST){
            perror("ERROR in pipe_server_add_client calling mkfifo:");
            pthread_mutex_unlock(&mtx[ch]);
            return -1;
        }
    }

    // open the pipe for nonblocking writes so if one client misbehaves and the
    // pipe overflows then the whole server doesn't hang up.
    // This may take a moment for the pipe to appear in the file system, so try
    // until it succeeds, timing out after a half second
    int fd, i;
    const int open_sleep_us = 1000;
    const int attempts = 500;
    for(i=0;i<attempts;i++){
        fd = open(full_path, O_WRONLY | O_NONBLOCK);
        if(fd>0) break;
        usleep(open_sleep_us);
    }

    // check if we failed to open the pipe
    if(fd<0){
        perror("ERROR in pipe_server_add_client calling open:");
        fprintf(stderr, "removing failed fifo %s\n", full_path);
        remove(full_path);
        pthread_mutex_unlock(&mtx[ch]);
        return -1;
    }
    if(en_debug){
        printf("new fifo took %d tries (%dms) to open\n", i, i*open_sleep_us/1000);
        printf("default pipe size: %d\n", fcntl(fd, F_GETPIPE_SZ));  
    } 

    // save the path string to check against later
    strcpy(c[ch].data_path[new_client_id], full_path);
    // save client name and increment n_client counter
    strcpy(c[ch].client_names[new_client_id], newname);

    // save the file descriptor and update that the client has been initialized
    pthread_mutex_lock(&c[ch].client_mtx[new_client_id]);
    c[ch].data_fd[new_client_id] = fd;
    c[ch].client_state[new_client_id] = CLIENT_INITIALIZED;

    // set the pipe size before unlocking the mutex so it's set for the first write
    errno = 0;
    int new_size = fcntl(fd, F_SETPIPE_SZ, c[ch].info.size_bytes);

    // error check if the pipe size is not as expected
    if(new_size < c[ch].info.size_bytes){
        perror("WARNING failed to set pipe size");
        if(errno == EPERM){
            fprintf(stderr, "You may need to be root to make a pipe that big\n");
        }

        // see why this happened, system might not support this large of a pipe
        char buf[24];
        int fd, system_max = 0;

        // read in what the system claims is its limit
        fd = open("/proc/sys/fs/pipe-max-size", O_RDONLY);
        if(fd>0){
            if(read(fd, buf, sizeof(buf))<1){
                perror("WARNING failed to read /proc/sys/fs/pipe-max-size");
                close(fd);
            }
            sscanf(buf, "%d", &system_max);
            close(fd);
        }

        // set new default to the system max size for future pipes
        if(system_max>0) c[ch].info.size_bytes = system_max;
        else c[ch].info.size_bytes = 1024*1024; // 1m is common

        // now try setting it again
        errno = 0;
        new_size = fcntl(fd, F_SETPIPE_SZ, c[ch].info.size_bytes);
        if(en_debug) printf("pipe %d size achieved: %d requested: %d\n", ch, new_size, c[ch].info.size_bytes);
    }

    // On some systems, a failure in F_SETPIPE_SZ can result in pipe size of 0
    if( new_size <= 0 ){
        printf( "***\nPIPE NOT CREATED!!\n***\n");
        printf("pipe %d size achieved: %d requested: %d\n", ch, new_size, c[ch].info.size_bytes);
        pthread_mutex_unlock(&c[ch].client_mtx[new_client_id]);
        pthread_mutex_unlock(&mtx[ch]);
        // TODO there are lots of other things to reset in this case
        // but this error hasn't come up for me yet
        return -1;
    }

    // if this server enables ion buffers, then create the necessary sockets
    if (c[ch].flags & SERVER_FLAG_EN_ION_BUF){
        int sock_fd = socket(AF_UNIX, SOCK_STREAM, 0);
        if (sock_fd == -1) {
            perror("ERROR could not create socket");
            pthread_mutex_unlock(&c[ch].client_mtx[new_client_id]);
            pthread_mutex_unlock(&mtx[ch]);
            return -1;
        }

        // the socket path is located at the client pipe + ".sock" 
        char sock_path[PATH_LEN];
        int len = sprintf(sock_path, "%s%s%s", c[ch].base_dir, newname, ".sock");
        if(len<0){
            perror("ERROR in pipe_server_add_client constructing path:");
            pthread_mutex_unlock(&c[ch].client_mtx[new_client_id]);
            pthread_mutex_unlock(&mtx[ch]);
            return -1;
        }

        struct sockaddr_un addr = {0};
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, sock_path, sizeof(addr.sun_path) - 1);

        // bind to the socket
        unlink(sock_path);
        if (bind(sock_fd, (struct sockaddr*)&addr, sizeof(addr)) == -1){
            perror("ERROR could not bind to socket");
            pthread_mutex_unlock(&c[ch].client_mtx[new_client_id]);
            pthread_mutex_unlock(&mtx[ch]);
            return -1;
        }

        // listen for client connections on this socket
        if (listen(sock_fd, 1) == -1) {
            perror("ERROR could not listen on socket");
            pthread_mutex_unlock(&c[ch].client_mtx[new_client_id]);
            pthread_mutex_unlock(&mtx[ch]);
            return -1;
        }

        // wait for a client connection, with a 1 second timeout
        int client_fd;
        struct timeval timeout = {.tv_sec = 1, .tv_usec = 0}; // 1 sec timeout
        fd_set set;
        FD_ZERO(&set);
        FD_SET(sock_fd, &set);

        int select_result = select(sock_fd + 1, &set, NULL, NULL, &timeout);
        if (select_result > 0) {
            client_fd = accept(sock_fd, NULL, NULL);
            if (client_fd == -1) {
                perror("ERROR: accept() failed");
                close(sock_fd);
                pthread_mutex_unlock(&c[ch].client_mtx[new_client_id]);
                pthread_mutex_unlock(&mtx[ch]);
                return -1;
            }

            // send the assigned client_id to the client
            int32_t id_msg = new_client_id;
            ssize_t n = send(client_fd, &id_msg, sizeof(id_msg), 0);
            if (n != sizeof(id_msg)) {
                perror("ERROR: failed to send client_id to client");
                close(client_fd);
                pthread_mutex_unlock(&c[ch].client_mtx[new_client_id]);
                pthread_mutex_unlock(&mtx[ch]);
                return -1;
            }
        } else if (select_result == 0) {
            printf("ERROR: No client connection within timeout period\n");
            close(sock_fd);
            unlink(sock_path);
            remove(sock_path);
            pthread_mutex_unlock(&c[ch].client_mtx[new_client_id]);
            pthread_mutex_unlock(&mtx[ch]);
            return -1;
        } else {
            perror("ERROR: select() failed");
            close(sock_fd);
            pthread_mutex_unlock(&c[ch].client_mtx[new_client_id]);
            pthread_mutex_unlock(&mtx[ch]);
            return -1;
        }

        // client connection established
        close(sock_fd);
        c[ch].client_sock_fd[new_client_id] = client_fd;
    }


    // save real pipe size for this client
    c[ch].actual_pipe_size[new_client_id] = new_size;

    // if this is a new "highest" client id we've had, bump the total
    // this does not happen if a client disconnects and reconnects
    // n_clients should only ever reflect the max number of clients ever
    // connected at a single point in time.
    if(new_client_id >= c[ch].n_clients){
        c[ch].n_clients = new_client_id+1;
    }

    // done manipulating the per-client fifo, unlock that mutex indicating it's ready to write to
    pthread_mutex_unlock(&c[ch].client_mtx[new_client_id]);

    // also done manipulating the channel struct, unlock the mutex before setting pipe size
    pthread_mutex_unlock(&mtx[ch]);

    // now we can safely let the user know a client has connected with their callback
    if(c[ch].connect_cb_func != NULL){
        c[ch].connect_cb_func(ch,new_client_id,c[ch].client_names[new_client_id],c[ch].connect_cb_context);
    }


    // in the case of h264/h265, if we have a header saved send it first thing
    // before writing any other data
    if(c[ch].encode_header) {
        c[ch].accepting_p_frames[new_client_id] = 0;
        // first try writing metadata
        int ret = pipe_server_write_to_client(ch, new_client_id, (char*)&c[ch].encode_header_meta, sizeof(camera_image_metadata_t));
        // only write the camera frame if the metadata succeeded
        if(ret==0){
            pipe_server_write_to_client(ch, new_client_id, c[ch].encode_header, c[ch].encode_header_meta.size_bytes);
        }
    }

    return new_client_id;
}


// internal version of the public pipe_server_bytes_in_pipe for use by the write
// functions that have already locked the client mutex
static int _pipe_server_bytes_in_pipe_nolock(int ch, int client_id)
{
    int n_bytes;
    if(ioctl(c[ch].data_fd[client_id], FIONREAD, &n_bytes)){
        return -1;
    }
    return n_bytes;
}


int pipe_server_bytes_in_pipe(int ch, int client_id)
{
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return -1;
    }
    if(client_id<0 || client_id>=N_CLIENT){
        fprintf(stderr, "ERROR in %s, client_id should be between 0 & %d\n", __FUNCTION__, N_CLIENT-1);
        return -1;
    }
    if(!c[ch].data_fd[client_id]){
        fprintf(stderr, "ERROR in %s, channel %d client %d not initialized yet\n", __FUNCTION__, ch, client_id);
        return -1;
    }

    // lock mutex first
    pthread_mutex_lock(&c[ch].client_mtx[client_id]);

    // use ioctl to check
    int n_bytes;
    if(ioctl(c[ch].data_fd[client_id], FIONREAD, &n_bytes)){
        perror("ERROR in pipe_client_bytes_in_pipe");
        pthread_mutex_unlock(&mtx[ch]);
        return -1;
    }

    // done, unlock mutex
    pthread_mutex_unlock(&c[ch].client_mtx[client_id]);
    return n_bytes;
}


int pipe_server_get_pipe_size(int ch, int client_id)
{
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return -1;
    }
    if(client_id<0 || client_id>=N_CLIENT){
        fprintf(stderr, "ERROR in %s, client_id should be between 0 & %d\n", __FUNCTION__, N_CLIENT-1);
        return -1;
    }
    if(!c[ch].data_fd[client_id]){
        fprintf(stderr, "ERROR in %s, channel %d client %d not initialized yet\n", __FUNCTION__, ch, client_id);
        return -1;
    }

    pthread_mutex_lock(&c[ch].client_mtx[client_id]);
    int ret = fcntl(c[ch].data_fd[client_id], F_GETPIPE_SZ);
    pthread_mutex_unlock(&c[ch].client_mtx[client_id]);
    return ret;
}


int pipe_server_set_pipe_size(int ch, int client_id, int size_bytes)
{
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return -1;
    }
    if(client_id<0 || client_id>=N_CLIENT){
        fprintf(stderr, "ERROR in %s, client_id should be between 0 & %d\n", __FUNCTION__, N_CLIENT-1);
        return -1;
    }
    if(!c[ch].data_fd[client_id]){
        fprintf(stderr, "ERROR in %s, channel %d client %d not initialized yet\n", __FUNCTION__, ch, client_id);
        return -1;
    }

    // use fctl with mutex protection
    pthread_mutex_lock(&c[ch].client_mtx[client_id]);
    errno = 0;
    int new_size = fcntl(c[ch].data_fd[client_id], F_SETPIPE_SZ, size_bytes);
    pthread_mutex_unlock(&c[ch].client_mtx[client_id]);

    // error check
    if(new_size<size_bytes){
        perror("ERROR failed to set pipe size");
        if(errno == EPERM){
            fprintf(stderr, "You may need to be root to make a pipe that big\n");
        }
        // if fcntl fails, it may return 0 instead of the actual size, so fetch
        // the new size and return that instead.
        new_size = pipe_server_get_pipe_size(ch, client_id);
    }

    #ifdef __ANDROID__
        LOGI( "new pipe size %d", new_size);
    #endif

    // if fcntl was successful it returned the new size in bytes, so return it
    return new_size;
}


int pipe_server_set_control_pipe_size(int ch, int pipe_size, int read_buf_size)
{
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return -1;
    }
    if(c[ch].running){
        fprintf(stderr, "ERROR in %s, must set control pipe size before creating the pipe\n", __FUNCTION__);
        return -1;
    }
    if(pipe_size<0){
        fprintf(stderr, "ERROR in %s, pipe_size must be >=0\n", __FUNCTION__);
        return -1;
    }
    if(read_buf_size<0){
        fprintf(stderr, "ERROR in %s, read_buf_size must be >=0\n", __FUNCTION__);
        return -1;
    }
    if(pipe_size>(256*1024*1024)){
        fprintf(stderr, "WARNING in %s, trying to set default pipe size >256MiB probably won't work\n", __FUNCTION__);
    }
    c[ch].control_pipe_size = pipe_size;
    c[ch].control_read_buf_size = read_buf_size;
    return 0;
}


int pipe_server_set_control_cb(int ch, server_control_cb* cb, void* context)
{
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return -1;
    }
    pthread_mutex_lock(&mtx[ch]);
    c[ch].control_cb_context = context;
    c[ch].control_cb_func = cb;
    pthread_mutex_unlock(&mtx[ch]);
    return 0;
}


int pipe_server_set_control_thread_priority(int ch, int priority)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return PIPE_ERROR_CHANNEL_OOB;
    }
    if(priority<0 || priority>99){
        fprintf(stderr, "ERROR in %s, priority should be between 0 & 99\n", __FUNCTION__);
        return PIPE_ERROR_INVALID_ARG;
    }
    if(c[ch].running){
        fprintf(stderr, "ERROR in %s, call this before pipe_client_open()\n", __FUNCTION__);
        return PIPE_ERROR_OTHER;
    }

    pthread_mutex_lock(&mtx[ch]);
    c[ch].control_thread_priority = priority;
    pthread_mutex_unlock(&mtx[ch]);
    return 0;
}


int pipe_server_set_connect_cb(int ch, server_connect_cb* cb, void* context)
{
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return -1;
    }
    pthread_mutex_lock(&mtx[ch]);
    c[ch].connect_cb_context = context;
    c[ch].connect_cb_func = cb;
    pthread_mutex_unlock(&mtx[ch]);
    return 0;
}


int pipe_server_set_available_control_commands(int ch, const char* commands)
{
    //Server has not been initialized
    if(c[ch].info_json == NULL){
        return -1;
    }

    char buffer[2048];
    strcpy(buffer, commands);

    if(cJSON_HasObjectItem(c[ch].info_json, "available_commands")){
        cJSON_DeleteItemFromObject(c[ch].info_json, "available_commands");
    }

    cJSON* cmds = cJSON_CreateArray();

    cJSON_AddItemToObject(c[ch].info_json, "available_commands", cmds);

    // Extract the first token
    char * token = strtok(buffer, ",");
    // loop through the string to extract all other tokens
    while( token != NULL ) {
        cJSON_AddItemToArray(cmds, cJSON_CreateString(token));
        token = strtok(NULL, ",");
    }

    pipe_server_update_info(ch);

    return 0;

}


int pipe_server_set_disconnect_cb(int ch, server_disconnect_cb* cb, void* context)
{
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return -1;
    }
    pthread_mutex_lock(&mtx[ch]);
    c[ch].disconnect_cb_context = context;
    c[ch].disconnect_cb_func = cb;
    pthread_mutex_unlock(&mtx[ch]);
    return 0;
}


static int _disconnect_client(int ch, int client_id)
{
    // you can print this in the disconnect callback if you want to keep
    // this message but don't want to enable verbose debug mode.
    // see modal-hello-server for an example
    if(en_debug){
        fprintf(stderr,"Client %s (id %d) disconnected from channel %d\n", \
                                c[ch].client_names[client_id], client_id, ch);
    }
    // flag as disconnected
    c[ch].client_state[client_id] = CLIENT_DISCONNECTED;
    // delete the pipe indicating to other clients they can request this name
    close(c[ch].data_fd[client_id]);
    c[ch].data_fd[client_id] = 0;
    remove(c[ch].data_path[client_id]);

    // if ion buffers are enabled, then close and remove the sockets
    if((c[ch].flags & SERVER_FLAG_EN_ION_BUF) &&
        c[ch].client_sock_fd[client_id] != 0){
            shutdown(c[ch].client_sock_fd[client_id], SHUT_RDWR);
            close(c[ch].client_sock_fd[client_id]);
            c[ch].client_sock_fd[client_id] = 0;

            char sock_path[PATH_LEN];
            sprintf(sock_path, "%s%s", c[ch].data_path[client_id], ".sock");
            unlink(sock_path);
            remove(sock_path);
    }
    // call disconnect cb if user has set one
    if(c[ch].disconnect_cb_func!=NULL){
        c[ch].disconnect_cb_func(ch, client_id, c[ch].client_names[client_id], c[ch].disconnect_cb_context);
    }

    // This client will need new headers if it's recieving encoded video, and this cariable is unused if not so just set it
    c[ch].accepting_p_frames[client_id] = 0;

    // now the client is flagged as disconnected but we keep their name in
    // memory as a previously registered client, then we know if they reconnect
    return 0;
}


// This has no protection for pipe overflow, needs to be wrapped by something that
// does and also locks the mutex!!!! not a public function, internal only!
static int _pipe_server_write_to_client_nolock(int ch, int client_id, const void* data, int bytes)
{
    if(bytes<1){
        fprintf(stderr, "ERROR in %s, bytes should be >=1\n", __FUNCTION__);
        return -1;
    }

    // client has disconnected, nothing to write.
    if((c[ch].client_state[client_id] == CLIENT_DISCONNECTED) || \
        (c[ch].data_fd[client_id]<=0)){
        return -1;
    }

    // try a write!
    errno = 0;
    int result = write(c[ch].data_fd[client_id], data, bytes);

    // optional debug prints
    if(en_debug){
        fprintf(stderr, "write to ch: %d id: %d result: %d errno: %d\n", ch, client_id, result, errno);
        if(result!=bytes) perror("write error");
    }

    // write was good!
    if(result==bytes){
        if(en_debug && c[ch].client_state[client_id] != CLIENT_CONNECTED){
            fprintf(stderr, "client %d first good write of %d bytes\n", client_id, bytes);
        }
        // write was successfull! Flag the client as connected and return
        c[ch].client_state[client_id] = CLIENT_CONNECTED;
        return 0;
    }

    // partial write, possible if the pipe is full. Shouldn't happen anymore
    // since we started checking pipe capacity before writing
    if(result>0){
        fprintf(stderr, "WARNING PIPE FULL tried to write %d bytes but write returned %d\n",\
                                bytes, result);
        fprintf(stderr, "Likely client %s on pipe %s is struggling\n", \
                                c[ch].client_names[client_id], c[ch].base_dir);

        return -1;
    }

    // if we got here, write threw a real error, most likely client disconnected
    // TODO it's possible the CLIENT_INITIALIZED check should be replaced by a time
    // since initialization since the client may just still be opening the pipe for read
    int last_state = c[ch].client_state[client_id];
    if(last_state == CLIENT_CONNECTED || last_state == CLIENT_INITIALIZED){
        if(en_debug){
            printf("disconnecting client %s due to bad write\n", c[ch].client_names[client_id]);
        }
        _disconnect_client(ch, client_id);
    }
    return -1;
}


int pipe_server_write_to_client(int ch, int client_id, const void* data, int bytes)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return -1;
    }
    if(client_id<0 || client_id>=N_CLIENT){
        fprintf(stderr, "ERROR in %s, client_id should be between 0 & %d, received %d\n", __FUNCTION__, N_CLIENT-1, client_id);
        return -1;
    }
    if(bytes<1){
        fprintf(stderr, "ERROR in %s, bytes should be >=1\n", __FUNCTION__);
        return -1;
    }
    // client has disconnected, nothing to write.
    if((c[ch].client_state[client_id] == CLIENT_DISCONNECTED) || \
        (c[ch].data_fd[client_id]<=0)){
        return -1;
    }

    // lock the client mutex so another thread doesn't try to write to the pipe
    pthread_mutex_lock(&c[ch].client_mtx[client_id]);

    // check there is enough space
    // TODO this doesn't account for clients expanding the pipe themselves
    // but we should probably move away from letting them do that anyway
    int bytes_in_pipe = _pipe_server_bytes_in_pipe_nolock(ch, client_id);
    if(bytes > (c[ch].actual_pipe_size[client_id]-bytes_in_pipe)){
        if(en_debug){
            fprintf(stderr, "WARNING, client %s pipe backed up (in pipe %d)\n", c[ch].client_names[client_id], bytes_in_pipe);
        }
        pthread_mutex_unlock(&c[ch].client_mtx[client_id]);
        return -1;
    }

    // do a nolock write while inside the mutex
    int ret = _pipe_server_write_to_client_nolock(ch, client_id, data, bytes);

    pthread_mutex_unlock(&c[ch].client_mtx[client_id]);
    return ret;
}


int pipe_server_write(int ch, const void* data, int bytes)
{
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return -1;
    }
    if(bytes<=0){
        fprintf(stderr, "ERROR in %s, bytes to send must be >=0\n", __FUNCTION__);
        return -1;
    }
    // go through all clients, some may be disconnected, and that's okay.
    for(int i=0; i<c[ch].n_clients; i++){
        // note, this call locks the client mutex!
        pipe_server_write_to_client(ch, i, data, bytes);
    }
    return 0;
}


static int _pipe_server_write_list_client(int ch, int client_id, size_t total_bytes,\
                                 int nbuf, const void** bufs, const size_t* lens)
{
    // client has disconnected, nothing to write.
    if((c[ch].client_state[client_id] == CLIENT_DISCONNECTED) || \
        (c[ch].data_fd[client_id]<=0)){
        return -1;
    }

    pthread_mutex_lock(&c[ch].client_mtx[client_id]);

    // check there is enough space
    // TODO this doesn't account for clients expanding the pipe themselves
    // but we should probably move away from letting them do that anyway
    int bytes_in_pipe = _pipe_server_bytes_in_pipe_nolock(ch, client_id);
    if((int)total_bytes > (c[ch].actual_pipe_size[client_id]-bytes_in_pipe)){
        if(en_debug){
            fprintf(stderr, "WARNING, client %s pipe backed up (in pipe %d)\n", c[ch].client_names[client_id], bytes_in_pipe);
        }
        pthread_mutex_unlock(&c[ch].client_mtx[client_id]);
        return -1;
    }

    // write each chunk
    int ret = 0;
    for(int i=0; i<nbuf; i++) {
        ret = _pipe_server_write_to_client_nolock(ch, client_id, bufs[i], lens[i]);
        if(ret<0) break; // client probably disconnected
    }

    pthread_mutex_unlock(&c[ch].client_mtx[client_id]);

    return ret;
}


int pipe_server_write_list(int ch, int nbuf, const void** bufs, const size_t* lens)
{
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return -1;
    }
    if(nbuf<1){
        fprintf(stderr, "ERROR in %s, at least 1 buffer and length to send\n", __FUNCTION__);
        return -1;
    }
    if(bufs==NULL || lens==NULL){
        fprintf(stderr, "ERROR in %s, received NULL pointer\n", __FUNCTION__);
        return -1;
    }

    // count total bytes the user wants to write
    size_t total_bytes = 0;
    for(int i=0; i<nbuf; i++){
        total_bytes += lens[i];
        if(bufs[i]==NULL){
            fprintf(stderr, "ERROR in %s, received NULL pointer\n", __FUNCTION__);
            return -1;
        }
        if(lens[i]<1){
            fprintf(stderr, "ERROR in %s, each buffer should have >=1 bytes to transfer\n", __FUNCTION__);
            return -1;
        }
    }

    // go through all clients, some may be disconnected, and that's okay.
    // We're resetting the args each time, inefficient but
    for(int client_id=0; client_id<c[ch].n_clients; client_id++){
        int rc = _pipe_server_write_list_client(ch, client_id, total_bytes, nbuf, bufs, lens);
        if( rc < 0 && en_debug ){
            printf( "server.c %d Error writing pipe data to client %d rc: %d\n", __LINE__, client_id, rc);
        }
    }

    return 0;
}




#define FRAME_TYPE_HEADER 0
#define FRAME_TYPE_I      1
#define FRAME_TYPE_P      2
#define FRAME_TYPE_B      3

/**
 * Figure out if an h264/h265 frame is a header, I, P, or B frame
 */
static int _get_encoded_frame_type (camera_image_metadata_t meta, const void* data){

    switch(meta.format){
        case IMAGE_FORMAT_H264 :
            switch (((uint8_t*)data)[4]) {
                case 0x67 : //Header
                    return FRAME_TYPE_HEADER;
                case 0x65 : // I Frame
                    return FRAME_TYPE_I;
                case 0x41 : // P Frame
                    return FRAME_TYPE_P;
                default   : // TODO test to see what a b frame is
                    fprintf(stderr, "Recieved frame of unknown type for H264: 0x%x\n", ((uint8_t*)data)[4]);
                    return -1;
            }
            break;
        case IMAGE_FORMAT_H265 :
            switch (((uint8_t*)data)[4]) {
                case 0x40 : //Header
                    return FRAME_TYPE_HEADER;
                case 0x26 : // I Frame
                    return FRAME_TYPE_I;
                case 0x02 : // P Frame
                    return FRAME_TYPE_P;
                default   : // TODO test to see what a b frame is
                    fprintf(stderr, "Recieved frame of unknown type for H265: 0x%x\n", ((uint8_t*)data)[4]);
                    return -1;
            }
            break;
        default :
            fprintf(stderr, "Frames that are not encoded will not have an encoded type\n");
            return -1;
    }

}


int pipe_server_write_camera_frame(int ch, camera_image_metadata_t meta, const void* data)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return -1;
    }
    if(data==NULL){
        fprintf(stderr, "ERROR in %s, received NULL data pointer\n", __FUNCTION__);
        return -1;
    }
    if(meta.size_bytes<=0){
        fprintf(stderr, "ERROR in %s, metadata must specify a positive frame size in bytes\n", __FUNCTION__);
        return -1;
    }

    // set the magic number so the user doesn't have to
    meta.magic_number = CAMERA_MAGIC_NUMBER;

    // Encoded frames need to be sent in specific order, we'll handle this for the server
    if(meta.format == IMAGE_FORMAT_H264 || meta.format == IMAGE_FORMAT_H265){

        int frame_type = _get_encoded_frame_type(meta, data);

        // if we got a header, save it for when a client connects
        if(frame_type == FRAME_TYPE_HEADER){
            c[ch].encode_header = realloc(c[ch].encode_header, meta.size_bytes);
            memcpy(c[ch].encode_header, data, meta.size_bytes);
            c[ch].encode_header_meta = meta;
            for(int i=0; i<c[ch].n_clients; i++){
                c[ch].accepting_p_frames[i] = 0;
            }
        }

        // p frames can only be written to clients that already have a header and I frame
        else if(frame_type == FRAME_TYPE_P){

            // now just write to each client
            for(int i=0; i<c[ch].n_clients; i++){

                // skip clients that aren't accepting p frames yet
                if(!c[ch].accepting_p_frames[i]) continue;

                // TODO CHECK PIPE CAPACITY HERE
                // first try writing metadata
                int ret = pipe_server_write_to_client(ch, i, (char*)&meta, sizeof(camera_image_metadata_t));
                // only write the data if the metadata succeeded
                if(ret==0){
                    pipe_server_write_to_client(ch, i, data, meta.size_bytes);
                }
            }
            // already wrote to those clients, can return now
            return 0;
        }

        // I frames are unique in that we need to record that a client
        // actually received an I frame before sending P frames
        else if(frame_type == FRAME_TYPE_I){

            for(int i=0; i<c[ch].n_clients; i++){

                // TODO CHECK PIPE CAPACITY HERE
                // first try writing metadata
                int ret = pipe_server_write_to_client(ch, i, (char*)&meta, sizeof(camera_image_metadata_t));
                // only write the data if the metadata succeeded
                if(ret==0){
                    pipe_server_write_to_client(ch, i, data, meta.size_bytes);

                    // This client has now recieved an I frame so it can accept P frames
                    c[ch].accepting_p_frames[i] = 1;
                }
            }
            // already wrote to those clients, can return now
            return 0;
        }
    }

    // normal operation, write metadata and frame in one go
    const void* bufs[] = {&meta, data};
    size_t lens[] = {sizeof(camera_image_metadata_t), meta.size_bytes};
    return pipe_server_write_list(ch, 2, bufs, lens);
}


int pipe_server_write_stereo_frame(int ch, camera_image_metadata_t meta, const void* left, const void* right)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return -1;
    }
    if(left==NULL){
        fprintf(stderr, "ERROR in %s, received NULL data pointer\n", __FUNCTION__);
        return -1;
    }
    if(right==NULL){
        fprintf(stderr, "ERROR in %s, received NULL data pointer\n", __FUNCTION__);
        return -1;
    }
    if(meta.size_bytes<=0){
        fprintf(stderr, "ERROR in %s, metadata must specify a positive frame size in bytes\n", __FUNCTION__);
        return -1;
    }
    if(meta.size_bytes%2){
        fprintf(stderr, "ERROR in %s, metadata must specify an even number of bytes\n", __FUNCTION__);
        return -1;
    }

    // set the magic number so the user doesn't have to
    meta.magic_number = CAMERA_MAGIC_NUMBER;

     // normal operation, write metadata and frame in one go
    const void* bufs[] = {&meta, left, right};
    size_t lens[] = {sizeof(camera_image_metadata_t), meta.size_bytes/2, meta.size_bytes/2};
    return pipe_server_write_list(ch, 3, bufs, lens);
}


int pipe_server_write_point_cloud(int ch, point_cloud_metadata_t meta, const void* data)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return -1;
    }
    if(data==NULL){
        fprintf(stderr, "ERROR in %s, received NULL data pointer\n", __FUNCTION__);
        return -1;
    }

    // set the magic number so the user doesn't have to
    meta.magic_number = POINT_CLOUD_MAGIC_NUMBER;

    int size_bytes = pipe_point_cloud_meta_to_size_bytes(meta);
    if(size_bytes<0){
        fprintf(stderr, "ERROR in %s, bad metadata\n", __FUNCTION__);
        return -1;
    }

    // normal operation, write metadata and frame in one go
    const void* bufs[] = {&meta, data};
    size_t lens[] = {sizeof(point_cloud_metadata_t), size_bytes};
    return pipe_server_write_list(ch, 2, bufs, lens);
}


/**
 * @brief       Sends a file descriptor over a UNIX domain socket.
 *
 * @param[in]   socket the UNIX domain socket to send the file descriptor
 * @param[in]   fd     the file descriptor
 * 
 * @return      0 on success, or -1 on failure.
 */
static int send_fd(int socket, int fd)
{
    struct msghdr msg = {0};
    char buf[1] = {0};
    struct iovec iov = { .iov_base = buf, .iov_len = 1 };

    char control[CMSG_SPACE(sizeof(int))];
    msg.msg_control = control;
    msg.msg_controllen = sizeof(control);
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;

    struct cmsghdr *cmsg = CMSG_FIRSTHDR(&msg);
    cmsg->cmsg_level = SOL_SOCKET;
    cmsg->cmsg_type = SCM_RIGHTS;
    cmsg->cmsg_len = CMSG_LEN(sizeof(int));

    *((int *) CMSG_DATA(cmsg)) = fd;

    if (sendmsg(socket, &msg, 0) == -1) {
        perror("sendmsg");
        return -1;
    }

    return 0;
}


#ifdef EN_ION_BUF
int pipe_server_write_ion_buffer(int ch, mpa_ion_buf_pool_t* ion_pool, mpa_ion_buf_t* ion_buf)
{
    // sanity checks
    if(ch<0 || ch>=PIPE_SERVER_MAX_CHANNELS){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, PIPE_SERVER_MAX_CHANNELS-1);
        return -1;
    }
    if(!ion_buf){
        fprintf(stderr, "ERROR in %s, received NULL ion buf pointer\n", __FUNCTION__);
        return -1;
    }
    int fd = ion_buf->fd;
    if(fd<0){
        fprintf(stderr, "ERROR in %s, received invalid file descriptor: %d\n", __FUNCTION__, fd);
        return -1;
    }

    errno = 0;
    if (fcntl(fd, F_GETFD) == -1) {
        if (errno == EBADF) {
            fprintf(stderr, "ERROR in %s, %d is not valid (closed or uninitialized)\n", __FUNCTION__, fd);
        } else {
            perror("fcntl error, could not verify that fd is open");
        }
        return -1;
    }

    ion_buf->magic_number = MPA_ION_BUFFER_MAGIC_NUMBER;

    int clients_sent_to = 0;
    if(en_debug){
        printf("server preparing to send ion buffer id %d (fd %d) to clients, n_clients: %d, time: %ld\n",
               ion_buf->buffer_id, fd, c[ch].n_clients, _time_monotonic_ns());
    }

    for(int i=0; i<c[ch].n_clients; i++){
        pthread_mutex_lock(&c[ch].client_mtx[i]);

        if((c[ch].client_state[i] == CLIENT_DISCONNECTED) || (c[ch].data_fd[i]<=0)){
            pthread_mutex_unlock(&c[ch].client_mtx[i]);
            continue;
        }

        int sock = c[ch].client_sock_fd[i];
        int flag_as_disconnected = (sock < 0) ? 1 : 0;

        if(!flag_as_disconnected){        
            struct pollfd pfd;
            pfd.fd = sock;
            pfd.events = POLLOUT; // Check writability
            int ret = poll(&pfd, 1, 0); // Non-blocking poll
            bool ok_to_write = (ret > 0) &&
                               (pfd.revents & POLLOUT) &&
                               !(pfd.revents & (POLLERR | POLLHUP | POLLNVAL));
            if(!ok_to_write){
                if(en_debug){
                    fprintf(stderr, "client %d (ch %d) socket not writable, client likey dc'd\n", i, ch);
                }
                flag_as_disconnected = 1;
            }
        }
        
        if(!flag_as_disconnected && send_fd(sock, fd) < 0) {
            if(en_debug){
                fprintf(stderr, "send_fd failed for client %d (ch %d), client likey dc'd\n", i, ch);
            }
            flag_as_disconnected = 1;
        }

        if(flag_as_disconnected){
            _disconnect_client(ch, i);
            pthread_mutex_unlock(&c[ch].client_mtx[i]);
            continue;
        }

        pthread_mutex_unlock(&c[ch].client_mtx[i]);

        // TODO: check that this ion_pool_ref does not change, as we expect to only write
        // buffers that all come from the same pool
        c[ch].ion_pool_ref = ion_pool;
        int wr = pipe_server_write_to_client(ch, i, (const void*)ion_buf, sizeof(mpa_ion_buf_t));
        if(wr<0){
            if(en_debug){
                fprintf(stderr, "WARN: wrote FD but header write failed to client %d (ch %d)\n", i, ch);
            }
            continue;
        }

        pthread_mutex_lock(&ion_pool->pool_mtx);
        ion_pool->client_mask[ion_buf->buffer_id] |= (1 << i);
        ion_pool->n_client_references[ion_buf->buffer_id]++;
        pthread_mutex_unlock(&ion_pool->pool_mtx);
    }
    
    if(en_debug){
        printf("server sending ion buffer id %d to %d clients, n_clients: %d\n",
               ion_buf->buffer_id, clients_sent_to, c[ch].n_clients);
    }

    return 0;
}
#endif


int pipe_server_write_string(int ch, const char* string)
{
    int len = strlen(string);
    if(len<1){
        fprintf(stderr, "ERROR in %s, got empty string\n", __FUNCTION__);
        return -1;
    }
    return pipe_server_write(ch, string, len+1);
}


int pipe_server_get_client_state(int ch, int client_id)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return -1;
    }
    if(client_id<0 || client_id>=N_CLIENT){
        fprintf(stderr, "ERROR in %s, client_id should be between 0 & %d\n", __FUNCTION__, N_CLIENT-1);
        return -1;
    }

    return c[ch].client_state[client_id];
}


int pipe_server_get_num_clients(int ch)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return -1;
    }
    pthread_mutex_lock(&mtx[ch]);
    int n_connected = 0;

    // count the number of clients that are either connected or initialized.
    // This is because a client is not marked as connected until AFTER the first
    // successful transfer. Between the client sending the request and reading
    // the first packet they are instead "initialized".
    for(int i=0;i<c[ch].n_clients;i++){
        if( c[ch].client_state[i]==CLIENT_CONNECTED ||
            c[ch].client_state[i]==CLIENT_INITIALIZED ){
            n_connected++;
        }
    }
    pthread_mutex_unlock(&mtx[ch]);
    return n_connected;
}


int pipe_server_get_client_id_from_name(int ch, char* name)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return -1;
    }
    if(name==NULL){
        fprintf(stderr, "ERROR in %s, received NULL pointer\n", __FUNCTION__);
        return -1;
    }

    int ret = -1;
    pthread_mutex_lock(&mtx[ch]);

    // search through clients for a match
    for(int i=0; i<c[ch].n_clients; i++){
        if(strcmp(name, c[ch].client_names[i])==0){
            ret = i;
            break;
        }
    }
    pthread_mutex_unlock(&mtx[ch]);
    return ret;
}


char* pipe_server_get_client_name_from_id(int ch, int client_id)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return NULL;
    }
    if(client_id<0 || client_id>=N_CLIENT){
        fprintf(stderr, "ERROR in %s, client id must be >=0\n", __FUNCTION__);
        return NULL;
    }
    // This will automatically be NULL if client hasn't initialized yet
    return c[ch].client_names[client_id];
}


void pipe_server_close(int ch)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return;
    }

    // nothing to do if not running
    if(!c[ch].running) return;

    // this is the main use for the mutex, locking the channel during init/close
    pthread_mutex_lock(&mtx[ch]);

    // signal the request and control threads to stop and disable callbacks
    c[ch].running = 0;
    c[ch].n_clients = 0;
    c[ch].control_cb_func = NULL;
    c[ch].request_cb_func = NULL;
    c[ch].disconnect_cb_func = NULL;

    // send signal to request thread, this is just to make the blocking read quit
    pthread_kill(c[ch].request_thread, SIGUSR1);
    pthread_kill(c[ch].monitor_thread, SIGUSR1);

    // do a timed join, 1 second timeout
    struct timespec thread_timeout;
    clock_gettime(CLOCK_REALTIME, &thread_timeout);
    thread_timeout.tv_sec += 1;
    int ret = pthread_timedjoin_np(c[ch].request_thread, NULL, &thread_timeout);
    if(ret==ETIMEDOUT){
        fprintf(stderr, "WARNING, %s timed out joining request thread\n", __FUNCTION__);
    }

    clock_gettime(CLOCK_REALTIME, &thread_timeout);
    thread_timeout.tv_sec += 1;
    ret = pthread_timedjoin_np(c[ch].monitor_thread, NULL, &thread_timeout);
    if(ret==ETIMEDOUT){
        fprintf(stderr, "WARNING, %s timed out joining disconnect monitor thread\n", __FUNCTION__);
    }

    // cleanup request thread stuff
    close(c[ch].request_fd);
    remove(c[ch].request_path);

    // join control thread too if it was enabled
    if(c[ch].control_thread){
        // send signal to control thread, this is just to make the blocking read quit
        pthread_kill(c[ch].control_thread, SIGUSR1);

        // do a timed join, 1 second timeout
        clock_gettime(CLOCK_REALTIME, &thread_timeout);
        thread_timeout.tv_sec += 1;
        ret = pthread_timedjoin_np(c[ch].control_thread, NULL, &thread_timeout);
        if(ret==ETIMEDOUT){
            fprintf(stderr, "WARNING, %s timed out joining request thread\n", __FUNCTION__);
        }

        // cleanup control pipe stuff
        close(c[ch].control_fd);
        remove(c[ch].control_path);
    }

    // close the data pipes
    for(int i=0; i<N_CLIENT; i++){
        if(c[ch].data_fd[i] != 0){
            close(c[ch].data_fd[i]);
            c[ch].data_fd[i] = 0;
        }

        // if ion buffers are enabled, then shutdown and unlink client sockets
        if((c[ch].flags & SERVER_FLAG_EN_ION_BUF) && 
            c[ch].client_sock_fd[i] != 0){
                shutdown(c[ch].client_sock_fd[i], SHUT_RDWR);
                close(c[ch].client_sock_fd[i]);

                char sock_path[PATH_LEN];
                sprintf(sock_path, "%s%s", c[ch].client_names[i], ".sock");
                unlink(sock_path);
                remove(sock_path);
        }
    }

    // delete the pipe directory form the file system
    _remove_recursive(c[ch].base_dir);

    // all done, wipe the channel's data struct back to 0 and release the mutex
    _wipe_channel(ch);
    pthread_mutex_unlock(&mtx[ch]);

    // unclaim now we are done putting the channel back to a clean state
    pthread_mutex_lock(&claim_mtx);
    c[ch].claimed = 0;
    pthread_mutex_unlock(&claim_mtx);

    return;
}


void pipe_server_close_all(void)
{
    for(int i=0; i<N_CH; i++) pipe_server_close(i);
    return;
}



int pipe_suggest_cam_pipe_size(int16_t format, int width, int height)
{
    if(width<1 || height<1){
        fprintf(stderr, "ERROR in %s, invalid width or height\n", __FUNCTION__);
        return -1;
    }

    // make pipes big enough to fit at least 8 full images
    const int min_images = 8;
    int size;

    // For common formats suggest something more reasonable
    switch(format){
        case IMAGE_FORMAT_RAW8:
            size = width * height * min_images;
            break;
        case IMAGE_FORMAT_NV12:
            size = width * height * min_images * 3 / 2;
            break;
        case IMAGE_FORMAT_STEREO_RAW8:
            size = width * height * min_images * 2;
            break;
        case IMAGE_FORMAT_H264:
        case IMAGE_FORMAT_H265:
            size = width * height * min_images / 10;
            break;
        case IMAGE_FORMAT_RAW16:
            size = width * height * min_images * 2;
            break;
        case IMAGE_FORMAT_NV21:
            size = width * height * min_images * 3 / 2;
            break;
        case IMAGE_FORMAT_JPG:
            size = width * height * min_images / 5;
            break;
        case IMAGE_FORMAT_YUV422:
        case IMAGE_FORMAT_YUV420:
            size = width * height * min_images * 3 / 2;
            break;
        case IMAGE_FORMAT_RGB:
            size = width * height * min_images * 3;
            break;
        case IMAGE_FORMAT_FLOAT32:
            size = width * height * min_images * 4;
            break;
        case IMAGE_FORMAT_STEREO_NV21:
            size = width * height * min_images * 3;
            break;
        case IMAGE_FORMAT_STEREO_RGB:
            size = width * height * min_images * 3 * 2;
            break;
        case IMAGE_FORMAT_YUV422_UYVY:
            size = width * height * min_images * 3 / 2;
            break;
        case IMAGE_FORMAT_STEREO_NV12:
            size = width * height * min_images * 3;
            break;
        case IMAGE_FORMAT_RAW10:
            size = width * height * min_images * 5 / 4;
            break;
        case IMAGE_FORMAT_RAW12:
            size = width * height * min_images * 3 / 2;
            break;
        default:
            // unknown format, assume a bad case like RGB
            size = width * height * min_images * 3;
    }

    // limit pipe size to 128MB, any more is unreasonable and may lead to
    // memory fragmentation. also set minimum to 1mb
    const int max = 128 * 1024 * 1024;
    const int min = 1024 * 1024;
    if(size>max) size = max;
    if(size<min) size = min;

    // round up to the next power of 2
    size--; // Decrement n to handle exact powers of 2
    size |= size >> 1;
    size |= size >> 2;
    size |= size >> 4;
    size |= size >> 8;
    size |= size >> 16;
    size++; // Increment to get the next power of 2

    return size;
}




////////////////////////////////////////////////////////////////////////////////
// DEPRECATED FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

// DEPRECATED
void pipe_server_close_all_channels(void)
{
    pipe_server_close_all();
    return;
}

// DEPRECATED
void pipe_server_close_channel(int ch)
{
    pipe_server_close(ch);
    return;
}

// DEPRECATED
int pipe_server_init_channel(int ch, const char* topic, int flags)
{
    pipe_info_t info = PIPE_INFO_INITIALIZER;

    // clean up the pipe path location in case the user messed it up
    char dir[MODAL_PIPE_MAX_DIR_LEN];
    if(pipe_expand_location_string((char*)topic, dir)<0){
        fprintf(stderr, "ERROR in %s, invalid pipe location: %s\n", __FUNCTION__, info.location);
        return -1;
    }
    strcpy(info.location, dir);

    // find the start of the pipe name
    int dirlen = strlen(dir);
    int start = 0;
    for(int i=dirlen-3;i>=0;i--){
        if(dir[i]=='/'){
            start = i+1;
            break;
        }
    }
    // copy pipe name out of the full path
    if(start>0){
        memcpy(info.name, &dir[start], dirlen-start-1);
        info.name[dirlen-start-1] = 0;
    }

    return pipe_server_create(ch, info, flags);
}


// DEPRECATED
int pipe_server_send_to_channel(int ch, char* data, int bytes){
    return pipe_server_write(ch, data, bytes);
}

// DEPRECATED
int pipe_server_send_camera_frame_to_channel(int ch, camera_image_metadata_t meta, char* data)
{
    return pipe_server_write_camera_frame(ch, meta, data);
}

// DEPRECATED
int pipe_server_send_stereo_frame_to_channel(int ch, camera_image_metadata_t meta, char* left, char* right)
{
    return pipe_server_write_stereo_frame(ch, meta, left, right);
}

// DEPRECATED
int pipe_server_send_point_cloud_to_channel(int ch, point_cloud_metadata_t meta, float* data)
{
    return pipe_server_write_point_cloud(ch, meta, data);
}

// DEPRECATED
int pipe_server_send_to_client(int ch, int client_id, char* data, int bytes)
{
    return pipe_server_write_to_client(ch, client_id, data, bytes);
}

// DEPRECATED FUNCTION
int pipe_server_set_request_cb(int ch, server_request_cb* cb, void* context)
{
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return -1;
    }
    pthread_mutex_lock(&mtx[ch]);
    c[ch].request_cb_context = context;
    c[ch].request_cb_func = cb;
    pthread_mutex_unlock(&mtx[ch]);
    return 0;
}

// DEPRECATED
int pipe_server_set_default_pipe_size(int ch, int size_bytes)
{
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return -1;
    }
    if(size_bytes<0){
        fprintf(stderr, "ERROR in %s, size_bytes must be >=0\n", __FUNCTION__);
        return -1;
    }
    if(size_bytes>(256*1024*1024)){
        fprintf(stderr, "WARNING in %s, trying to set default pipe size >256MiB probably won't work\n", __FUNCTION__);
    }
    pthread_mutex_lock(&mtx[ch]);
    c[ch].info.size_bytes = size_bytes;
    pthread_mutex_unlock(&mtx[ch]);
    return 0;
}

// DEPRECATED
int pipe_server_set_info_string(__attribute__((unused))int ch, __attribute__((unused))const char* string)
{
    fprintf(stderr, "ERROR pipe_server_set_info_string() is now deprecated\n");
    fprintf(stderr, "Please use pipe_server_get_info_json_ptr() and\n");
    fprintf(stderr, "pipe_server_update_info_json() instead\n");
    return -1;

}
