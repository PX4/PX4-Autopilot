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



#include <stdio.h>      // for fprintf
#include <fcntl.h>      // for O_WRONLY & O_NONBLOCK
#include <string.h>
#include <modal_json.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>

#include "modal_pipe_common.h"
#include "misc.h"


void pipe_print_error(int e)
{
    if(e == PIPE_ERROR_SERVER_NOT_AVAILABLE){
        fprintf(stderr, "Server not available\n");
    }
    else if(e == PIPE_ERROR_REACHED_MAX_NAME_INDEX){
        fprintf(stderr, "ERROR: Reached maximum number of clients with the same name\n");
    }
    else if(e == PIPE_ERROR_FILE_IO){
        fprintf(stderr, "ERROR: File IO\n");
    }
    else if(e == PIPE_ERROR_TIMEOUT){
        fprintf(stderr, "ERROR: timeout waiting for server\n");
    }
    else if(e == PIPE_ERROR_OTHER){
        fprintf(stderr, "ERROR: other\n");
    }
    else if(e == PIPE_ERROR_INVALID_ARG){
        fprintf(stderr, "ERROR: Invalid Argument\n");
    }
    else if(e == PIPE_ERROR_NOT_CONNECTED){
        fprintf(stderr, "ERROR: not connected\n");
    }
    else if(e == PIPE_ERROR_CTRL_NOT_AVAILABLE){
        fprintf(stderr, "ERROR: control pipe not available\n");
    }
    else if(e == PIPE_ERROR_INFO_NOT_AVAILABLE){
        fprintf(stderr, "ERROR: info pipe not available\n");
    }
    else if(e == PIPE_ERROR_CHANNEL_OOB){
        fprintf(stderr, "ERROR: channel out of bounds\n");
    }
    else if(e < 0){
        fprintf(stderr, "ERROR: unknown error\n");
    }
    // note, there is no final else here so that the client can call this
    // function even when there is no error (e>=0) and nothing will print out.
    return;
}




int pipe_exists(const char* name_or_location)
{
    // construct the full path to where the request FIFO should be
    char request_path[MODAL_PIPE_MAX_PATH_LEN];
    if(pipe_expand_location_string(name_or_location, request_path)){
        fprintf(stderr, "ERROR in %s invalid name_or_location\n", __FUNCTION__);
        return 0;
    }
    strcat(request_path,"request");
    // if request fifo exists, we assume the pipe exists
    return _exists(request_path);
}


int pipe_is_type(const char* name_or_location, const char* desired_type)
{
    // pipe_get_info_json should fail silently!
    cJSON* json = pipe_get_info_json(name_or_location);
    if(json == NULL){
        return 0;
    }

    // fetch the type string from the json data
    char type[MODAL_PIPE_MAX_TYPE_LEN];
    int ret = json_fetch_string(json, "type", type, MODAL_PIPE_MAX_TYPE_LEN);
    if(ret){
        return 0;
    }

    // check if the types match!
    if(strcmp(type,desired_type) == 0){
        return 1;
    }
    return 0;
}



int pipe_get_info(const char* name_or_location, pipe_info_t* info)
{
    // pipe_get_info_json should fail silently!
    // Silently fail!! this function will be used in cases where failure
    // is normal and expected
    cJSON* json = pipe_get_info_json(name_or_location);
    if(json == NULL){
        return -1;
    }

    // fetch everything from the json data
    json_fetch_string_with_default(json,"name",        info->name,        MODAL_PIPE_MAX_NAME_LEN, "unknown");
    json_fetch_string_with_default(json,"location",    info->location,    MODAL_PIPE_MAX_DIR_LEN,  "unknown");
    json_fetch_string_with_default(json,"type",        info->type,        MODAL_PIPE_MAX_TYPE_LEN, "unknown");
    json_fetch_string_with_default(json,"server_name", info->server_name, MODAL_PIPE_MAX_NAME_LEN, "unknown");
    json_fetch_int_with_default(   json,"size_bytes", &info->size_bytes,  -1);
    json_fetch_int_with_default(   json,"server_pid", &info->server_pid,  -1);

    // free up memory!!!!!
    cJSON_Delete(json);

    if(json_get_parse_error_flag()){
        fprintf(stderr, "WARNING, encountered issues parsing json info\n");
    }
    return 0;
}



cJSON* pipe_get_info_json(const char* name_or_location)
{
    // first make sure the pipe exists before trying to read
    // Silently fail!! this function will be used in cases where failure
    // is normal and expected
    if(!pipe_exists(name_or_location)){
        return NULL;
    }

    // construct path to the info file
    char info_path[MODAL_PIPE_MAX_PATH_LEN];
    if(pipe_expand_location_string(name_or_location, info_path)){
        fprintf(stderr, "ERROR in %s\n", __FUNCTION__);
        return NULL;
    }
    strcat(info_path,"info");

    // use libmodal_json to read and parse the
    cJSON* json = json_read_file(info_path);
    if(json == NULL){
        fprintf(stderr, "ERROR in %s, failed to read info file\n", __FUNCTION__);
    }
    return json;
}


int pipe_expand_location_string(const char* in, char* out)
{
    // sanity checks
    if(in==NULL || out==NULL){
        fprintf(stderr, "ERROR in %s, received NULL pointer\n", __FUNCTION__);
        return PIPE_ERROR_INVALID_ARG;
    }

    int len = strlen(in);

    // TODO test for more edge cases
    if(len<1){
        fprintf(stderr, "ERROR in %s, recevied empty string\n", __FUNCTION__);
        return PIPE_ERROR_INVALID_ARG;
    }
    if(len==1 && in[0]=='/'){
        fprintf(stderr, "ERROR in %s, pipe path can't just be root '/'\n", __FUNCTION__);
        return PIPE_ERROR_INVALID_ARG;
    }

    len = 0;
    // if user didn't provide a full path starting with '/' then prepend the
    // default path and write out, recording the new length.
    if(in[0]!='/'){
        len += sprintf(out,"%s", MODAL_PIPE_DEFAULT_BASE_DIR);
    }
    // write out the full path provided (excluding non-printable characters)
    for(int i = 0; in[i] != 0; i++){
        if(in[i] > ' ' && in[i] <= '~') out[len++] = in[i];
    }
    out[len] = 0;


    // make sure the path ends in '/' since it should be a directory
    if(out[len-1]!='/'){
        out[len]='/';
        out[len+1]=0;
    }

    return 0;
}



int pipe_kill_server_process(const char* name_or_location, float timeout_s)
{
    if(timeout_s<0.1f){
        fprintf(stderr, "ERROR in %s timeout_s must be >= 0.1f\n", __FUNCTION__);
        return -4;
    }

    char location[MODAL_PIPE_MAX_DIR_LEN];
    if(pipe_expand_location_string(name_or_location, location)){
        fprintf(stderr, "ERROR in %s invalid name_or_location\n", __FUNCTION__);
        return -4;
    }

    // if pipe doesn't exist, all done
    if(!pipe_exists(location)){
        _remove_recursive(location);
        return 0;
    }

    // pipe does exist, load info
    pipe_info_t info;
    if(pipe_get_info(name_or_location, &info)){
        fprintf(stderr, "ERROR in %s failed to read pipe info\n", __FUNCTION__);
        _remove_recursive(location);
        return -2;
    }


    int old_pid = info.server_pid;
    if(old_pid<=0){
        fprintf(stderr, "ERROR in %s failed to read pipe info\n", __FUNCTION__);
        _remove_recursive(location);
        return -2;
    }


    // now see if the process for the read pid is still running
    if(getpgid(old_pid) < 0){
        // process not running
        fprintf(stderr, "WARNING in %s, PID pipe exists but value points to process that's not running\n", __FUNCTION__);
        _remove_recursive(location);
        return 0;
    }

    // process must be running, attempt a clean shutdown
    if(kill((pid_t)old_pid, SIGINT)==-1){
        if(errno==EPERM){
            fprintf(stderr, "ERROR in %s, insufficient permissions to stop\n", __FUNCTION__);
            fprintf(stderr, "an existing process which is probably running as root.\n");
            _remove_recursive(location);
            return -3;
        }
        _remove_recursive(location);
        return -2;
    }

    // check every 0.1 seconds to see if it closed
    int num_checks=timeout_s/0.1f;
    int i;
    for(i=0; i<=num_checks; i++){
        // check if PID has stopped
        if(getpgid(old_pid)==-1){
            // succcess, it shut down properly
            _remove_recursive(location);
            return 1;
        }
        else usleep(100000);
    }

    // otherwise force kill the program if the PID file never got cleaned up
    kill((pid_t)old_pid, SIGKILL);
    for(i=0; i<=num_checks; i++){
        // check if PID has stopped
        if(getpgid(old_pid)==-1){
            // succcess, it shut down properly
            _remove_recursive(location);
            return 1;
        }
        else usleep(100000);
    }

    // return -1 indicating the program had to be killed
    fprintf(stderr, "WARNING in %s, process failed to close cleanly and had to be killed.\n", __FUNCTION__);
    _remove_recursive(location);
    return -1;
}


int pipe_send_control_cmd(const char* pipe_name, const void* control_cmd)
{
    // construct path to the info file
    char path[MODAL_PIPE_MAX_PATH_LEN];
    if(pipe_expand_location_string(pipe_name, path)){
        fprintf(stderr, "ERROR in %s\n", __FUNCTION__);
        return -1;
    }
    strcat(path,"control");

    int fd = open(path, O_WRONLY|O_NONBLOCK);
    if(fd<0) return -1;

    int len = strlen(control_cmd)+1;
    if(write(fd, control_cmd, len)!=len){
        perror("ERROR writing to control pipe");
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}



int pipe_send_control_cmd_bytes(const char* pipe_name, const void* data, int bytes)
{
    // construct path to the info file
    char path[MODAL_PIPE_MAX_PATH_LEN];
    if(pipe_expand_location_string(pipe_name, path)){
        fprintf(stderr, "ERROR in %s\n", __FUNCTION__);
        return -1;
    }
    strcat(path,"control");

    int fd = open(path, O_WRONLY|O_NONBLOCK);
    if(fd<0) return -1;

    if(write(fd, data, bytes)!=bytes){
        perror("ERROR writing to control pipe");
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}




int write_fault_code(fault_t *fault)
{
    int fd = open(FAULT_PATH, O_WRONLY|O_NONBLOCK);
    if(fd<0){
        perror("couldn't open fault pipe:");
        fprintf(stderr, "make sure voxl-fault-manager service is running\n");
        return -1;
    }

    // write the raw struct and check for error
    int bytes = write(fd, fault, sizeof(fault_t));
    if(bytes < (int)sizeof(fault_t)){
        perror("error writing to pipe:");
        close(fd);
        return -1;
    }

    // close and quit
    printf("closing %s\n", FAULT_PATH);
    close(fd);
    return 0;

}
