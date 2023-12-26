
#pragma once

#define PIPE_SINK_MAX_CHANNELS  16

#define MODAL_PIPE_DEFAULT_BASE_DIR "/run/mpa/"
#define MODAL_PIPE_MAX_DIR_LEN      64

// Sensible limits of the length of directories and paths
#define MODAL_PIPE_MAX_NAME_LEN     32
#define MODAL_PIPE_MAX_PATH_LEN     (MODAL_PIPE_MAX_DIR_LEN + MODAL_PIPE_MAX_NAME_LEN)

// Flags that can be passed to pipe_sink_create()
#define SINK_FLAG_EN_SIMPLE_HELPER      (1<<0) // must provide a buffer length on init
#define SINK_FLAG_EN_DEBUG_PRINTS       (1<<1)

#ifndef F_LINUX_SPECIFIC_BASE
#define F_LINUX_SPECIFIC_BASE	1024
#endif

#ifndef F_SETPIPE_SZ
#define F_SETPIPE_SZ	(F_LINUX_SPECIFIC_BASE + 7)
#endif



typedef void sink_simple_cb(int ch, char* data, int bytes, void* context);

#ifdef __cplusplus
extern "C" {
#endif

int pipe_sink_create(int ch, const char* path, int flags, int pipe_size, int buf_len);
int pipe_sink_set_simple_cb(int ch, sink_simple_cb* cb, void* context);

#ifdef __cplusplus
}
#endif

int pthread_timedjoin_np(pthread_t thread, void **retval, const struct timespec *abstime);
