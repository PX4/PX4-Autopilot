#ifndef CPU_MONITOR_INTERFACE_H
#define CPU_MONITOR_INTERFACE_H

#ifdef __cplusplus
extern "C"
{
#endif


#include <stdint.h>
#include <time.h>


// commands that can be sent to the voxl-cpu-monitor control pipe
// be sure to update the CONTROL_COMMANDS array below if you add anything here
#define COMMAND_SET_CPU_MODE_PERF   "set_cpu_mode_perf"
#define COMMAND_SET_CPU_MODE_AUTO   "set_cpu_mode_auto"
#define COMMAND_SET_CPU_MODE_POWERSAVE      "set_cpu_mode_powersave"
#define COMMAND_SET_CPU_MODE_CONSERVATIVE   "set_cpu_mode_conservative"
#define COMMAND_SET_CPU_MODE_COOL   "set_cpu_mode_cool"
#define COMMAND_SET_FAN_MODE_OFF    "set_fan_mode_off"
#define COMMAND_SET_FAN_MODE_SLOW   "set_fan_mode_slow"
#define COMMAND_SET_FAN_MODE_MAX    "set_fan_mode_max"
#define COMMAND_SET_FAN_MODE_AUTO   "set_fan_mode_auto"
#define COMMAND_SET_LED_MODE_OFF    "set_led_mode_off"
#define COMMAND_SET_LED_MODE_ON     "set_led_mode_on"
#define COMMAND_SET_LED_MODE_COLOR  "set_led_mode_color"
#define COMMAND_SET_CPU_MODE_PERF_WITH_PITMODE   "set_cpu_mode_perf_with_pitmode"
#define COMMAND_SET_CPU_MODE_AUTO_WITH_PITMODE   "set_cpu_mode_auto_with_pitmode"

// pitmode keeps the CPU clocked down and cool when drone is disarmed and sitting still
// it is automatic by defualt based on arming state but can be toggled on and off
// or put back to auto with voxl-set-cpu-pitmode for development
#define COMMAND_SET_PITMODE_ON      "set_pitmode_on"
#define COMMAND_SET_PITMODE_OFF     "set_pitmode_off"
#define COMMAND_SET_PITMODE_AUTO    "set_pitmode_auto"

// THESE ARE DEPRECATED, use PITMODE INSTEAD
#define COMMAND_SET_STANDBY_ON      "set_standby_on"
#define COMMAND_SET_STANDBY_OFF     "set_standby_off"
#define COMMAND_SET_STANDBY_AUTO    "set_standby_auto"

#ifndef CTYPESGEN

// Be sure to update these if you add anything to the above list!
#define CPU_MON_CONTROL_COMMANDS (\
COMMAND_SET_CPU_MODE_POWERSAVE"," \
COMMAND_SET_CPU_MODE_CONSERVATIVE"," \
COMMAND_SET_CPU_MODE_PERF"," \
COMMAND_SET_CPU_MODE_AUTO"," \
COMMAND_SET_CPU_MODE_COOL"," \
COMMAND_SET_FAN_MODE_OFF"," \
COMMAND_SET_FAN_MODE_SLOW"," \
COMMAND_SET_FAN_MODE_MAX"," \
COMMAND_SET_FAN_MODE_AUTO"," \
COMMAND_SET_LED_MODE_OFF"," \
COMMAND_SET_LED_MODE_ON"," \
COMMAND_SET_LED_MODE_COLOR"," \
COMMAND_SET_PITMODE_ON"," \
COMMAND_SET_PITMODE_OFF"," \
COMMAND_SET_PITMODE_AUTO"," \
COMMAND_SET_CPU_MODE_PERF_WITH_PITMODE"," \
COMMAND_SET_CPU_MODE_AUTO_WITH_PITMODE)

#endif // CTYPESGEN

#include "magic_number.h"
#ifndef CPU_MON_MAGIC_NUMBER
#error "CPU_MON_MAGIC_NUMBER not defined!"
#endif

#define CPU_MON_MAX_CPU         8 // packet allows up to 8 cores

// thresholds to trigger overheat and overload flags
#define CPU_STATS_CPU_OVERLOAD_THRESHOLD      80.0f   // %80 total cpu use to trigger CPU Overload flag
#define CPU_STATS_CPU_OVERHEAT_THRESHOLD      85.0f   // DEPRECATED DO NOT USE
#define CPU_STATS_CPU_OVERHEAT_WARN_THRESHOLD 85.0f   // 85C max cpu temp to trigger CPU Overheat Warning flag
#define CPU_STATS_CPU_OVERHEAT_CRIT_THRESHOLD 95.0f   // 95C max cpu temp to trigger CPU Overheat Crtical flag

// flags that can be set in cpu_stats_t
#define CPU_STATS_FLAG_CPU_MODE_AUTO     (1<<0)  // indicates CPU clock scaling is in Auto mode
#define CPU_STATS_FLAG_CPU_MODE_PERF     (1<<1)  // indicates CPU clock scaling is in performance mode
#define CPU_STATS_FLAG_GPU_MODE_AUTO     (1<<2)  // indicates GPU clock scaling is in Auto mode
#define CPU_STATS_FLAG_GPU_MODE_PERF     (1<<3)  // indicates GPU clock scaling is in performance mode
#define CPU_STATS_FLAG_CPU_OVERLOAD      (1<<4)  // indicates the total CPU usage is over %80 use
#define CPU_STATS_FLAG_CPU_OVERHEAT      (1<<5)  // DEPRECATED
#define CPU_STATS_FLAG_CPU_OVERHEAT_WARN (1<<5)  // CPU temperature warning, not critical
#define CPU_STATS_FLAG_CPU_MODE_POWERSAVE    (1<<6)  // indicates CPU clock scaling is in powersave mode
#define CPU_STATS_FLAG_CPU_MODE_CONSERVATIVE (1<<7)  // indicates CPU clock scaling is in conservative mode
#define CPU_STATS_FLAG_STANDBY_ACTIVE    (1<<8)  // DEPRECATED
#define CPU_STATS_FLAG_PITMODE_ACTIVE    (1<<8)  // indicates pitmode is active
#define CPU_STATS_FLAG_CPU_MODE_COOL     (1<<9)  // indicates CPU clock scaling is in cool mode
#define CPU_STATS_FLAG_PITMODE_AUTOMATIC (1<<10) // indicates pitmode is toggling automatically
#define CPU_STATS_FLAG_CPU_OVERHEAT_CRIT (1<<11) // indicates critical overheating
#define CPU_STATS_FLAG_MEM_LEAK_DETECTED (1<<12) // indicates potential memory leak detected


////////////////////////////////////////////////////////////////////////////////
// This is the old cpu_stats_t struct. Nothing wrong with it, and it will
// continue to be published. Use cpu_stats2_t for more info.
////////////////////////////////////////////////////////////////////////////////
typedef struct cpu_stats_t{
    uint32_t magic_number;      ///< unique 32-bit number used to signal the beginning of a packet
    int32_t  num_cpu;           ///< number of CPU's
    float    cpu_freq[CPU_MON_MAX_CPU]; ///< CPU freq (MHz)
    float    cpu_t[CPU_MON_MAX_CPU];    ///< CPU temperature (C)
    float    cpu_t_max;         ///< max core temp (C)
    float    cpu_load[CPU_MON_MAX_CPU]; ///< CPU load (%)
    float    cpu_load_10s;      ///< CPU load for past 10 seconds (%)
    float    total_cpu_load;    ///< calculate total cpu load (%)
    float    gpu_freq;          ///< GPU freq (MHz)
    float    gpu_t;             ///< GPU temperature (C)
    float    gpu_load;          ///< current gpu load (%)
    float    gpu_load_10s;      ///< gpu load for past 10 seconds (%)
    float    mem_t;             ///< Memory Temperature
    uint32_t mem_total_mb;      ///< total memory in MB
    uint32_t mem_use_mb;        ///< memory used by processes in MB, not including cache
    uint32_t flags;             ///< flags
    uint32_t reserved;          ///< spare reserved bytes for future expansion
}__attribute__((packed)) cpu_stats_t;


/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync. Here we use 13 packets because the
 * cpu_stats_t packet is 152-bytes long and this gives a buffer just under 2k
 *
 * Note this is NOT the size of the pipe which can hold more. This is just the
 * read buffer size allocated on the heap into which data from the pipe is read.
 */
#define CPU_STATS_RECOMMENDED_READ_BUF_SIZE (sizeof(cpu_stats_t) * 13)


/**
 * We recommend cpu_stats servers use a pipe size of 32kB
 */
#define CPU_STATS_RECOMMENDED_PIPE_SIZE (32*1024)


/**
 * @brief      Use this to simultaneously validate that the bytes from a pipe
 *             contains valid data, find the number of valid packets
 *             contained in a single read from the pipe, and cast the raw data
 *             buffer as an cpu_stats_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate a cpu_stats_t array separate from the pipe read buffer.
 *             The data can be read straight out of the pipe read buffer, much
 *             like reading data directly out of a mavlink_message_t message.
 *
 *             However, this does mean the user should finish processing this
 *             data before returning the pipe data callback which triggers a new
 *             read() from the pipe.
 *
 * @param[in]  data       pointer to pipe read data buffer
 * @param[in]  bytes      number of bytes read into that buffer
 * @param[out] n_packets  number of valid packets received
 *
 * @return     Returns the same data pointer provided by the first argument, but
 *             cast to an tag_detection_t* struct for convenience. If there was an
 *             error then NULL is returned and n_packets is set to 0
 */
#ifndef CTYPESGEN
#include <stdio.h>
static inline cpu_stats_t* modal_cpu_validate_pipe_data(char* data, int bytes, int* n_packets)
{
    cpu_stats_t* new_ptr = (cpu_stats_t*) data;
    *n_packets = 0;

    // basic sanity checks
    if(bytes<0){
        fprintf(stderr, "ERROR validating cpu monitor data received through pipe: number of bytes = %d\n", bytes);
        return NULL;
    }
    if(data==NULL){
        fprintf(stderr, "ERROR validating cpu monitor data received through pipe: got NULL data pointer\n");
        return NULL;
    }
    if(bytes%sizeof(cpu_stats_t)){
        fprintf(stderr, "ERROR validating cpu monitor data received through pipe: read partial packet\n");
        fprintf(stderr, "read %d bytes, but it should be a multiple of %zu\n", bytes, sizeof(cpu_stats_t));
        return NULL;
    }

    // calculate number of packets locally until we validate each packet
    int n_packets_tmp = bytes/sizeof(cpu_stats_t);

    // check if any packets failed the magic number check
    int i, n_failed = 0;
    for(i=0;i<n_packets_tmp;i++){
        if(new_ptr[i].magic_number != CPU_MON_MAGIC_NUMBER) n_failed++;
    }
    if(n_failed>0){
        fprintf(stderr, "ERROR validating cpu monitor data received through pipe: %d of %d packets failed\n", n_failed, n_packets_tmp);
        return NULL;
    }

    *n_packets = n_packets_tmp;
    return new_ptr;
}
#endif // CTYPESGEN





////////////////////////////////////////////////////////////////////////////////
// NEW CPU_STATS2_T type, USE THIS!
////////////////////////////////////////////////////////////////////////////////


#include "magic_number.h"
#ifndef CPU_STATS2_MAGIC_NUMBER
#error "CPU_STATS2_MAGIC_NUMBER not defined!"
#endif

#define CPU_STATS2_N_PROCS          10
#define CPU_STATS2_PROC_NAME_LEN    24


typedef struct mpa_process_info_t {
    char name[CPU_STATS2_PROC_NAME_LEN];    ///< full process name from /proc/pid/comm
    pid_t pid;                  ///< process ID
    uint32_t memory_kb;         ///< current rss memory use from /proc/pid/stat
    float cpu_usage;            ///< percent use of a core over past second
    float cpu_usage_avg;        ///< percent use of a core over past 10 seconds
    uint32_t uptime_s;          ///< process uptime in seconds
    int8_t core;                ///< last used cpu core
    int8_t rt_priority;         ///< current RT priority if using SCHED_FIFO
    uint8_t reserved1;          ///< reserved for future use
    uint8_t reserved2;          ///< reserved for future use
}__attribute__((packed)) mpa_process_info_t;




/**
* newer extended version of  cpu_stats_t
*/
typedef struct cpu_stats2_t{
    uint32_t magic_number;      ///< unique 32-bit number used to signal the beginning of a packet
    float    cpu_freq[CPU_MON_MAX_CPU]; ///< CPU freq (MHz)
    float    cpu_t[CPU_MON_MAX_CPU];    ///< CPU temperature (C)
    float    cpu_t_max;         ///< max core temp (C)
    float    cpu_load[CPU_MON_MAX_CPU]; ///< CPU load (%)
    float    cpu_load_10s;      ///< CPU load for past 10 seconds (%)
    float    total_cpu_load;    ///< calculate total cpu load (%)
    float    gpu_freq;          ///< GPU freq (MHz)
    float    gpu_t;             ///< GPU temperature (C)
    float    gpu_load;          ///< current gpu load (%)
    float    gpu_load_10s;      ///< gpu load for past 10 seconds (%)
    float    mem_t;             ///< Memory Temperature
    uint32_t mem_total_kb;      ///< total memory in KB
    uint32_t mem_use_kb;        ///< memory used by processes in KB, not including cache
    uint32_t flags;             ///< flags
    uint32_t uptime_s;          ///< system uptime in seconds
    time_t   clock_time;        ///< seconds since epoch, set by time() in time.h
    mpa_process_info_t top_cpu[CPU_STATS2_N_PROCS]; ///< top consumers of cpu cycles
    mpa_process_info_t top_mem[CPU_STATS2_N_PROCS]; ///< top consumers of memory
    float    mem_rate_mbps;     ///< rate that memory is being allocated in MB per second
    uint32_t reserved1;         ///< spare reserved bytes for future expansion
    uint32_t reserved2;         ///< spare reserved bytes for future expansion
    uint32_t reserved3;         ///< spare reserved bytes for future expansion
}__attribute__((packed)) cpu_stats2_t;




/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync.
 *
 * Note this is NOT the size of the pipe which can hold more. This is just the
 * read buffer size allocated on the heap into which data from the pipe is read.
 */
#define CPU_STATS2_RECOMMENDED_READ_BUF_SIZE    (sizeof(cpu_stats2_t) * 10)


/**
 * We recommend cpu_stats2_t servers use a pipe size of 64kB
 */
#define CPU_STATS2_RECOMMENDED_PIPE_SIZE    (64*1024)



/**
 * @brief      Use this to simultaneously validate that the data from a pipe
 *             contains valid imu data, find the number of valid packets
 *             contained in a single read from the pipe, and cast the raw data
 *             buffer as an imu_data_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate an imu_data_t array separate from the pipe read buffer.
 *             The data can be read straight out of the pipe read buffer, much
 *             like reading data directly out of a mavlink_message_t message.
 *
 *             However, this does mean the user should finish processing this
 *             data before returning the pipe data callback which triggers a new
 *             read() from the pipe.
 *
 * @param[in]  data       pointer to pipe read data buffer
 * @param[in]  bytes      number of bytes read into that buffer
 * @param[out] n_packets  number of valid imu_data_t packets received
 *
 * @return     Returns the same data pointer provided by the first argument, but
 *             cast to a cpu_stats2_t* struct for convenience. If there was an
 *             error then NULL is returned and n_packets is set to 0
 */
cpu_stats2_t* pipe_validate_cpu_stats2_t(char *data, int bytes, int *n_packets);




#ifdef __cplusplus
}
#endif


#endif // CPU_MONITOR_INTERFACE_H
