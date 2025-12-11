# RTCM Injection - Next Task: Add DEBUG_RTCM_INJECT Compile Flag

## Status: READY FOR IMPLEMENTATION

The data loss bug has been fixed. Now we need to put the instrumentation code behind a compile-time flag so we can test without the extra printouts affecting timing.

---

## Task: Wrap Instrumentation in DEBUG_RTCM_INJECT

### Goal
All diagnostic/instrumentation code should be compiled only when `DEBUG_RTCM_INJECT` is defined. This includes:
- Periodic stats printouts (PX4_INFO for frame counts, rates, latencies)
- Diagnostic counters and their reporting
- CPU load monitoring
- RTCM frame counting parsers (used only for stats)

### Files to Modify

#### 1. `src/drivers/gps/gps.cpp`

Wrap the following in `#ifdef DEBUG_RTCM_INJECT`:
- Diagnostic counter variables (`_rtcm_uorb_messages`, `_rtcm_uorb_gaps`, `_rtcm_parser_bytes_dropped`, `_rtcm_tx_not_ready`, `_rtcm_tx_blocked`)
- Latency tracking variables (`_inject_latency_min/max/sum/count`, `_last_inject_orb_timestamp`)
- TX injection log (`_inject_log[]`, `_inject_log_idx`, `_inject_log_count`, `_inject_bytes_total`)
- CPU load tracking (`_last_cpu_check_time`, `_last_cpu_runtime`)
- RX buffer high water mark (`_rx_buf_high_water`)
- RTCM TX frame counting (`_rtcm_tx_frame_count`, etc.)
- All PX4_INFO stats printing (in the 5-second reporting block)
- `getTaskCpuLoad()` function

Keep the following **always enabled** (not behind flag):
- The core `drainRTCMFromORB()` fix (use update() only)
- The `_rtcm_uorb_gaps++` counter increment and PX4_WARN (useful for detecting real issues)
- The `_rtcm_parser_bytes_dropped` increment and PX4_WARN (useful for detecting real issues)

#### 2. `src/drivers/uavcannode/Subscribers/MovingBaselineData.hpp`

Wrap the following in `#ifdef DEBUG_RTCM_INJECT`:
- `_rtcm_counter` parser instance
- RTCM frame counting variables (`_rtcm_frame_count`, etc.)
- MBD message stats (`_msg_count`, `_msg_count_per_interval`, `_bytes_count`)
- Stats printing block (every 5 seconds)

Keep **always enabled**:
- `_publish_failures` counter and its warning (detects real issues)

#### 3. `src/drivers/uavcannode/Publishers/MovingBaselineData.hpp`

Wrap the following in `#ifdef DEBUG_RTCM_INJECT`:
- RTCM frame counting parser
- All stats variables
- Stats printing block

#### 4. `src/drivers/uavcan/sensors/gnss.cpp` (FC)

Wrap the following in `#ifdef DEBUG_RTCM_INJECT`:
- RTCM LOG frame counting
- Stats printing

---

## Implementation Pattern

Use this pattern consistently:

```cpp
#ifdef DEBUG_RTCM_INJECT
    // Stats variables
    uint32_t _rtcm_frame_count{0};
    // ...
#endif

void someFunction() {
#ifdef DEBUG_RTCM_INJECT
    _rtcm_frame_count++;
#endif

    // Core logic always runs
    doImportantWork();

#ifdef DEBUG_RTCM_INJECT
    if (now > _last_stats_time + 5000000ULL) {
        PX4_INFO("stats...");
    }
#endif
}
```

---

## Testing

1. Build **without** `DEBUG_RTCM_INJECT` - verify no stats printouts, smaller binary
2. Build **with** `DEBUG_RTCM_INJECT` - verify all stats work as before
3. Test both builds for correct RTCM injection (no data loss)

---

## How to Enable the Flag

For development/debugging, add to board config or compile command:
```bash
# In CMakeLists.txt or board config:
add_definitions(-DDEBUG_RTCM_INJECT)

# Or via command line:
make ark_can-rtk-gps_default CMAKE_EXTRA_FLAGS="-DDEBUG_RTCM_INJECT=1"
```

---

## Summary of What Stays vs What Gets Wrapped

### Always Enabled (Critical for Correctness)
- `drainRTCMFromORB()` using only `update()` (the bug fix)
- Generation gap detection with `_rtcm_uorb_gaps++` and PX4_WARN
- Parser buffer overflow detection with `_rtcm_parser_bytes_dropped` and PX4_WARN
- Publish failure counter `_publish_failures` and PX4_WARN

### Behind DEBUG_RTCM_INJECT (Instrumentation Only)
- All PX4_INFO periodic stats
- Frame counting parsers
- Latency tracking
- CPU load monitoring
- TX timing stats
- RX high water mark
