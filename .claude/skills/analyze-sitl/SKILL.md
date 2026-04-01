---
name: analyze-sitl
description: Analyze PX4 SITL test logs after running run_sitl_test.sh. Spawns 3 agents in parallel to analyze SITL log, Python test log, and firmware code.
---

Analyze SITL test logs and find root cause of failures.

The user has just run `bash Tools/run_sitl_test.sh` and provided a RUN_ID.

Log files:
- SITL log: `/tmp/sitl_$ARGUMENTS.log`
- Test log: `/tmp/test_$ARGUMENTS.log`

If no RUN_ID provided, list available log pairs:
```bash
ls /tmp/sitl_*.log 2>/dev/null | sort | tail -5
```

## Step 1 — Lead Agent reads both log files FIRST

Before spawning any agents, YOU (Lead Agent) must read both files directly:
- Read `/tmp/sitl_$ARGUMENTS.log` (use Read tool, up to 2000 lines)
- Read `/tmp/test_$ARGUMENTS.log` (use Read tool)

Store the full content. You will paste it into each agent's prompt below.

## Step 2 — Spawn 3 agents IN PARALLEL (single message, multiple Agent tool calls)

Pass the log content directly in each agent's prompt so they do NOT need file access.

**Agent A — SITL Log Analyzer**
Include the full SITL log content in the prompt and ask it to report:
1. Count of `[ext_acc]` lines (exact number)
2. All lines containing `[ext_acc]` (first 20)
3. All WARN/ERROR lines after arm event
4. Mode transitions (POSCTL/ALTCTL confirmed?)
5. Arm/disarm events with timestamps

**Agent B — Test Log Analyzer**
Include the full test log content in the prompt and ask it to report:
1. Which TC passed / failed and why
2. Any Python exceptions or tracebacks
3. Telemetry values at moment of failure (tilt_deg, velocity)
4. Timing: when were acc commands sent?

**Agent C — Firmware Investigator**
Include findings from Agent A and B in the prompt (wait for them first if needed), then ask it to:
- Read firmware source files directly (it has access to the repo)
- If `[ext_acc]` = 0: trace publish path in `src/modules/mavlink/mavlink_receiver.cpp`
- If `[ext_acc]` > 0 but no movement: trace `FlightTaskManualAltitude::_applyExternalAcceleration()` → mc_pos_control chain
- Report suspected root cause with file:line

Note: Agent C reads repo files (not /tmp/), so it works fine without special permissions.

## Step 3 — After all 3 agents complete

Synthesize findings into:
1. Root cause (1-2 sentences)
2. Proposed fix (specific file:line changes)
3. Ask user to confirm before implementing
