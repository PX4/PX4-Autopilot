---
name: analyze-unit-test
description: Analyze PX4 unit test results after running run_unit_test.sh. Reads log, reports pass/fail, spawns agents to trace failures to source code.
---

Analyze PX4 unit test log and find root cause of failures.

The user has just run `bash Tools/run_unit_test.sh` and provided a RUN_ID.

Log file: `/tmp/unit_test_$ARGUMENTS.log`

If no RUN_ID provided, list available logs:
```bash
ls /tmp/unit_test_*.log 2>/dev/null | sort | tail -5
```

## Step 1 — Lead Agent reads log file FIRST

Read `/tmp/unit_test_$ARGUMENTS.log` directly using the Read tool.

Extract:
- Total passed / failed count
- List of failed test names (lines with `[ FAILED ]`)
- Error messages and assertion failures for each failed test

## Step 2 — Decision logic

**If 0 failures:**
Report "All tests passed" with counts. No agents needed. Done.

**If failures exist:**
Spawn 2 agents IN PARALLEL (single message):

**Agent A — Test Output Analyzer**
Pass the full log content in the prompt. Ask it to:
1. List every failed test (suite name + test name)
2. For each failure: extract the exact assertion that failed (Expected/Actual values)
3. Identify the pattern — is it logic error, threshold mismatch, wrong sign, etc.?
4. Return structured report: test_name → failure_reason

**Agent B — Source Code Investigator**
Pass Agent A findings in the prompt (after Agent A completes). Ask it to:
- Read the relevant source files in the repo:
  - `src/modules/flight_mode_manager/tasks/ManualAltitude/FlightTaskManualAltitude.cpp`
  - `src/modules/flight_mode_manager/tasks/ManualAltitude/FlightTaskManualAltitude.hpp`
  - `src/modules/flight_mode_manager/tasks/ManualAltitude/AccSpExternalTest.cpp`
- For each failed test: find the corresponding test case in AccSpExternalTest.cpp
- Cross-reference with the helper logic in the same file
- Identify: is the bug in the test helper (mirrors wrong logic) or in the firmware source?
- Report: file:line → what needs to change

## Step 3 — Synthesize

After both agents complete:
1. Root cause (1-2 sentences per failed test)
2. Proposed fix (specific file:line changes)
3. Ask user to confirm before implementing
