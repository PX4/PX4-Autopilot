---
name: review-pr
description: Review a pull request with structured, domain-aware feedback
argument-hint: "<PR number or URL>"
allowed-tools: Bash, Read, Glob, Grep, Agent
---

# PX4 Pull Request Review

Review a pull request with domain-aware checks based on which files are changed.

**No Claude attribution anywhere.**

## Steps

1. **Fetch PR context.** Run these in parallel:
   - `gh pr view <PR> --json number,title,body,baseRefName,headRefName,files,commits,reviewRequests,reviews,author`
   - `gh pr checks <PR>` (exit code 8 means some checks are pending, this is normal, not an error)
   - `gh pr diff <PR>` -- if this fails with HTTP 406 (300+ files), do NOT retry. Instead use `gh api repos/OWNER/REPO/pulls/NUMBER/files --paginate` to get the full file list in one call, then fetch patches for key infrastructure files individually and sample representative changes from each domain touched.
   - `gh api repos/OWNER/REPO/pulls/NUMBER/comments --paginate --jq '.[] | {user: .user.login, body: .body, path: .path, created_at: .created_at}'` to get inline review comments
   - `gh api repos/OWNER/REPO/issues/NUMBER/comments --paginate --jq '.[] | {user: .user.login, body: .body, created_at: .created_at}'` to get PR conversation comments

   From the PR metadata, note:
   - **Assigned reviewers**: who has been requested to review (from `reviewRequests`)
   - **Existing reviews**: who has already reviewed and their verdict (from `reviews` -- approved, changes_requested, commented, dismissed)
   - **PR comments and inline comments**: read all existing feedback to avoid duplicating points already raised by other reviewers, and to build on their discussion rather than ignoring it

2. **Check CI status.** From the `gh pr checks` output in step 1, summarize pass/fail/pending. If there are failures, fetch logs with `gh run view <run-id> --log-failed`. Include CI status in the output.

3. **Recommend merge strategy.** Analyze the commit history and recommend squash or rebase merge. This decision informs all subsequent commit hygiene feedback.

   **Recommend rebase merge** when:
   - Commits are atomic, each builds/works independently
   - Each commit has a proper `type(scope): description` message
   - The PR intentionally separates logical changes (e.g., refactor + feature, or one commit per module)
   - The commit history tells a useful story that would be lost by squashing

   **Recommend squash merge** when:
   - There are WIP, fixup, or review-response commits
   - Commit messages are messy or inconsistent
   - The PR is a single logical change spread across multiple commits
   - There are "oops" or "make format" commits mixed in

   Include the recommendation in the output. If recommending rebase, flag any commits that break atomicity or have bad messages. If recommending squash, don't bother flagging individual commit messages (they'll be discarded) but ensure the PR title is correct since it becomes the squash commit message.

4. **Check conventional commit title.** Verify the PR title follows `type(scope): description` per CONTRIBUTING.md. The PR title becomes the commit message on squash-merge, so it must be accurate and descriptive. Verify the scope matches the primary area of changed files. If the PR introduces breaking changes, the title must include `!` before the colon. If rebase merge was recommended in step 3, also scan individual commit messages for anti-patterns: vague messages ("fix", "update"), missing type prefix, review-response noise ("apply suggestions from code review", "do make format"), or WIP markers. Flag these for rewording.

5. **Identify domains touched.** Classify changed files into domains based on paths (a PR may touch multiple):
   - **Estimation**: `src/modules/ekf2/`, `src/lib/wind_estimator/`, `src/lib/world_magnetic_model/`
   - **Control**: `src/modules/mc_*control*/`, `src/modules/fw_*control*/`, `src/modules/flight_mode_manager/`, `src/lib/rate_control/`, `src/lib/npfg/`, `src/modules/vtol_att_control/`
   - **Drivers/CAN**: `src/drivers/`, `src/modules/cyphal/`, `src/drivers/uavcan*/`
   - **Simulation**: `src/modules/simulation/`, `Tools/simulation/`
   - **System**: `src/modules/commander/`, `src/modules/logger/`, `src/systemcmds/`, `platforms/`, `src/modules/dataman/`
   - **Board Addition**: `boards/{manufacturer}/{board}/` (new directories only, not modifications to existing boards)
   - **CI/Build**: `.github/`, `CMakeLists.txt`, `Makefile`, `cmake/`, `Tools/`, `Kconfig`
   - **Messages/Protocol**: `msg/`, `src/modules/mavlink/`, `src/modules/uxrce_dds_client/`

6. **Apply core checks** (always):
   - **Correctness**: logic errors, off-by-ones, unhandled edge cases
   - **Type safety**: int16 overflow, float/double promotion, unsigned subtraction, use `uint64_t` for absolute time
   - **Initialization**: uninitialized variables, missing default construction
   - **Buffer safety**: unchecked array access, stack allocation of large buffers, snprintf bounds
   - **Magic numbers**: every numeric literal needs a named constant or justification
   - **Framework reuse**: use PX4_ERR/WARN/INFO, existing libraries (AlphaFilter, SlewRate, RateControl), MAVLink constants from the library
   - **Naming**: accurate, no unjustified abbreviations, current terminology (GPS -> GNSS for new code)
   - **Unnecessary complexity**: can code be removed instead of added? Is there a simpler pattern?
   - **Test coverage**: new features should include unit or integration tests; bug fixes should include regression tests where practical. When automated testing is infeasible (hardware-specific), require a flight log link from https://logs.px4.io or bench test evidence.
   - **PR hygiene**: focused scope, no unrelated formatting, no stale submodule changes. Commits should be atomic and independently revertable. Multiple WIP or review-response commits should be squashed. Clean, logical commits will be preserved individually on main via rebase merge. **Do NOT assume PRs are squash-merged. Both squash and rebase merge are enabled; merge commits are disabled.** Verify the PR targets `main` unless it is a backport or release-specific fix.
   - **Formatting**: `make format` / `make check_format` (astyle) for C/C++ files; `clang-tidy` clean. Python files checked with `mypy` and `flake8`. PRs failing CI format or lint checks will not be merged.
   - **Coding style**: C/C++ must follow the [PX4 coding style](https://docs.px4.io/main/en/contribute/code.html)
   - **Necessity**: challenge every addition with "Why?" Is this actually needed or just copied? Can we change a default instead of adding runtime detection?
   - **Root cause vs symptom**: is this fixing the real problem or masking it?
   - **Ecosystem impact**: what does this change mean for QGC users, log analysis tools, and third-party integrations?
   - **Sustainability**: who will maintain this? Does it create long-term burden?
   - **Architecture fit**: does the code live in the module that naturally owns the data? Are there unnecessary cross-module dependencies?
   - **End user impact**: will parameters confuse less-technical users? Are error messages actionable in QGC?

7. **Apply domain checks** based on step 5:

   **Estimation:**
   - Singularities in aerospace math (euler angles near gimbal lock, sideslip at low airspeed)
   - Aliasing from downsampling sensor data without filtering
   - Kalman filter correctness (Joseph form, innovation variance, covariance symmetry)
   - CPU cost on embedded targets (avoid unnecessary sqrt, limit fusion rate)
   - Frame/coordinate system correctness (FRD vs NED, body vs earth)

   **Control:**
   - Phase margin: output filters consume margin for no benefit; prefer adjusting gyro/d-gyro cutoffs
   - Circular dependencies: sensor data feeding back into its own control loop (e.g., throttle-based airspeed in TECS)
   - NaN propagation in flight-critical math; check `PX4_ISFINITE` before magnitude checks
   - Setpoint generation vs output-stage hacks: prefer proper setpoint smoothing over controller output filtering
   - Yaw control edge cases: heading lock, drift, setpoint propagation
   - Flight task inheritance chain: correct base class for the desired behavior
   - Control allocation: actuator function ordering, motor index mapping

   **Drivers/CAN:**
   - CAN bus devices behave differently from serial/SPI; check driver assumptions
   - ESC index mapping: telemetry index != channel when motors are disabled
   - ESC hardware quirks: 4-in-1 ESCs may report current on only one channel
   - device_id correctness and I2CSPIDriver patterns
   - Time representation: prefer `hrt_abstime` over iteration counts

   **Simulation:**
   - Physics fidelity: noise models should match reality (GPS noise is not Gaussian)
   - Keep gz_bridge generic; vehicle-specific logic belongs in plugins
   - Prefer gz-transport over ROS2 dependencies when possible
   - Wrench commands for physics correctness vs kinematic constraints
   - Library generic/specific boundary: only base classes in common libs

   **System:**
   - Race conditions and concurrency: no partial fixes, demand complete solutions
   - Semaphore/scheduling edge cases; understand RTOS guarantees
   - State machine sequential-logic bugs (consecutive RTL, armed/disarmed alternation)
   - uORB-driven scheduling (`SubscriptionCallback`), not extra threads
   - param_set triggers auto-save; no redundant param_save_default
   - Flash/memory efficiency: avoid `std::string` on embedded, minimize SubscriptionData usage
   - Constructor initialization order matters

   **CI/Build:**
   - Pipeline race conditions (tag + branch push double-trigger, git describe correctness)
   - Container image size (check layer bloat)
   - Ubuntu LTS support policy (latest + one prior only)
   - Build time impact
   - CMake preferred over Makefiles

   **Messages/Protocol:**
   - Backwards compatibility: will this break QGC, post-flight tools, or uLog parsers?
   - uORB: `timestamp` for publication metadata, `timestamp_sample` close to physical sample, include `device_id`
   - Don't version messages unless strictly needed
   - Parameter UX: will this confuse users in a GCS? Every new param is a configuration burden
   - MAVLink: use library constants, don't implement custom stream rates

   **Board Addition:**
   - **Flight logs**: require a link to https://logs.px4.io demonstrating basic operation for the vehicle type (hover for multicopters, stable flight for fixed-wing, driving for rovers, etc.); short bench-only logs are insufficient
   - **Documentation**: require a docs page in `docs/en/flight_controller/` with pinout, where-to-buy, connector types, version badge, and manufacturer-supported notice block
   - **USB VID/PID**: must not reuse another manufacturer's Vendor ID; manufacturer must use their own
   - **Board naming**: directory is `boards/{manufacturer}/{board}/`, both lowercase, hyphens for board name
   - **Unique board_id**: registered in `boards/boards.json`, no collisions
   - **Copied code cleanup**: check for leftover files, configs, or comments from the template board; "Is this real or leftover?"
   - **RC configuration**: prefer `CONFIG_DRIVERS_COMMON_RC` over legacy `CONFIG_DRIVERS_RC_INPUT`
   - **No board-specific custom modules**: reject copy-pasted drivers (e.g., custom heater) when existing infrastructure works
   - **Bootloader**: expect a bootloader defconfig (`nuttx-config/bootloader/defconfig`) or explanation of shared bootloader
   - **CI integration**: board must be added to CI compile workflows so it builds on every PR
   - **Flash constraints**: verify enabled modules fit in flash; we are running low across all board targets
   - **Port labels**: serial port labels must match what is physically printed on the board
   - **Hardware availability**: for unknown manufacturers, verify the product exists and is purchasable (no vaporware)

8. **Format output** as:
   - **CI status**: pass/fail summary, link to failed runs if any
   - **Merge strategy**: recommend squash or rebase merge with reasoning
   - **Title check**: pass/fail with suggestion
   - **Review status**: list assigned reviewers and any existing reviews (who approved, who requested changes, key points already raised). Note if your review would duplicate feedback already given.
   - **Domains detected**: list which domain checks were applied
   - **Summary**: one paragraph on what the PR does and whether the approach is sound
   - **Issues**: numbered list, each with file:line, severity (blocker/warning/nit), and explanation. Skip issues already raised by other reviewers unless you have something to add.
   - **Verdict**: approve, request changes, or needs discussion

   After the structured output, also display a **draft PR comment** formatted using the PR comment formatting rules from step 9. This gives the user a preview of what would be posted.

9. **Interactive dialog.** After displaying the review, present the user with these options:

   Present options based on the verdict:

   If verdict is **approve**:
   ```
   What would you like to do?
   1. Chat about this PR (ask questions, explore code) [default]
   2. Approve this PR and post the review comment
   3. Adjust the review or draft (tell me what to change)
   4. Done for now
   ```

   If verdict is **request changes**:
   ```
   What would you like to do?
   1. Chat about this PR (ask questions, explore code) [default]
   2. Request changes on this PR and post the review comment
   3. Adjust the review or draft (tell me what to change)
   4. Done for now
   ```

   If verdict is **needs discussion**:
   ```
   What would you like to do?
   1. Chat about this PR (ask questions, explore code) [default]
   2. Post the review as a comment (no approval or rejection)
   3. Adjust the review or draft (tell me what to change)
   4. Done for now
   ```

   Wait for the user to choose before proceeding. If they pick:
   - **1 (chat)**: enter a free-form conversation about the PR. The user can ask about specific files, code paths, or decisions. When done, loop back to the options. This is the default if the user just presses enter.
   - **2 (submit)**: use the draft PR comment already shown. Before posting, check if you have review permissions: run `gh api repos/OWNER/REPO/collaborators/$(gh api user --jq .login)/permission --jq .permission` -- if `admin` or `write`, submit as a formal review with `gh pr review <PR> --approve --body "..."` or `gh pr review <PR> --request-changes --body "..."` based on the verdict. If no write access, fall back to `gh pr comment <PR> --body "..."`. Always confirm with the user before posting.
   - **3 (adjust)**: ask what to change, update the review and draft, then loop back to the options.
   - **4 (done)**: stop.

   **PR comment formatting rules** (for the draft):
   When writing the GitHub comment, rewrite the review to sound like a human reviewer, not a structured report. Do NOT include the full skill output. Instead:
   - Drop most meta-sections (CI status, title check, domains detected, severity labels) but keep the merge strategy recommendation (e.g., "I'd suggest a rebase merge here since the commits are clean and atomic" or "This should be squash-merged, the commit history is messy")
   - Write conversationally: "Nice work on this. A few things I noticed:" not "Issues: 1. file:line (warning):"
   - Lead with a brief take on the overall change (1-2 sentences)
   - List only actionable feedback as natural review comments, not numbered checklists
   - Skip nits unless they are particularly useful
   - End with a clear stance: looks good to merge, needs a few changes, or let's discuss X
   - Post with `gh pr comment <PR> --body "$(cat <<'EOF' ... EOF)"`. Do not post without explicit confirmation.

If the user provided arguments, use them as context: $ARGUMENTS
