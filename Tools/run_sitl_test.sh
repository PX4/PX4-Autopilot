#!/bin/bash
# run_sitl_test.sh
# Orchestrate: SITL → Python test → capture logs → cleanup
#
# Usage:
#   bash Tools/run_sitl_test.sh [position|altitude]
#
# Outputs (mỗi lần chạy tạo RUN_ID riêng):
#   /tmp/sitl_<RUN_ID>.log   — full SITL terminal output
#   /tmp/test_<RUN_ID>.log   — Python test output
#
# Dừng: Ctrl+C — script tự cleanup SITL + Gazebo

set -euo pipefail

MODE="${1:-position}"
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
SITL_LOG="/tmp/sitl_${RUN_ID}.log"
TEST_LOG="/tmp/test_${RUN_ID}.log"
SITL_READY_PATTERN="pxh>"
SITL_TIMEOUT=120  # seconds to wait for SITL ready

# ─── Cleanup ──────────────────────────────────────────────────────────────────
cleanup() {
    echo ""
    echo "[run_sitl_test] Stopping... (Ctrl+C hoặc test kết thúc)"
    pkill -f "px4_sitl_default" 2>/dev/null || true
    pkill -f "gzserver"         2>/dev/null || true
    pkill -f "gzclient"         2>/dev/null || true
    pkill -f "px4"              2>/dev/null || true
    sleep 1
    echo "[run_sitl_test] Done. Logs của lần chạy này:"
    echo "   SITL : ${SITL_LOG}"
    echo "   Test : ${TEST_LOG}"
}
trap cleanup EXIT

# ─── Header ───────────────────────────────────────────────────────────────────
echo "════════════════════════════════════════"
echo " PX4 SITL Acc Test"
echo " RUN_ID : ${RUN_ID}"
echo " Mode   : ${MODE}"
echo " Dừng   : Ctrl+C"
echo "════════════════════════════════════════"

# ─── Step 1: Start SITL ───────────────────────────────────────────────────────
echo "[run_sitl_test] Starting PX4 SITL → ${SITL_LOG}"
> "${SITL_LOG}"
cd "${REPO_ROOT}"
stdbuf -oL -eL make px4_sitl_default gazebo-classic \
    >> "${SITL_LOG}" 2>&1 &
SITL_PID=$!

# ─── Step 2: Wait for SITL ready ─────────────────────────────────────────────
echo "[run_sitl_test] Chờ SITL ready (timeout=${SITL_TIMEOUT}s)..."
ELAPSED=0
while ! grep -q "${SITL_READY_PATTERN}" "${SITL_LOG}" 2>/dev/null; do
    sleep 2
    ELAPSED=$((ELAPSED + 2))
    printf "\r[run_sitl_test] ${ELAPSED}s..."
    if [ "${ELAPSED}" -ge "${SITL_TIMEOUT}" ]; then
        echo ""
        echo "[run_sitl_test] ERROR: SITL không ready sau ${SITL_TIMEOUT}s"
        tail -20 "${SITL_LOG}"
        exit 1
    fi
    if ! kill -0 "${SITL_PID}" 2>/dev/null; then
        echo ""
        echo "[run_sitl_test] ERROR: SITL process died unexpectedly"
        tail -20 "${SITL_LOG}"
        exit 1
    fi
done
echo ""
echo "[run_sitl_test] SITL ready sau ${ELAPSED}s"

# ─── Step 3: Run Python test ──────────────────────────────────────────────────
echo "[run_sitl_test] Chạy sitl_acc_test.py → ${TEST_LOG}"
echo "[run_sitl_test] (test tự kết thúc khi xong, hoặc Ctrl+C để dừng sớm)"
echo ""
python3 "${REPO_ROOT}/Tools/sitl_acc_test.py" \
    --mode "${MODE}" \
    > "${TEST_LOG}" 2>&1
TEST_EXIT="${PIPESTATUS[0]}"

# ─── Step 4: Summary ──────────────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════"
echo " SUMMARY  [RUN_ID: ${RUN_ID}]"
echo "════════════════════════════════════════"
echo " Exit code: ${TEST_EXIT}"
echo ""
echo "── [ext_acc] hits ──"
COUNT=$(grep -c "\[ext_acc\]" "${SITL_LOG}" 2>/dev/null || echo "0")
echo " ${COUNT} lines trong SITL log"
echo ""
echo "── Test results ──"
grep -E "TC-|PASS|FAIL" "${TEST_LOG}" 2>/dev/null || echo " (không có kết quả)"
echo "════════════════════════════════════════"

exit "${TEST_EXIT}"
