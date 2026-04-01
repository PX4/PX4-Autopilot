#!/bin/bash
# run_unit_test.sh
# Build và chạy PX4 unit tests, capture output để analyze.
#
# Usage:
#   bash Tools/run_unit_test.sh [TEST_FILTER]
#
# Ví dụ:
#   bash Tools/run_unit_test.sh AccSpExternal   # chỉ test AccSpExternal
#   bash Tools/run_unit_test.sh                 # tất cả tests
#
# Output:
#   /tmp/unit_test_<RUN_ID>.log  — full test output (gtest format)

set -euo pipefail

FILTER="${1:-AccSpExternal}"
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
TEST_LOG="/tmp/unit_test_${RUN_ID}.log"

# ─── Cleanup ──────────────────────────────────────────────────────────────────
cleanup() {
    echo ""
    echo "[run_unit_test] Done. Log:"
    echo "   Test : ${TEST_LOG}"
}
trap cleanup EXIT

# ─── Header ───────────────────────────────────────────────────────────────────
echo "════════════════════════════════════════"
echo " PX4 Unit Test Runner"
echo " RUN_ID : ${RUN_ID}"
echo " Filter : ${FILTER}"
echo "════════════════════════════════════════"

# ─── Run tests ────────────────────────────────────────────────────────────────
echo "[run_unit_test] Building và chạy tests..."
cd "${REPO_ROOT}"

set +e
# Bước 1: Build (make tests build binary nhưng output CTest-level, không có gtest detail)
make tests TESTFILTER="${FILTER}" > /tmp/unit_build_${RUN_ID}.log 2>&1
BUILD_EXIT=$?

if [ "${BUILD_EXIT}" -ne "0" ]; then
    cat /tmp/unit_build_${RUN_ID}.log | tee "${TEST_LOG}"
    TEST_EXIT="${BUILD_EXIT}"
else
    # Bước 2: Chạy binary trực tiếp để lấy gtest output đầy đủ
    TEST_BIN=$(find build/px4_sitl_test -name "${FILTER}*" -o -name "*${FILTER}" 2>/dev/null \
               | grep -v "\.dir\|\.a\|\.o\|CMake" | head -1)
    if [ -z "${TEST_BIN}" ]; then
        TEST_BIN=$(find build/px4_sitl_test -name "unit-${FILTER}" -o -name "functional-${FILTER}" 2>/dev/null | head -1)
    fi

    if [ -n "${TEST_BIN}" ]; then
        echo "[run_unit_test] Running: ${TEST_BIN}"
        "${TEST_BIN}" 2>&1 | tee "${TEST_LOG}"
        TEST_EXIT="${PIPESTATUS[0]}"
    else
        echo "[run_unit_test] WARN: binary not found, showing build log"
        cat /tmp/unit_build_${RUN_ID}.log | tee "${TEST_LOG}"
        TEST_EXIT="${BUILD_EXIT}"
    fi
fi
set -e

# ─── Summary ──────────────────────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════"
echo " SUMMARY  [RUN_ID: ${RUN_ID}]"
echo "════════════════════════════════════════"

# Strip ANSI color codes trước khi grep (gtest dùng \e[0;32m etc.)
CLEAN_LOG=$(sed 's/\x1b\[[0-9;]*m//g' "${TEST_LOG}" 2>/dev/null)

# Đếm individual gtest cases từ summary line: "[  PASSED  ] 27 tests."
PASSED=$(echo "${CLEAN_LOG}" | grep -oP '(?<=\[  PASSED  \] )\d+' | tail -1 || echo "0")
FAILED=$(echo "${CLEAN_LOG}" | grep -oP '(?<=\[  FAILED  \] )\d+ test' | grep -oP '^\d+' | tail -1 || echo "0")
PASSED=${PASSED:-0}
FAILED=${FAILED:-0}

echo " Passed : ${PASSED}"
echo " Failed : ${FAILED}"
echo " Exit   : ${TEST_EXIT}"
echo ""

if [ "${FAILED}" -gt "0" ]; then
    echo "── Failed tests ──"
    echo "${CLEAN_LOG}" | grep "\[  FAILED  \]" || true
fi

echo "════════════════════════════════════════"
echo " RUN_ID: ${RUN_ID}"
echo "════════════════════════════════════════"

exit "${TEST_EXIT}"
