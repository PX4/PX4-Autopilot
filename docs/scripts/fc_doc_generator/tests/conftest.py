"""
Shared fixtures and snapshot helper for fc_doc_generator tests.
"""
import difflib
import sys
from pathlib import Path

import pytest

# Make fc_doc_generator importable from any working directory
SCRIPT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SCRIPT_DIR))

FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"
SNAPSHOTS_DIR = Path(__file__).resolve().parent / "snapshots"


# ---------------------------------------------------------------------------
# Custom CLI option
# ---------------------------------------------------------------------------

def pytest_addoption(parser):
    parser.addoption(
        "--update-snapshots",
        action="store_true",
        default=False,
        help="Regenerate all snapshot files from current script output.",
    )


# ---------------------------------------------------------------------------
# Snapshot fixture
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def snapshot(request):
    """Assert that `actual` matches the stored snapshot file, or write it.

    Usage::

        def test_something(snapshot):
            result = generate_something(...)
            snapshot("my_snapshot.md", result)

    Run ``pytest --update-snapshots`` to regenerate all snapshot files after
    intentional changes to the generator.
    """
    update = request.config.getoption("--update-snapshots")

    def _assert(name: str, actual: str):
        snap = SNAPSHOTS_DIR / name
        if update:
            SNAPSHOTS_DIR.mkdir(parents=True, exist_ok=True)
            snap.write_text(actual, encoding="utf-8")
            return
        if not snap.exists():
            pytest.fail(
                f"Snapshot missing: {snap}\n"
                f"Run with --update-snapshots to create it.\n\n"
                f"Actual output:\n{actual}"
            )
        expected = snap.read_text(encoding="utf-8")
        if actual != expected:
            diff = "\n".join(difflib.unified_diff(
                expected.splitlines(),
                actual.splitlines(),
                fromfile=f"snapshots/{name}",
                tofile="actual",
                lineterm="",
            ))
            pytest.fail(
                f"Snapshot mismatch for {name}:\n{diff}\n\n"
                f"Run with --update-snapshots to regenerate."
            )

    return _assert


# ---------------------------------------------------------------------------
# Board path fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def board_stm32h7_all_dshot():
    """STM32H7, 8 outputs, 4 timers all with DMA (all DShot/BDShot), no IO board."""
    return FIXTURES_DIR / "stm32h7_all_dshot"


@pytest.fixture(scope="session")
def board_stm32h7_mixed_io():
    """STM32H7, 9 FMU outputs, IO board, mixed DShot timers, PPM pin, COMMON_RC."""
    return FIXTURES_DIR / "stm32h7_mixed_io"


@pytest.fixture(scope="session")
def board_imxrt():
    """iMXRT, 8 outputs, all BDShot, safety switch + LED, PPS capture."""
    return FIXTURES_DIR / "imxrt_all_dshot"


@pytest.fixture(scope="session")
def board_stm32f4():
    """STM32F4, 6 outputs, 3 timers with no DMA (no DShot)."""
    return FIXTURES_DIR / "stm32f4_no_dshot"


@pytest.fixture(scope="session")
def board_ppm_shared():
    """STM32H7, 4 outputs, PPM pin shared with RC serial UART."""
    return FIXTURES_DIR / "stm32h7_ppm_shared"


@pytest.fixture(scope="session")
def board_stm32h7_variant():
    """STM32H7 board with variant sensor blocks (if ver hwtypecmp)."""
    return FIXTURES_DIR / "stm32h7_variant"


@pytest.fixture(scope="session")
def board_stm32h7_graceful_fail():
    """STM32H7, graceful-fail pattern: BMI088 starts unconditionally AND in VD000000 block."""
    return FIXTURES_DIR / "stm32h7_graceful_fail"


@pytest.fixture(scope="session")
def board_stm32h7_capture_channels():
    """STM32H7, 16 outputs: 8 regular (DShot) + 8 initIOTimerChannelCapture (no DShot)."""
    return FIXTURES_DIR / "stm32h7_capture_channels"
