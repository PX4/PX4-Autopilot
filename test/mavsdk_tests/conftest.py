from typing import Iterator, Optional

import pytest

# Show the values in failed asserts inside the tester, not just in tests.
pytest.register_assert_rewrite("autopilot_tester")

from autopilot_tester import AutopilotTester  # noqa: E402


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption("--url", default="udpin://0.0.0.0:14540",
                     help="MAVLink connection URL of the vehicle")
    parser.addoption("--speed-factor", type=float, default=None,
                     help="simulation speed factor, for information only")


@pytest.fixture
def tester(request: pytest.FixtureRequest) -> Iterator[AutopilotTester]:
    """A tester connected to the vehicle."""
    speed_factor: Optional[float] = request.config.getoption("speed_factor")
    url: str = request.config.getoption("url")

    tester = AutopilotTester(speed_factor)
    try:
        tester.connect(url)
        yield tester
    finally:
        failures = tester.close()

    if failures:
        pytest.fail("{} check{} failed in callbacks, first: {}".format(
            len(failures), "s" if len(failures) > 1 else "", failures[0]))
