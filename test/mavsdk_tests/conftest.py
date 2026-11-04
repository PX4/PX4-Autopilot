from typing import Iterator, Optional, Type, TypeVar

import pytest

# Show the values in failed asserts inside the testers, not just in tests.
pytest.register_assert_rewrite("autopilot_tester", "autopilot_tester_follow_me",
                               "autopilot_tester_loiter", "autopilot_tester_rtl",
                               "autopilot_tester_vision_target")

from autopilot_tester import AutopilotTester  # noqa: E402
from autopilot_tester_follow_me import AutopilotTesterFollowMe  # noqa: E402
from autopilot_tester_loiter import AutopilotTesterLoiter  # noqa: E402
from autopilot_tester_rtl import AutopilotTesterRtl  # noqa: E402
from autopilot_tester_vision_target import AutopilotTesterVisionTarget  # noqa: E402

T = TypeVar("T", bound=AutopilotTester)


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption("--url", default="udpin://0.0.0.0:14540",
                     help="MAVLink connection URL of the vehicle")
    parser.addoption("--speed-factor", type=float, default=None,
                     help="simulation speed factor, for information only")


def connected_tester(request: pytest.FixtureRequest, tester_class: Type[T]) -> Iterator[T]:
    speed_factor: Optional[float] = request.config.getoption("speed_factor")
    url: str = request.config.getoption("url")

    tester = tester_class(speed_factor)
    try:
        tester.connect(url)
        yield tester
    finally:
        failures = tester.close()

    if failures:
        pytest.fail("{} check{} failed in callbacks, first: {}".format(
            len(failures), "s" if len(failures) > 1 else "", failures[0]))


@pytest.fixture
def tester(request: pytest.FixtureRequest) -> Iterator[AutopilotTester]:
    """A tester connected to the vehicle."""
    yield from connected_tester(request, AutopilotTester)


@pytest.fixture
def tester_rtl(request: pytest.FixtureRequest) -> Iterator[AutopilotTesterRtl]:
    """A tester with rally point and geofence support, connected to the vehicle."""
    yield from connected_tester(request, AutopilotTesterRtl)


@pytest.fixture
def tester_loiter(request: pytest.FixtureRequest) -> Iterator[AutopilotTesterLoiter]:
    """A tester for fixed-wing loiter, reposition and figure eight, connected to the vehicle."""
    yield from connected_tester(request, AutopilotTesterLoiter)


@pytest.fixture
def tester_follow_me(request: pytest.FixtureRequest) -> Iterator[AutopilotTesterFollowMe]:
    """A tester with follow-me support, connected to the vehicle."""
    yield from connected_tester(request, AutopilotTesterFollowMe)


@pytest.fixture
def tester_vision_target(request: pytest.FixtureRequest) -> Iterator[AutopilotTesterVisionTarget]:
    """A tester simulating landing target sensors, connected to the vehicle."""
    yield from connected_tester(request, AutopilotTesterVisionTarget)
