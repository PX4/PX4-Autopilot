"""Run the real planner through the out-of-tree mission_route_planner_cli binary.

The binary reads one scenario on stdin and prints the plan and the planner's debug
traces, one entry per line. See planner_cli/src for its source and the build command.
"""

from dataclasses import dataclass, field, fields
from pathlib import Path
import shlex
import subprocess
from typing import Dict, List, Optional, Sequence, Tuple

from .geometry import distance_m
from .models import Position, Scene, Waypoint

BINARY_NAME = "mission_route_planner_cli"
BUILD_DIR = Path("build/px4_sitl_test")
CLI_LOCATION = Path("Tools/navigator_mission_planner_visualizer/planner_cli")
# Any of these newer than the binary means the binary no longer runs the current code.
SOURCE_GLOBS = (
    "src/modules/navigator/mission_route_*.cpp",
    "src/modules/navigator/mission_route_*.h",
    "src/modules/navigator/mission_item_utils.*",
    "src/modules/navigator/navigation.h",
    str(CLI_LOCATION / "src" / "*"),
)
ACTIONS = ("RTL", "RESUME")
LAND_COMMANDS = ("NAV_CMD_LAND", "NAV_CMD_VTOL_LAND")
VTOL_STATES = ("UNDEFINED", "MC", "FW")
_STRUCTURED_TAGS = ("ACTION", "STATUS", "PLAN", "END")


@dataclass
class PlanRequest:
    """The request fields, defaulting to the PX4 parameter defaults where one exists."""

    action: str = "RTL"
    mission_index: int = 1
    mission_land_index: int = -1
    current_route_direction_reversed: bool = False
    active_jump_anchor: int = -1
    home_altitude_amsl: float = 500.0
    projection_search_distance_m: float = 30.0  # MIS_MC_SEG_DIST (MIS_FW_SEG_DIST is 150 m)
    safe_point_projection_search_distance_m: float = 30.0  # RTL_RP_SEG_DIST
    acceptance_radius_m: float = 10.0  # NAV_ACC_RAD
    direct_goal_acceptance_radius_m: float = 10.0
    altitude_acceptance_radius_m: float = 10.0
    fw_u_turn_penalty_m: float = 4000.0  # RTL_FW_UTURN_PEN
    is_fixed_wing: bool = False
    is_vtol: bool = False
    in_transition_to_fw: bool = False
    vtol_state_on_mission_upload: str = "UNDEFINED"
    require_vtol_approach: bool = False

    def settings(self) -> Dict[str, str]:
        """Field values as the CLI reads them, in declaration order."""
        result = {}
        for item in fields(self):
            if item.name == "action":
                continue
            value = getattr(self, item.name)
            result[item.name] = _cli_value(value)
        return result


@dataclass
class PlanResult:
    action: str = ""
    status: str = ""
    plan: Dict[str, str] = field(default_factory=dict)
    trace: List[str] = field(default_factory=list)
    raw: str = ""

    @property
    def ok(self) -> bool:
        return self.status == "OK"

    @property
    def failure(self) -> str:
        return self.status[5:] if self.status.startswith("FAIL ") else ""

    def position(self, prefix: str) -> Optional[Tuple[float, float, float]]:
        try:
            values = tuple(float(self.plan[f"{prefix}_{axis}"]) for axis in ("lat", "lon", "alt"))
        except (KeyError, ValueError):
            return None
        return values if all(value == value for value in values) else None

    def int_field(self, name: str, default: int = -1) -> int:
        try:
            return int(self.plan[name])
        except (KeyError, ValueError):
            return default

    def bool_field(self, name: str) -> bool:
        return self.plan.get(name) == "1"

    def rows(self) -> List[Dict[str, str]]:
        """The plan for a narrow table: one row per position, the always-true valid flag dropped."""
        rows = []
        for key, value in self.plan.items():
            if key == "valid" or key.endswith(("_lon", "_alt")):
                continue
            if key.endswith("_lat"):
                key = key[:-4]
                position = self.position(key)
                value = "none" if position is None else _position_text(position)
            rows.append({"field": key, "value": value})
        return rows


def _position_text(position: Tuple[float, float, float]) -> str:
    return f"{position[0]:.7f}, {position[1]:.7f}, {position[2]:.1f} m"


# ---- Scenario encoding ----


def first_mission(scene: Scene) -> Tuple[str, List[Waypoint]]:
    for name, items in scene.missions.items():
        if items:
            return name, items
    return "mission", []


def vehicle_positions(scene: Scene) -> List[Position]:
    """Every position named like a vehicle, else the first position, else nothing."""
    vehicles = [position for position in scene.positions if position.is_vehicle]
    return vehicles or scene.positions[:1]


def vehicle_position(scene: Scene) -> Optional[Position]:
    vehicles = vehicle_positions(scene)
    return vehicles[0] if vehicles else None


def land_index(items: Sequence[Waypoint]) -> int:
    for index, item in enumerate(items):
        if item.cmd in LAND_COMMANDS:
            return index
    return -1


def encode(scene: Scene, request: PlanRequest, vehicle: Optional[Position] = None) -> str:
    """The scenario text the CLI reads, for one vehicle position. Only position items exist in a Scene."""
    _, items = first_mission(scene)
    vehicle = vehicle or vehicle_position(scene)
    if not items:
        raise ValueError("The scenario has no mission items.")
    if vehicle is None:
        raise ValueError("The scenario has no vehicle position.")
    lines = [f"ITEM {_cmd_name(item.cmd)} {item.lat:.9f} {item.lon:.9f} {item.alt:.3f}" for item in items]
    lines += [f"RALLY {point.lat:.9f} {point.lon:.9f} {point.alt:.3f}" for point in scene.safe_points]
    lines.append(f"VEHICLE {vehicle.lat:.9f} {vehicle.lon:.9f} {vehicle.alt:.3f}")
    lines += [f"SET {key} {value}" for key, value in request.settings().items()]
    if scene.velocity is not None:
        lines.append(f"SET velocity_north_m_s {scene.velocity[0]:.3f}")
        lines.append(f"SET velocity_east_m_s {scene.velocity[1]:.3f}")
    lines.append(f"ACTION {request.action}")
    return "\n".join(lines) + "\n"


def _cmd_name(cmd: str) -> str:
    return cmd[8:] if cmd.startswith("NAV_CMD_") else (cmd or "WAYPOINT")


def _cli_value(value) -> str:
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


# ---- Output parsing ----


def parse_output(text: str) -> PlanResult:
    result = PlanResult(raw=text)
    for line in text.splitlines():
        tokens = line.split()
        if not tokens or tokens[0] not in _STRUCTURED_TAGS:
            if line.strip():
                result.trace.append(line)
            continue
        tag, args = tokens[0], tokens[1:]
        if tag == "ACTION" and args:
            result.action = args[0]
        elif tag == "STATUS":
            result.status = " ".join(args)
        elif tag == "PLAN" and len(args) >= 2:
            result.plan[args[0]] = " ".join(args[1:])
    return result


# ---- The path the plan describes, for the map ----


def plan_path(items: Sequence[Waypoint], vehicle: Position, result: PlanResult) -> List[Tuple[float, float]]:
    """Vehicle, join point, route items in the planned direction up to the first landing, branch-off, goal."""
    if not result.ok:
        return []
    start = (vehicle.lat, vehicle.lon)
    goal = result.position("goal")
    if result.bool_field("fly_direct_to_goal") and goal is not None:
        return [start, goal[:2]]
    join = result.position("join")
    if join is None:
        return []
    path = [start, join[:2]]
    index = result.int_field("first_mission_item_index")
    step = -1 if result.bool_field("direction_reversed") else 1
    branch_off_index = result.int_field("branch_off_mission_item_index")
    branch_off = result.position("branch_off")
    while 0 <= index < len(items):
        if index == branch_off_index and branch_off is not None:
            path.append(branch_off[:2])
            break
        path.append((items[index].lat, items[index].lon))
        if items[index].cmd in LAND_COMMANDS:
            break
        index += step
    if goal is not None and result.action == "RTL" and distance_m(*path[-1], *goal[:2]) > 0.01:
        path.append(goal[:2])
    return path


# ---- Locating, checking and running the binary ----


def repo_root(start: Optional[Path] = None) -> Optional[Path]:
    for ancestor in (start or Path(__file__)).resolve().parents:
        if (ancestor / "Makefile").is_file() and (ancestor / "src/modules/navigator").is_dir():
            return ancestor
    return None


def binary_path(root: Path) -> Path:
    return root / BUILD_DIR / BINARY_NAME


def build_command(root: Path) -> str:
    location = shlex.quote(str(root / CLI_LOCATION))
    return f"make px4_sitl_test EXTERNAL_MODULES_LOCATION={location} {BINARY_NAME}"


def stale_sources(root: Path, binary: Path) -> List[str]:
    """Planner and CLI sources modified after the binary was built, relative to the repo."""
    if not binary.is_file():
        return []
    built = binary.stat().st_mtime
    stale = []
    for pattern in SOURCE_GLOBS:
        for source in sorted(root.glob(pattern)):
            if source.is_file() and source.stat().st_mtime > built:
                stale.append(str(source.relative_to(root)))
    return stale


def run(binary: Path, scenario: str, timeout_s: float = 20.0) -> PlanResult:
    completed = subprocess.run(
        [str(binary)], input=scenario, capture_output=True, text=True, timeout=timeout_s, check=False
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"{binary.name} exited with {completed.returncode}: {completed.stderr.strip() or completed.stdout.strip()}"
        )
    return parse_output(completed.stdout)
