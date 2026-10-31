"""Best effort reader for the C++ fixture syntax used by the mission route tests.

Nothing is executed. Whatever the reader does not understand is skipped, so a
whole TEST_F body can be pasted as is.
"""

import ast
from functools import lru_cache
import math
import operator
from pathlib import Path
import re
from typing import Dict, Iterator, List, Optional, Tuple

from .geometry import offset_to_latlon
from .models import Fence, HELPER_HEADER, Position, Scene, Waypoint, base_from

# C++ sources the fixture geometry and the request defaults are read from, so they are not copied here.
GEOFENCE_HELPER_HEADER = "src/modules/navigator/test/support/geofence_test_helpers.h"
ROUTE_GEOFENCE_TESTS = "src/modules/navigator/test/test_mission_route_geofence.cpp"
PLANNER_TESTS = "src/modules/navigator/test/test_mission_route_planner.cpp"
ROUTE_TYPES = "src/modules/navigator/mission_route_types.h"

# Item helpers of mission_route_test_helpers.h and the nav command each one produces.
ITEM_HELPERS = {
    "makePositionItem": "NAV_CMD_WAYPOINT",
    "makePositionItemFromOffset": "NAV_CMD_WAYPOINT",
    "makeTakeoffItem": "NAV_CMD_TAKEOFF",
    "makeTakeoffItemFromOffset": "NAV_CMD_TAKEOFF",
    "makeLandItem": "NAV_CMD_LAND",
    "makeLandItemFromOffset": "NAV_CMD_LAND",
}
SAFE_POINT_HELPERS = ("makeSafePointAbsolute", "makeSafePointFromOffset")
POSITION_HELPERS = ("makePositionAbsolute", "makePositionFromOffset")

_STRING = r'"(?:\\.|[^"\\])*"'
_COMMENT_OR_STRING_RE = re.compile(_STRING + r"|/\*.*?\*/|//[^\n]*", re.DOTALL)
_STRING_RE = re.compile(_STRING)
_QUALIFIERS = r"(?:(?:static|inline|constexpr|const)\s+)*"
_VECTOR = r"std::vector<\s*mission_item_s\s*>"
_ARRAY = r"std::array<\s*mission_item_s\s*,\s*\d+\s*>"
_NUMERIC_DEF_RE = re.compile(
    _QUALIFIERS
    + r"\b(?:float|double|int|int32_t|uint16_t|uint8_t|size_t)\s+(\w+)\s*=\s*([^;]+);"
)
_CONTAINER_RE = re.compile(_QUALIFIERS + rf"(?:{_VECTOR}|{_ARRAY})\s+(\w+)\s*(?:=\s*)?\{{")
_AUTO_CONTAINER_RE = re.compile(rf"\bauto\s+(\w+)\s*=\s*{_VECTOR}\s*\{{")
_EMPTY_CONTAINER_RE = re.compile(_QUALIFIERS + rf"{_VECTOR}\s+(\w+)\s*;")
_FUNCTION_RE = re.compile(_QUALIFIERS + rf"{_VECTOR}\s+(\w+)\s*\([^)]*\)\s*\{{")
_PUSH_BACK_RE = re.compile(r"\b(\w+)\.push_back\s*\(")
_POSITION_VAR_RE = re.compile(
    _QUALIFIERS + r"mission_route::Position\s+(\w+)\s*(?:=\s*)?\{([^{}]*)\}"
)
_POSITION_LITERAL_RE = re.compile(r"mission_route::Position\s*\{([^{}]*)\}")
_VELOCITY_RE = re.compile(r"\bvelocity_(north|east)_m_s\s*=\s*([^;]+);")
_GEOFENCE_REFERENCE_RE = re.compile(
    r"\bGeofenceTestBase\s*\(\s*double\s+\w+\s*=\s*([^,()]+),\s*double\s+\w+\s*=\s*([^,()]+)\)"
)
_FENCE_SHAPE_RE = re.compile(r"\bFencePoints\s+(\w+)\s*\(\s*\)\s*const\s*\{")
_ROUTE_POSITION_DEFAULT_RE = re.compile(r"\broutePosition\s*\([^)]*=\s*([^,)]+)\)")
_MEMBER_DEFAULT_RE = re.compile(r"\b(\w+)\s*\{([^{}]*)\}\s*;")
_REQUEST_ASSIGNMENT_RE = re.compile(r"\brequest\.(\w+)\s*=\s*([^;]+);")
_FLOAT_SUFFIX_RE = re.compile(r"((?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)[fF]\b")
_OPERATORS = {
    ast.Add: operator.add,
    ast.Sub: operator.sub,
    ast.Mult: operator.mul,
    ast.Div: operator.truediv,
}


def parse_cpp(text: str) -> Scene:
    """Read mission containers, safe points, positions, fences and request velocity."""
    src = _mask_strings(strip_comments(text or ""))
    constants = resolve_constants(src, header_constants())
    return Scene(
        missions=_parse_missions(src, constants),
        safe_points=_parse_safe_points(src, constants),
        positions=_parse_positions(src, constants),
        velocity=_parse_velocity(src, constants),
        constants=constants,
        fences=_parse_fences(src, constants),
    )


@lru_cache(maxsize=None)
def repo_file(path: str) -> str:
    """A source file of the repository this tool sits in, comments blanked. Empty outside the repo."""
    for ancestor in Path(__file__).resolve().parents:
        source = ancestor / path
        if source.is_file():
            return strip_comments(source.read_text(encoding="utf-8"))
    return ""


@lru_cache(maxsize=1)
def header_constants() -> Dict[str, float]:
    """Constants of the shared test helper header, when this tool runs inside the repo."""
    return resolve_constants(repo_file(HELPER_HEADER), {})


@lru_cache(maxsize=1)
def geofence_reference() -> Optional[Tuple[float, float]]:
    """GeofenceTestBase's default reference, the origin of its north/east fixture offsets.

    Fixtures that pass a reference of their own are not detected.
    """
    match = _GEOFENCE_REFERENCE_RE.search(repo_file(GEOFENCE_HELPER_HEADER))
    if match is None:
        return None
    lat, lon = (resolve_number(value, {}) for value in match.groups())
    return None if lat is None or lon is None else (lat, lon)


@lru_cache(maxsize=1)
def fence_shapes() -> Dict[str, Tuple[str, List[str]]]:
    """GeofenceTestBase shapes without arguments, such as exclusionSquare(), as the call they return."""
    src = repo_file(GEOFENCE_HELPER_HEADER)
    shapes: Dict[str, Tuple[str, List[str]]] = {}
    for match in _FENCE_SHAPE_RE.finditer(src):
        call = next(_calls(_block_at(src, match.end() - 1), ("polygon", "circle")), None)
        if call is not None:
            shapes[match.group(1)] = (call[0], call[1])
    return shapes


@lru_cache(maxsize=1)
def route_position_altitude() -> Optional[float]:
    """The default altitude argument of routePosition() in the route geofence tests."""
    src = repo_file(ROUTE_GEOFENCE_TESTS)
    match = _ROUTE_POSITION_DEFAULT_RE.search(src)
    return resolve_number(match.group(1), resolve_constants(src, {})) if match else None


@lru_cache(maxsize=None)
def request_defaults(helper: str) -> Dict[str, float]:
    """Numeric request fields as a planner test helper such as makeRtlRouteRequest() returns them.

    The request struct defaults, then the helper's assignments. A field whose value cannot be
    read is missing, and so is every field outside the repo.
    """
    tests = repo_file(PLANNER_TESTS)
    definition = re.search(rf"\b{helper}\s*\([^{{;]*\{{", tests)
    if definition is None:
        return {}
    types = repo_file(ROUTE_TYPES)
    values: Dict[str, Optional[float]] = {}
    for struct in re.finditer(r"\bstruct\s+(?:MissionResumeRequest|RtlRouteRequest)\b[^{;]*\{", types):
        for name, expr in _MEMBER_DEFAULT_RE.findall(_block_at(types, struct.end() - 1)):
            values[name] = resolve_number(expr, {})
    for name, expr in _REQUEST_ASSIGNMENT_RE.findall(_block_at(tests, definition.end() - 1)):
        values[name] = resolve_number(expr, header_constants())
    return {name: value for name, value in values.items() if value is not None}


# ---- Tokens and expressions ----


def strip_comments(src: str) -> str:
    """Blank out comments while keeping strings and line breaks in place."""

    def blank(match: "re.Match") -> str:
        token = match.group()
        if token.startswith(("//", "/*")):
            return re.sub(r"[^\n]", " ", token)
        return token

    return _COMMENT_OR_STRING_RE.sub(blank, src)


def _mask_strings(src: str) -> str:
    # No fixture value lives in a string, so hide them from the bracket matching.
    return _STRING_RE.sub(lambda match: " " * len(match.group()), src)


def _closing_bracket(src: str, open_idx: int) -> int:
    """Index of the bracket closing the one at open_idx, or -1 when unbalanced."""
    opener = src[open_idx]
    closer = {"(": ")", "[": "]", "{": "}"}[opener]
    depth = 0
    for idx in range(open_idx, len(src)):
        if src[idx] == opener:
            depth += 1
        elif src[idx] == closer:
            depth -= 1
            if depth == 0:
                return idx
    return -1


def _block_at(src: str, open_idx: int) -> str:
    """Text between the bracket at open_idx and the one closing it, or "" when unbalanced."""
    close = _closing_bracket(src, open_idx)
    return src[open_idx + 1 : close] if close != -1 else ""


def _split_args(text: str) -> List[str]:
    """Split on the commas that are not nested inside brackets."""
    args: List[str] = []
    depth = 0
    start = 0
    for idx, char in enumerate(text):
        if char in "([{":
            depth += 1
        elif char in ")]}":
            depth -= 1
        elif char == "," and depth == 0:
            args.append(text[start:idx].strip())
            start = idx + 1
    tail = text[start:].strip()
    if tail:
        args.append(tail)
    return args


def _calls(src: str, names) -> Iterator[Tuple[str, List[str], int]]:
    """Yield (helper, argument strings, source index) for every call to one of names."""
    pattern = re.compile(r"\b(" + "|".join(names) + r")\s*\(")
    for match in pattern.finditer(src):
        close = _closing_bracket(src, match.end() - 1)
        if close != -1:
            yield match.group(1), _split_args(src[match.end() : close]), match.start()


def resolve_number(expr: str, constants: Dict[str, float]) -> Optional[float]:
    """Evaluate literals, known constants and + - * / between them. None when unknown."""
    expr = re.sub(r"\b\w+::", "", expr)
    expr = _FLOAT_SUFFIX_RE.sub(r"\1", expr)
    try:
        tree = ast.parse(expr.strip(), mode="eval")
    except (SyntaxError, ValueError, RecursionError):
        return None
    return _evaluate(tree.body, constants)


def _evaluate(node: ast.AST, constants: Dict[str, float]) -> Optional[float]:
    if isinstance(node, ast.Constant) and type(node.value) in (int, float):
        return float(node.value)
    if isinstance(node, ast.Name):
        return constants.get(node.id)
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
        value = _evaluate(node.operand, constants)
        if value is None:
            return None
        return -value if isinstance(node.op, ast.USub) else value
    if isinstance(node, ast.BinOp) and type(node.op) in _OPERATORS:
        left = _evaluate(node.left, constants)
        right = _evaluate(node.right, constants)
        if left is None or right is None or (isinstance(node.op, ast.Div) and right == 0):
            return None
        return _OPERATORS[type(node.op)](left, right)
    return None


def resolve_constants(src: str, defaults: Dict[str, float]) -> Dict[str, float]:
    """Numeric const/constexpr definitions, resolved in dependency order."""
    constants = dict(defaults)
    pending = [(match.group(1), match.group(2)) for match in _NUMERIC_DEF_RE.finditer(src)]
    for name, _ in pending:
        constants.pop(name, None)  # a local definition overrides the header default
    while pending:
        unresolved = []
        for name, expr in pending:
            value = resolve_number(expr, constants)
            if value is None:
                unresolved.append((name, expr))
            else:
                constants[name] = value
        if len(unresolved) == len(pending):
            break
        pending = unresolved
    return constants


# ---- Fixtures ----


def _waypoint(helper: str, args: List[str], cmd: str, constants: Dict[str, float]) -> Optional[Waypoint]:
    """Build a waypoint from helper arguments, applying offsets like the C++ helper."""
    from_offset = helper.endswith("FromOffset")
    needed = 5 if from_offset else 3
    values = [resolve_number(arg, constants) for arg in args[:needed]]
    if len(values) < needed or None in values:
        return None
    if cmd == "NAV_CMD_WAYPOINT" and len(args) > needed:
        cmd = args[needed]  # explicit nav command such as NAV_CMD_LOITER_TO_ALT
    if from_offset:
        lat, lon = offset_to_latlon(values[0], values[1], values[2], values[3])
        return Waypoint(lat, lon, values[4], cmd)
    return Waypoint(values[0], values[1], values[2], cmd)


def _item_from_call(call: str, constants: Dict[str, float]) -> Optional[Waypoint]:
    call = call.strip()
    match = re.match(r"(\w+)\s*\(", call)
    if match is None or match.group(1) not in ITEM_HELPERS:
        return None
    close = _closing_bracket(call, match.end() - 1)
    if close == -1:
        return None
    args = _split_args(call[match.end() : close])
    return _waypoint(match.group(1), args, ITEM_HELPERS[match.group(1)], constants)


def _items(initializer: str, constants: Dict[str, float]) -> List[Waypoint]:
    """Positional items of a brace initializer; DO_JUMP and transitions are skipped."""
    items = [_item_from_call(arg, constants) for arg in _split_args(initializer)]
    return [item for item in items if item is not None]


def _parse_missions(src: str, constants: Dict[str, float]) -> Dict[str, List[Waypoint]]:
    missions: Dict[str, List[Waypoint]] = {}
    declarations: List[Tuple[str, str, int]] = []  # raw name, unique name, source index

    def declare(raw_name: str, index: int) -> str:
        # Two pasted tests may both call their vector `mission`; keep both.
        name = raw_name
        suffix = 2
        while name in missions:
            name = f"{raw_name}_{suffix}"
            suffix += 1
        missions[name] = []
        declarations.append((raw_name, name, index))
        return name

    for pattern in (_CONTAINER_RE, _AUTO_CONTAINER_RE):
        for match in pattern.finditer(src):
            close = _closing_bracket(src, match.end() - 1)
            if close == -1:
                continue
            body = src[match.end() : close].strip()
            if body.startswith("{") and body.endswith("}"):
                body = body[1:-1]  # std::array double braces
            missions[declare(match.group(1), match.start())] = _items(body, constants)

    for match in _EMPTY_CONTAINER_RE.finditer(src):
        declare(match.group(1), match.start())

    for match in _FUNCTION_RE.finditer(src):
        close = _closing_bracket(src, match.end() - 1)
        returned = re.search(r"\breturn\s*\{", src[match.end() : close]) if close != -1 else None
        if returned is None:
            continue
        list_open = match.end() + returned.end() - 1
        list_close = _closing_bracket(src, list_open)
        if list_close != -1:
            name = declare(match.group(1), match.start())
            missions[name] = _items(src[list_open + 1 : list_close], constants)

    for match in _PUSH_BACK_RE.finditer(src):
        owners = [
            (index, name)
            for raw_name, name, index in declarations
            if raw_name == match.group(1) and index < match.start()
        ]
        close = _closing_bracket(src, match.end() - 1)
        if not owners or close == -1:
            continue
        item = _item_from_call(src[match.end() : close], constants)
        if item is not None:
            missions[max(owners)[1]].append(item)

    # Source order, so the first mission is the one declared first.
    order = {name: index for _, name, index in declarations}
    missions = {name: missions[name] for name in sorted(missions, key=order.__getitem__) if missions[name]}
    if missions:
        return missions

    # No named container, e.g. items passed straight to a call: keep them in source order.
    loose = [
        _waypoint(helper, args, ITEM_HELPERS[helper], constants)
        for helper, args, _ in _calls(src, ITEM_HELPERS)
    ]
    loose = [item for item in loose if item is not None]
    return {"mission": loose} if loose else {}


def _parse_safe_points(src: str, constants: Dict[str, float]) -> List[Waypoint]:
    points = [
        _waypoint(helper, args, "NAV_CMD_RALLY_POINT", constants)
        for helper, args, _ in _calls(src, SAFE_POINT_HELPERS)
    ]
    return [point for point in points if point is not None]


def _fence(inclusion: bool, vertices, radius=None) -> Optional[Fence]:
    """Ignore incomplete or invalid geometry instead of drawing a different fence."""
    fence = Fence(inclusion, vertices, radius)
    return None if fence.problem() else fence


def _coordinate_literal(expr: str, constants: Dict[str, float]):
    """A {north, east} offset, as passed to the geofence fixture helpers."""
    match = re.fullmatch(r"\{([^{}]*)\}", expr.strip())
    if match is None:
        return None
    values = [resolve_number(arg, constants) for arg in _split_args(match.group(1))]
    return tuple(values) if len(values) == 2 and None not in values else None


def _parse_fences(src: str, constants: Dict[str, float]) -> List[Fence]:
    fences = []
    pending = []
    pending_command, pending_count, pending_start, previous_end = "", 0, 0, 0

    # Generated records encode a polygon as consecutive vertices with the same
    # command and vertex count. Do not join records from different containers.
    for _, args, index in _calls(src, ("makeFencePoint",)):
        if src[previous_end:index].strip(" ,\t\r\n"):
            pending = []
        previous_end = _closing_bracket(src, src.index("(", index)) + 1
        command = re.fullmatch(r"NAV_CMD_FENCE_(POLYGON_VERTEX|CIRCLE)_(INCLUSION|EXCLUSION)", args[0]) if args else None
        if command is None or len(args) != 5:
            pending = []
            continue
        lat, lon, count, radius = [resolve_number(arg, constants) for arg in args[1:]]
        coordinate = (lat, lon) if lat is not None and lon is not None else None
        inclusion = command.group(2) == "INCLUSION"
        if command.group(1) == "CIRCLE":
            fence = _fence(inclusion, [coordinate], radius) if radius is not None else None
            if fence is not None:
                fences.append((index, fence))
            pending = []
            continue
        if count is None or not math.isfinite(count) or not count.is_integer() or not 3 <= count <= 65535:
            pending = []
            continue
        if not pending or args[0] != pending_command or count != pending_count:
            pending, pending_command, pending_count, pending_start = [], args[0], count, index
        pending.append(coordinate)
        if len(pending) == pending_count:
            fence = _fence(inclusion, pending)
            if fence is not None:
                fences.append((pending_start, fence))
            pending = []

    # GeofenceTestBase helpers use offsets from its default reference, not kBaseLat/kBaseLon.
    # The reference and argument-free shapes such as exclusionSquare() are read from its header.
    shapes = fence_shapes()
    base = base_from(constants)
    for helper, args, index in _calls(src, ("polygon", "circle", "exclusionCircle", *shapes)):
        reference = geofence_reference()
        radius = None
        offsets = []
        inclusion = False
        if helper in shapes and not args:
            helper, args = shapes[helper]
        if helper == "exclusionCircle" and len(args) == 3:
            values = [resolve_number(arg, constants) for arg in args]
            if None in values:
                continue
            offsets, radius = [tuple(values[:2])], values[2]
            reference = (base.lat, base.lon) if base else None
        elif len(args) in (2, 3) and args[0] in ("true", "false"):
            inclusion = args[0] == "true"
            if helper == "polygon" and len(args) == 2 and args[1].startswith("{") and args[1].endswith("}"):
                offsets = [_coordinate_literal(value, constants) for value in _split_args(args[1][1:-1])]
            elif helper == "circle" and len(args) == 3:
                offsets, radius = [_coordinate_literal(args[1], constants)], resolve_number(args[2], constants)
                if radius is None:
                    continue
        if reference is None or not offsets or None in offsets:
            continue
        vertices = [offset_to_latlon(*reference, *point) for point in offsets]
        fence = _fence(inclusion, vertices, radius)
        if fence is not None:
            fences.append((index, fence))
    return [fence for _, fence in sorted(fences, key=lambda entry: entry[0])]


def _assigned_name(src: str, index: int) -> Optional[str]:
    """Variable or member receiving the value starting at index, as in `x = <value>`."""
    match = re.search(r"(\w+)\s*=\s*$", src[:index])
    return match.group(1) if match else None


def _parse_positions(src: str, constants: Dict[str, float]) -> List[Position]:
    positions: List[Position] = []

    def add(name: Optional[str], lat: float, lon: float, alt: float) -> None:
        positions.append(Position(name or f"position_{len(positions)}", lat, lon, alt))

    for helper, args, index in _calls(src, POSITION_HELPERS):
        point = _waypoint(helper, args, "", constants)
        if point is not None:
            add(_assigned_name(src, index), point.lat, point.lon, point.alt)

    reference = geofence_reference()
    for _, args, index in _calls(src, ("routePosition",)):
        values = [resolve_number(arg, constants) for arg in args]
        if len(values) == 2:
            values.append(route_position_altitude())
        if reference and len(values) == 3 and None not in values and all(math.isfinite(value) for value in values):
            lat, lon = offset_to_latlon(*reference, *values[:2])
            add(_assigned_name(src, index), lat, lon, values[2])

    for match in _POSITION_VAR_RE.finditer(src):
        values = [resolve_number(arg, constants) for arg in _split_args(match.group(2))]
        if len(values) == 3 and None not in values:
            add(match.group(1), *values)

    for match in _POSITION_LITERAL_RE.finditer(src):
        values = [resolve_number(arg, constants) for arg in _split_args(match.group(1))]
        if len(values) == 3 and None not in values:
            add(_assigned_name(src, match.start()), *values)

    return positions


def _parse_velocity(src: str, constants: Dict[str, float]) -> Optional[Tuple[float, float]]:
    found = {}
    for match in _VELOCITY_RE.finditer(src):
        value = resolve_number(match.group(2), constants)
        if value is not None:
            found[match.group(1)] = value
    if "north" in found and "east" in found:
        return found["north"], found["east"]
    return None
