"""Best effort reader for the C++ fixture syntax used by the mission route tests.

Nothing is executed. Whatever the reader does not understand is skipped, so a
whole TEST_F body can be pasted as is.
"""

import ast
from functools import lru_cache
import operator
from pathlib import Path
import re
from typing import Dict, Iterator, List, Optional, Tuple

from .geometry import offset_to_latlon
from .models import HELPER_HEADER, Position, Scene, Waypoint

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
_FLOAT_SUFFIX_RE = re.compile(r"((?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)[fF]\b")
_OPERATORS = {
    ast.Add: operator.add,
    ast.Sub: operator.sub,
    ast.Mult: operator.mul,
    ast.Div: operator.truediv,
}


def parse_cpp(text: str) -> Scene:
    """Read mission containers, safe points, positions and the request velocity."""
    src = _mask_strings(strip_comments(text or ""))
    constants = resolve_constants(src, header_constants())
    return Scene(
        missions=_parse_missions(src, constants),
        safe_points=_parse_safe_points(src, constants),
        positions=_parse_positions(src, constants),
        velocity=_parse_velocity(src, constants),
        constants=constants,
    )


@lru_cache(maxsize=1)
def header_constants() -> Dict[str, float]:
    """Constants of the shared test helper header, when this tool runs inside the repo."""
    for ancestor in Path(__file__).resolve().parents:
        header = ancestor / HELPER_HEADER
        if header.is_file():
            return resolve_constants(strip_comments(header.read_text(encoding="utf-8")), {})
    return {}


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

    missions = {name: items for name, items in missions.items() if items}
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
