"""Shared constants and helpers for conventional commit validation.

Format: type(scope): description
Optional breaking change marker: type(scope)!: description
"""

import re

CONVENTIONAL_TYPES = {
    'feat':     'A new feature',
    'fix':      'A bug fix',
    'docs':     'Documentation only changes',
    'style':    'Formatting, whitespace, no code change',
    'refactor': 'Code change that neither fixes a bug nor adds a feature',
    'perf':     'Performance improvement',
    'test':     'Adding or correcting tests',
    'build':    'Build system or external dependencies',
    'ci':       'CI configuration files and scripts',
    'chore':    'Other changes that don\'t modify src or test files',
    'revert':   'Reverts a previous commit',
}

# type(scope)[!]: description
# - type: one of CONVENTIONAL_TYPES keys
# - scope: required, alphanumeric with _/-/.
# - !: optional breaking change marker
# - description: at least 5 chars
HEADER_PATTERN = re.compile(
    r'^(' + '|'.join(CONVENTIONAL_TYPES.keys()) + r')'
    r'\(([a-zA-Z0-9_/\-\.]+)\)'
    r'(!)?'
    r': (.{5,})$'
)

EXEMPT_PREFIXES = ('Merge ',)

# Common PX4 subsystem scopes for suggestions
KNOWN_SCOPES = [
    'ekf2', 'mavlink', 'commander', 'navigator', 'sensors',
    'mc_att_control', 'mc_pos_control', 'mc_rate_control',
    'fw_att_control', 'fw_pos_control', 'fw_rate_control',
    'vtol', 'actuators', 'battery', 'param', 'logger',
    'uorb', 'drivers', 'boards', 'simulation', 'sitl',
    'gps', 'rc', 'safety', 'can', 'serial',
    'ci', 'docs', 'build', 'cmake', 'tools',
    'mixer', 'land_detector', 'airspeed', 'gyroscope',
    'accelerometer', 'magnetometer', 'barometer',
]

# Keyword patterns to suggest scopes from description text
KEYWORD_SCOPES = [
    (r'\b(ekf|estimator|height|fusion|imu|baro)\b', 'ekf2'),
    (r'\b(mavlink|MAVLink|MAVLINK|command_int|heartbeat)\b', 'mavlink'),
    (r'\b(uorb|orb|pub|sub|topic)\b', 'uorb'),
    (r'\b(board|fmu|nuttx|stm32)\b', 'boards'),
    (r'\b(mixer|actuator|motor|servo|pwm|dshot)\b', 'actuators'),
    (r'\b(battery|power)\b', 'battery'),
    (r'\b(param|parameter)\b', 'param'),
    (r'\b(log|logger|sdlog)\b', 'logger'),
    (r'\b(sensor|accel|gyro)\b', 'sensors'),
    (r'\b(land|takeoff|rtl|mission|navigator|geofence)\b', 'navigator'),
    (r'\b(position|velocity|attitude|rate)\s*(control|ctrl)\b', 'mc_att_control'),
    (r'\b(mc|multicopter|quad)\b', 'mc_att_control'),
    (r'\b(fw|fixedwing|fixed.wing|plane)\b', 'fw_att_control'),
    (r'\b(vtol|transition)\b', 'vtol'),
    (r'\b(ci|workflow|github.action|pipeline)\b', 'ci'),
    (r'\b(doc|docs|documentation|readme)\b', 'docs'),
    (r'\b(cmake|make|toolchain|compiler)\b', 'build'),
    (r'\b(sitl|simulation|gazebo|jmavsim|sih)\b', 'simulation'),
    (r'\b(can|uavcan|cyphal|dronecan)\b', 'can'),
    (r'\b(serial|uart|spi|i2c)\b', 'serial'),
    (r'\b(safety|failsafe|arm|disarm|kill)\b', 'safety'),
    (r'\b(rc|radio|sbus|crsf|elrs|dsm)\b', 'rc'),
    (r'\b(gps|gnss|rtk|ubx)\b', 'gps'),
    (r'\b(optical.flow|flow|rangefinder|lidar|distance)\b', 'sensors'),
    (r'\b(orbit|follow|offboard)\b', 'commander'),
    (r'\b(driver)\b', 'drivers'),
]

# Verb patterns to suggest conventional commit type
VERB_TYPE_MAP = [
    (r'^fix(e[ds])?[\s:]', 'fix'),
    (r'^bug[\s:]', 'fix'),
    (r'^add(s|ed|ing)?[\s:]', 'feat'),
    (r'^implement', 'feat'),
    (r'^introduce', 'feat'),
    (r'^support', 'feat'),
    (r'^enable', 'feat'),
    (r'^update[ds]?[\s:]', 'feat'),
    (r'^improv(e[ds]?|ing)', 'perf'),
    (r'^optimi[zs](e[ds]?|ing)', 'perf'),
    (r'^refactor', 'refactor'),
    (r'^clean\s*up', 'refactor'),
    (r'^restructure', 'refactor'),
    (r'^simplif(y|ied)', 'refactor'),
    (r'^remov(e[ds]?|ing)', 'refactor'),
    (r'^delet(e[ds]?|ing)', 'refactor'),
    (r'^deprecat', 'refactor'),
    (r'^replac(e[ds]?|ing)', 'refactor'),
    (r'^renam(e[ds]?|ing)', 'refactor'),
    (r'^migrat', 'refactor'),
    (r'^revert', 'revert'),
    (r'^doc(s|ument)', 'docs'),
    (r'^test', 'test'),
    (r'^format', 'style'),
    (r'^lint', 'style'),
    (r'^whitespace', 'style'),
    (r'^build', 'build'),
    (r'^ci[\s:]', 'ci'),
]


def parse_header(text: str) -> dict | None:
    """Parse a conventional commit header into components.

    Returns dict with keys {type, scope, breaking, subject} or None if
    the text doesn't match conventional commits format.
    """
    text = text.strip()
    m = HEADER_PATTERN.match(text)
    if not m:
        return None
    return {
        'type': m.group(1),
        'scope': m.group(2),
        'breaking': m.group(3) == '!',
        'subject': m.group(4),
    }


def suggest_type(text: str) -> str:
    """Infer a conventional commit type from description text."""
    lower = text.strip().lower()
    for pattern, commit_type in VERB_TYPE_MAP:
        if re.search(pattern, lower):
            return commit_type
    return 'feat'


def suggest_scope(text: str) -> str | None:
    """Infer a scope from keywords in the text."""
    lower = text.strip().lower()
    for pattern, scope in KEYWORD_SCOPES:
        if re.search(pattern, lower, re.IGNORECASE):
            return scope
    return None
