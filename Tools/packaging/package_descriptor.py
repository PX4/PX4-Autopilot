#!/usr/bin/env python3
"""Board package descriptors: boards/<vendor>/<board>/package.yaml.

A descriptor says how PX4 is packaged for a board whose Linux side is part
of the product: which build targets go into the package, where their
outputs land on the target filesystem, how coprocessor images reach their
core, and what the systemd unit looks like. The package format is a
backend; the install tree, the hooks and the unit are the contract.

This module validates descriptors (validation/package_schema.yaml plus the
cross-field rules below) and emits what Tools/packaging/CMakeLists.txt
consumes: the install rules, the hook runtime environment, the unit, the
Debian maintainer scripts and the package.json manifest.

    package_descriptor.py validate --all | <board-or-path>...
    package_descriptor.py cmake-vars <board-or-path> --output FILE
    package_descriptor.py emit <board-or-path> --output-dir DIR [--build-root DIR]
                                [--version V | --git-tag TAG]
    package_descriptor.py list [--coprocessor-targets]

<board-or-path> is <vendor>_<board> or a path to a package.yaml.
"""

import argparse
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

import yaml

PX4_ROOT = Path(__file__).resolve().parents[2]
SCHEMA_FILE = PX4_ROOT / 'validation' / 'package_schema.yaml'
PACKAGING_DIR = Path(__file__).resolve().parent

# Where the hook runtime lives on the target. Fixed: maintainer scripts and
# the unit refer to it, and two PX4 packages cannot be installed at once anyway.
LIB_DIR = '/usr/lib/px4'
UNIT_DIR = '/lib/systemd/system'
# Hook points dispatched by LIB_DIR/run-hooks. Package-time points are only
# the ones whose scripts are on disk when dpkg runs them: a new package's
# preinst and an old package's postrm run without the package's files.
HOOK_POINTS = ('post-install', 'pre-remove', 'apply-firmware', 'pre-start', 'post-stop')
HOOK_NAME = re.compile(r'^[A-Za-z0-9_-]+$')

TOOLCHAIN_ARCHITECTURES = {'aarch64-linux-gnu': 'arm64', 'arm-linux-gnueabihf': 'armhf'}
MACHINE_ARCHITECTURES = {'x86_64': 'amd64', 'amd64': 'amd64', 'aarch64': 'arm64',
                         'arm64': 'arm64', 'armv7l': 'armhf'}
# Component metadata the host build produces next to the binary.
METADATA_FILES = ('parameters.json.xz', 'actuators.json.xz', 'component_general.json.xz',
                  'events/all_events.json.xz')

# vX.Y.Z, an optional pre-release suffix that starts with a letter, and the
# optional -<commits>-g<hash> that git describe appends off a tag
GIT_TAG = re.compile(r'^v?(\d+\.\d+\.\d+)(?:-([A-Za-z].*?))?(?:-(\d+)-g([0-9a-f]+))?$')


class DescriptorError(Exception):
    """A descriptor that cannot be packaged, with every problem found."""

    def __init__(self, path, problems):
        self.path = path
        self.problems = list(problems)
        super().__init__('\n'.join(f'{path}: {p}' for p in self.problems))


def debian_version(git_tag):
    """Debian version for a `git describe` tag.

    v1.17.0 -> 1.17.0, v1.17.0-beta1 -> 1.17.0~beta1 (sorts before the
    release), v1.17.0-beta1-42-gabcdef -> 1.17.0~beta1.42.gabcdef and
    v1.17.0-42-gabcdef -> 1.17.0.42.gabcdef (sort after their base). This
    is the scheme the VOXL2 package has shipped with.
    """
    match = GIT_TAG.match(git_tag.strip())
    if not match:
        raise ValueError(f'cannot derive a Debian version from git tag {git_tag!r}')
    base, prerelease, commits, commit_hash = match.groups()
    version = base
    if prerelease:
        version += '~' + prerelease.replace('-', '~')
    if commits:
        version += f'.{commits}.g{commit_hash}'
    return version


def git_describe(source_dir):
    """The tag CMake uses for PX4_GIT_TAG, with the same v0.0.0 fallback."""
    result = subprocess.run(['git', 'describe', '--exclude', 'ext/*', '--tags', '--match', 'v[0-9]*'],
                            cwd=source_dir, capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 and result.stdout.strip() else 'v0.0.0'


def host_architecture():
    """Debian architecture of the build host, for native host components."""
    if shutil.which('dpkg'):
        return subprocess.run(['dpkg', '--print-architecture'], capture_output=True,
                              text=True, check=True).stdout.strip()
    machine = os.uname().machine
    if machine not in MACHINE_ARCHITECTURES:
        raise ValueError(f'unknown host machine {machine!r}; set package.architecture')
    return MACHINE_ARCHITECTURES[machine]


def px4board_config(path):
    """CONFIG_* assignments of a .px4board file, quotes stripped."""
    config = {}
    for line in Path(path).read_text().splitlines():
        match = re.match(r'^(CONFIG_\w+)=(.*)$', line.strip())
        if match:
            config[match.group(1)] = match.group(2).strip().strip('"')
    return config


def cmake_quote(value):
    return '"' + str(value).replace('\\', '\\\\').replace('"', '\\"').replace(';', '\\;') + '"'


def cmake_permissions(mode):
    """install() PERMISSIONS tokens for an octal mode string such as 0755."""
    bits = int(mode, 8)
    names = (('OWNER', 0o400, 0o200, 0o100), ('GROUP', 0o040, 0o020, 0o010),
             ('WORLD', 0o004, 0o002, 0o001))
    return ' '.join(f'{who}_{what}' for who, r, w, x in names
                    for what, bit in (('READ', r), ('WRITE', w), ('EXECUTE', x)) if bits & bit)


def board_dir(source_dir, board):
    """boards/<vendor>/<model> for <vendor>_<model>; neither part contains '_'."""
    vendor, _, model = board.partition('_')
    if not vendor or not model:
        raise ValueError(f'{board!r} is not <vendor>_<board>')
    return Path(source_dir) / 'boards' / vendor / model


def descriptor_paths(source_dir=PX4_ROOT):
    """Every boards/*/*/package.yaml, sorted."""
    return sorted((Path(source_dir) / 'boards').glob('*/*/package.yaml'))


def substitute(template, values):
    """Replace @KEY@ placeholders; an unknown key is a template bug."""
    def replace(match):
        key = match.group(1)
        if key not in values:
            raise KeyError(f'template placeholder @{key}@ has no value')
        return str(values[key])
    return re.sub(r'@([A-Z0-9_]+)@', replace, template)


class InstallRule:
    """One CMake install() command: kind is FILES, PROGRAMS or DIRECTORY."""

    def __init__(self, kind, source, destination, rename=None, mode=None, optional=False, pattern=None):
        self.kind, self.source, self.destination = kind, str(source), destination
        self.rename, self.mode, self.optional, self.pattern = rename, mode, optional, pattern

    def cmake(self):
        parts = [f'install({self.kind} {cmake_quote(self.source)}', f'DESTINATION {cmake_quote(self.destination)}']
        if self.rename:
            parts.append(f'RENAME {cmake_quote(self.rename)}')
        if self.mode:
            parts.append(f'PERMISSIONS {cmake_permissions(self.mode)}')
        if self.kind == 'DIRECTORY':
            parts.append('USE_SOURCE_PERMISSIONS')
        if self.optional:
            parts.append('OPTIONAL')
        if self.pattern:  # install(DIRECTORY) takes FILES_MATCHING last
            parts.append(f'FILES_MATCHING PATTERN {cmake_quote(self.pattern)}')
        return ' '.join(parts) + ')'


class Descriptor:
    """A loaded package.yaml. Call validate() before using anything else."""

    def __init__(self, path):
        self.path = Path(path).resolve()
        self.board_dir = self.path.parent
        self.model = self.board_dir.name
        self.vendor = self.board_dir.parent.name
        self.source_dir = self.board_dir.parents[1]
        self.board = f'{self.vendor}_{self.model}'
        with open(self.path) as stream:
            self.raw = yaml.safe_load(stream) or {}
        self.doc = {}  # normalized document, filled by validate()

    @classmethod
    def from_board(cls, board, source_dir=PX4_ROOT):
        return cls(board_dir(source_dir, board) / 'package.yaml')

    @classmethod
    def from_arg(cls, board_or_path, source_dir=PX4_ROOT):
        if board_or_path.endswith('.yaml') or os.sep in board_or_path:
            return cls(board_or_path)
        return cls.from_board(board_or_path, source_dir)

    # Reading without validation, for CI target discovery (no cerberus there).
    @property
    def coprocessor_targets(self):
        return [c['target'] for c in self.raw.get('components', [])
                if isinstance(c, dict) and c.get('role') == 'coprocessor' and 'target' in c]

    @property
    def component_targets(self):
        return [c['target'] for c in self.raw.get('components', [])
                if isinstance(c, dict) and 'target' in c]

    # Validation ---------------------------------------------------------

    def validate(self):
        problems = self._validate_schema()
        if problems:
            raise DescriptorError(self.path, problems)
        problems = self._validate_rules()
        if problems:
            raise DescriptorError(self.path, problems)
        return self

    def _validate_schema(self):
        import cerberus  # only needed to validate; CI target discovery imports this module without it
        with open(SCHEMA_FILE) as stream:
            schema = yaml.safe_load(stream)
        doc = dict(self.raw)
        # cerberus applies nested defaults only to dicts that exist
        for section in ('layout', 'service'):
            doc.setdefault(section, {})
        doc.setdefault('package', {}).setdefault('formats', {}).setdefault('deb', {})
        doc.setdefault('files', [])
        validator = cerberus.Validator(schema)
        if not validator.validate(doc):
            return [f'schema: {validator.errors}']
        self.doc = validator.document
        return []

    def _validate_rules(self):
        problems = []
        package, layout, service = self.doc['package'], self.doc['layout'], self.doc['service']
        package.setdefault('name', f'px4-{self.vendor}-{self.model}')
        service.setdefault('name', package['name'])

        for legacy in ('cmake/package.cmake', 'cmake/install.cmake', 'companion_targets'):
            if (self.board_dir / legacy).exists():
                problems.append(f'{legacy} conflicts with package.yaml; the descriptor replaces it')

        components = self.doc['components']
        names = [c['name'] for c in components]
        targets = [c['target'] for c in components]
        problems += [f'component name {n!r} is used more than once' for n in sorted(set(names)) if names.count(n) > 1]
        problems += [f'target {t!r} is used more than once' for t in sorted(set(targets)) if targets.count(t) > 1]
        hosts = [c for c in components if c['role'] == 'host']
        if len(hosts) > 1:
            problems.append('more than one host component')

        for component in components:
            problems += self._validate_component(component, layout)

        host = hosts[0] if hosts else None
        if host and 'exec_start' not in service:
            problems.append('service.exec_start is required with a host component')
        if not host and 'exec_start' in service:
            problems.append('service.exec_start needs a host component')

        if host and (self.board_dir / f'{self._label(host["target"])}.px4board').exists():
            # PX4 compiles its storage path in from CONFIG_BOARD_ROOT_PATH, whose
            # Kconfig default (/fs/microsd) is the NuttX SD card.
            root_path = px4board_config(self.board_dir / f'{self._label(host["target"])}.px4board').get('CONFIG_BOARD_ROOT_PATH')
            if not root_path:
                problems.append(f'{host["target"]} does not set CONFIG_BOARD_ROOT_PATH; PX4 stores parameters, logs '
                                f'and dataman there, and the default /fs/microsd is for NuttX')
            elif layout.get('data_dir', root_path) != root_path:
                problems.append(f'layout.data_dir {layout["data_dir"]} differs from CONFIG_BOARD_ROOT_PATH '
                                f'{root_path} of {host["target"]}; PX4 writes to the latter')
            layout.setdefault('data_dir', root_path or '/var/lib/px4')
        layout.setdefault('data_dir', '/var/lib/px4')

        if 'architecture' not in package:
            try:
                package['architecture'] = self._derive_architecture(host)
            except ValueError as error:
                problems.append(str(error))

        for entry in self.doc['files']:
            if entry['source'].startswith('build:'):
                component, _, _ = entry['source'][len('build:'):].partition('/')
                if component not in names:
                    problems.append(f'files: {entry["source"]} names an unknown component')
            elif not entry['optional'] and not (self.board_dir / entry['source']).is_file():
                problems.append(f'files: {entry["source"]} does not exist in {self.board_dir}')

        problems += self._validate_hooks()
        return problems

    def _label(self, target):
        prefix = self.board + '_'
        return target[len(prefix):] if target.startswith(prefix) else None

    def _validate_component(self, component, layout):
        problems = []
        label = self._label(component['target'])
        if not label or not (self.board_dir / f'{label}.px4board').is_file():
            problems.append(f'component {component["name"]}: target {component["target"]} is not a '
                            f'{self.board}_<label> with a {label or "<label>"}.px4board in {self.board_dir}')
        if component['role'] == 'host':
            for key in ('artifact', 'install', 'apply', 'remoteproc'):
                if key in component:
                    problems.append(f'component {component["name"]}: {key} is only for coprocessors')
            if label and (self.board_dir / f'{label}.px4board').is_file():
                if px4board_config(self.board_dir / f'{label}.px4board').get('CONFIG_PLATFORM_POSIX') != 'y':
                    problems.append(f'component {component["name"]}: a host component must be a POSIX target')
            return problems
        for key in ('artifact', 'install', 'apply'):
            if key not in component:
                problems.append(f'component {component["name"]}: coprocessor needs {key}')
        if component.get('apply') == 'remoteproc':
            if 'remoteproc' not in component:
                problems.append(f'component {component["name"]}: apply remoteproc needs remoteproc')
            firmware_dir = layout['firmware_dir'].rstrip('/') + '/'
            if 'install' in component and not component['install'].startswith(firmware_dir):
                problems.append(f'component {component["name"]}: remoteproc images install under '
                                f'{layout["firmware_dir"]}, not {component["install"]}')
        elif 'remoteproc' in component:
            problems.append(f'component {component["name"]}: remoteproc is only for apply remoteproc')
        if component.get('install', '/x').endswith('/'):
            problems.append(f'component {component["name"]}: install is a file path, not a directory')
        return problems

    def _validate_hooks(self):
        hooks_dir = self.board_dir / 'hooks'
        if not hooks_dir.is_dir():
            return []
        problems = []
        for entry in sorted(hooks_dir.iterdir()):
            point = entry.name[:-len('.d')] if entry.name.endswith('.d') else None
            if not entry.is_dir() or point not in HOOK_POINTS:
                problems.append(f'hooks/{entry.name}: expected one of {", ".join(p + ".d" for p in HOOK_POINTS)}')
                continue
            for script in sorted(entry.iterdir()):
                relative = f'hooks/{entry.name}/{script.name}'
                if not HOOK_NAME.match(script.name):
                    problems.append(f'{relative}: hook names are [A-Za-z0-9_-]+ with no extension')
                if not script.is_file() or not os.access(script, os.X_OK):
                    problems.append(f'{relative}: hooks must be executable files')
                    continue
                with open(script, 'rb') as handle:
                    shebang = handle.readline().rstrip(b'\r\n')
                if shebang != b'#!/bin/sh':
                    problems.append(f'{relative}: hooks are POSIX sh and start with #!/bin/sh')
        return problems

    def _derive_architecture(self, host):
        if host is None:
            return 'all'
        label = self._label(host['target'])
        toolchain = px4board_config(self.board_dir / f'{label}.px4board').get('CONFIG_BOARD_TOOLCHAIN')
        if not toolchain or toolchain == 'native':
            return host_architecture()
        if toolchain not in TOOLCHAIN_ARCHITECTURES:
            raise ValueError(f'no Debian architecture known for toolchain {toolchain!r}; set package.architecture')
        return TOOLCHAIN_ARCHITECTURES[toolchain]

    # Accessors -------------------------------------------------------------

    @property
    def package(self):
        return self.doc['package']

    @property
    def layout(self):
        return self.doc['layout']

    @property
    def service(self):
        return self.doc['service']

    @property
    def components(self):
        return self.doc['components']

    @property
    def host(self):
        return next((c for c in self.components if c['role'] == 'host'), None)

    @property
    def coprocessors(self):
        return [c for c in self.components if c['role'] == 'coprocessor']

    @property
    def hooks(self):
        """{point: [script names]} for the hook scripts the board ships."""
        hooks = {}
        for point in HOOK_POINTS:
            directory = self.board_dir / 'hooks' / f'{point}.d'
            if directory.is_dir():
                hooks[point] = sorted(p.name for p in directory.iterdir() if p.is_file())
        return hooks

    def build_dir(self, component, build_root):
        return Path(build_root) / component['target']

    # Emitters ---------------------------------------------------------------

    def install_rules(self, build_root):
        """Board-specific install rules; the runtime shared by every package is in CMakeLists.txt."""
        layout, rules = self.layout, []
        host = self.host
        if host:
            build = self.build_dir(host, build_root)
            rules.append(InstallRule('PROGRAMS', build / 'bin' / 'px4', layout['bin_dir'], optional=host['optional']))
            rules.append(InstallRule('DIRECTORY', f'{build / "bin"}/', layout['bin_dir'], pattern='px4-*', optional=True))
            if host['romfs']:
                rules.append(InstallRule('DIRECTORY', build / 'etc', layout['share_dir'], optional=True))
            if host['metadata']:
                for name in METADATA_FILES:
                    rules.append(InstallRule('FILES', build / name, f'{layout["share_dir"]}/etc/extras', optional=True))
        for component in self.coprocessors:
            artifact = self.build_dir(component, build_root) / component['artifact']
            rules.append(InstallRule('FILES', artifact, os.path.dirname(component['install']),
                                     rename=os.path.basename(component['install']), optional=component['optional']))
        for entry in self.doc['files']:
            rules.append(self._file_rule(entry, build_root))
        if self.hooks:
            rules.append(InstallRule('DIRECTORY', f'{self.board_dir / "hooks"}/', f'{LIB_DIR}/hooks'))
        return rules

    def _file_rule(self, entry, build_root):
        source = entry['source']
        if source.startswith('build:'):
            component, _, relative = source[len('build:'):].partition('/')
            source = self.build_dir(next(c for c in self.components if c['name'] == component), build_root) / relative
        else:
            source = self.board_dir / source
        destination, rename = entry['destination'], None
        if not destination.endswith('/'):
            destination, rename = os.path.dirname(destination), os.path.basename(destination)
        executable = source.is_file() and os.access(source, os.X_OK)
        kind = 'PROGRAMS' if executable and not entry.get('mode') else 'FILES'
        return InstallRule(kind, source, destination.rstrip('/') or '/', rename=rename,
                           mode=entry.get('mode'), optional=entry['optional'])

    def check_artifacts(self, build_root):
        """Every non-optional input exists; run after the component builds."""
        missing = []
        for rule in self.install_rules(build_root):
            if not rule.optional and not Path(rule.source).exists():
                missing.append(f'missing {rule.source}')
        if missing:
            raise DescriptorError(self.path, missing)

    def cmake_variables(self, version=None, git_tag=None):
        package, layout = self.package, self.layout
        deb = package['formats'].get('deb', {})
        variables = {
            'BOARD': self.board, 'BOARD_DIR': self.board_dir, 'NAME': package['name'],
            'VERSION': version or '', 'GIT_TAG': git_tag or '',
            'ARCHITECTURE': package['architecture'], 'DESCRIPTION': package['description'],
            'MAINTAINER': package['maintainer'], 'SECTION': package['section'],
            'SERVICE_NAME': self.service['name'],
        }
        variables.update({key.upper(): value for key, value in layout.items()})
        variables.update({f'DEB_{key.upper()}': ', '.join(deb.get(key, []))
                          for key in ('depends', 'recommends', 'conflicts', 'replaces', 'provides')})
        return variables

    def cmake(self, build_root, version, git_tag):
        lines = [f'# Generated by {Path(__file__).name} from {self.path}; do not edit.']
        lines += [f'set(PX4_PACKAGE_{key} {cmake_quote(value)})' for key, value in self.cmake_variables(version, git_tag).items()]
        lines.append('')
        lines += [rule.cmake() for rule in self.install_rules(build_root)]
        return '\n'.join(lines) + '\n'

    def environment(self, version, git_tag):
        """Shell-sourceable facts for run-hooks and the hook scripts."""
        layout = self.layout
        values = [('PX4_PACKAGE', self.package['name']), ('PX4_BOARD', self.board),
                  ('PX4_BOARD_VENDOR', self.vendor), ('PX4_BOARD_MODEL', self.model),
                  ('PX4_VERSION', version), ('PX4_GIT_TAG', git_tag), ('PX4_SERVICE', self.service['name'])]
        values += [(f'PX4_{key.upper()}', value) for key, value in layout.items()]
        values.append(('PX4_HOST_COMPONENT', self.host['name'] if self.host else ''))
        values.append(('PX4_COPROCESSORS', ' '.join(c['name'] for c in self.coprocessors)))
        for component in self.coprocessors:
            prefix = f'PX4_COPROCESSOR_{component["name"]}'
            values += [(f'{prefix}_TARGET', component['target']), (f'{prefix}_FIRMWARE', component['install']),
                       (f'{prefix}_APPLY', component['apply']), (f'{prefix}_REMOTEPROC', component.get('remoteproc', ''))]
        return '\n'.join(f'{key}={shlex.quote(str(value))}' for key, value in values) + '\n'

    def manifest(self, version, git_tag):
        """package.json: what the package contains, for tooling and the firmware manifest."""
        def describe(component):
            info = {key: component[key] for key in ('name', 'role', 'target')}
            if component['role'] == 'host':
                info['binary'] = f'{self.layout["bin_dir"]}/px4'
            else:
                info.update({key: component[key] for key in ('artifact', 'install', 'apply')})
                if 'remoteproc' in component:
                    info['remoteproc'] = component['remoteproc']
            return info

        package = {key: self.package[key] for key in ('name', 'description', 'maintainer', 'architecture', 'section', 'formats')}
        package['version'] = version
        return {
            'format': 1,
            'package': package,
            'board': {'name': self.board, 'vendor': self.vendor, 'model': self.model},
            'px4': {'git_tag': git_tag},
            'layout': dict(self.layout),
            'service': {key: self.service[key] for key in ('name', 'enable') + (('exec_start',) if self.host else ())},
            'components': [describe(c) for c in self.components],
            'hooks': self.hooks,
        }

    def unit(self):
        """systemd unit: hooks around the vendor's ExecStart, oneshot without a host."""
        service, lines = self.service, ['[Unit]', f'Description={self.package["description"]}']
        for key in ('after', 'requires', 'wants'):
            if service.get(key):
                lines.append(f'{key.capitalize()}={" ".join(service[key])}')
        lines += ['', '[Service]']
        if self.host:
            lines.append('Type=simple')
        else:
            lines += ['Type=oneshot', 'RemainAfterExit=yes']
        if service.get('user'):
            lines.append(f'User={service["user"]}')
        lines += [f'ExecStartPre=+{LIB_DIR}/run-hooks apply-firmware', f'ExecStartPre=+{LIB_DIR}/run-hooks pre-start']
        if self.host:
            lines.append(f'ExecStart={service["exec_start"]}')
        lines.append(f'ExecStopPost=+{LIB_DIR}/run-hooks post-stop')
        if self.host and service['restart'] != 'no':
            lines.append(f'Restart={service["restart"]}')
        lines += ['', '[Install]', 'WantedBy=multi-user.target', '']
        return '\n'.join(lines)

    def maintainer_script(self, name):
        template = (PACKAGING_DIR / 'deb' / 'board' / f'{name}.in').read_text()
        return substitute(template, {
            'PX4_PACKAGE': self.package['name'], 'PX4_SERVICE': self.service['name'],
            'PX4_SERVICE_ENABLE': 'yes' if self.service['enable'] else 'no',
            'PX4_DATA_DIR': self.layout['data_dir'], 'PX4_CONFIG_DIR': self.layout['config_dir'],
        })

    def emit(self, output_dir, build_root, version, git_tag):
        """Write everything CMakeLists.txt consumes into output_dir."""
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        self.check_artifacts(build_root)
        (output_dir / 'package_descriptor.cmake').write_text(self.cmake(build_root, version, git_tag))
        (output_dir / 'package.env').write_text(self.environment(version, git_tag))
        with open(output_dir / 'package.json', 'w') as handle:
            json.dump(self.manifest(version, git_tag), handle, indent=2, sort_keys=True)
            handle.write('\n')
        (output_dir / f'{self.service["name"]}.service').write_text(self.unit())
        for name in ('postinst', 'prerm', 'postrm'):
            script = output_dir / name
            script.write_text(self.maintainer_script(name))
            script.chmod(0o755)


# Command line -------------------------------------------------------------

def descriptors(source_dir=PX4_ROOT):
    return [Descriptor(path) for path in descriptor_paths(source_dir)]


def resolve_version(args, source_dir):
    git_tag = args.git_tag or git_describe(source_dir)
    return args.version or debian_version(git_tag), git_tag


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest='command', required=True)

    validate = commands.add_parser('validate', help='check descriptors; exit 1 on the first invalid one')
    validate.add_argument('descriptor', nargs='*', help='<vendor>_<board> or path to package.yaml')
    validate.add_argument('--all', action='store_true', help='every boards/*/*/package.yaml')

    cmake_vars = commands.add_parser('cmake-vars', help='validate and write set(PX4_PACKAGE_*) for the host build')
    cmake_vars.add_argument('descriptor')
    cmake_vars.add_argument('--output', required=True)

    emit = commands.add_parser('emit', help='write install rules, unit, scripts and manifest for the packaging project')
    emit.add_argument('descriptor')
    emit.add_argument('--output-dir', required=True)
    emit.add_argument('--build-root', help='where the component targets were built (default: <source>/build)')
    emit.add_argument('--version', help='Debian version (default: derived from --git-tag)')
    emit.add_argument('--git-tag', help='git describe output (default: run git describe)')

    listing = commands.add_parser('list', help='boards with a descriptor')
    listing.add_argument('--coprocessor-targets', action='store_true', help='print their coprocessor targets instead')

    args = parser.parse_args(argv)

    if args.command == 'list':
        for descriptor in descriptors():
            for line in descriptor.coprocessor_targets if args.coprocessor_targets else [descriptor.board]:
                print(line)
        return 0

    if args.command == 'validate':
        found = descriptors() if args.all else [Descriptor.from_arg(d) for d in args.descriptor]
        if not found:
            print('no package descriptors' if args.all else 'nothing to validate')
            return 0 if args.all else 2
        for descriptor in found:
            descriptor.validate()
            print(f'{descriptor.path.relative_to(descriptor.source_dir)}: OK')
        return 0

    descriptor = Descriptor.from_arg(args.descriptor).validate()
    if args.command == 'cmake-vars':
        lines = [f'set(PX4_PACKAGE_{key} {cmake_quote(value)})' for key, value in descriptor.cmake_variables().items()]
        Path(args.output).write_text('\n'.join(lines) + '\n')
        return 0

    build_root = args.build_root or descriptor.source_dir / 'build'
    version, git_tag = resolve_version(args, descriptor.source_dir)
    descriptor.emit(args.output_dir, build_root, version, git_tag)
    print(f'{descriptor.package["name"]} {version} ({descriptor.package["architecture"]}) -> {args.output_dir}')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (DescriptorError, ValueError) as error:
        sys.exit(f'error: {error}')
