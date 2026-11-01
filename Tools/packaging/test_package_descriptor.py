"""Check package_descriptor.py and the packaging project it feeds.

The install tree, the hooks and the unit are the contract a board package
makes with the target system, and a descriptor mistake surfaces as a broken
package on a vehicle rather than as a build error. Each test pins one rule
of that contract against a synthetic board tree, and the last one builds a
real .deb from it where CPack has the DEB generator.
"""

import io
import os
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import package_descriptor as pd  # noqa: E402

HOST_BOARD = 'CONFIG_PLATFORM_POSIX=y\nCONFIG_BOARD_LINUX_TARGET=y\n' \
             'CONFIG_BOARD_TOOLCHAIN="aarch64-linux-gnu"\nCONFIG_BOARD_ROOT_PATH="/data/px4"\n'
DSP_BOARD = 'CONFIG_PLATFORM_QURT=y\nCONFIG_BOARD_TOOLCHAIN="qurt"\n'

# Shaped like the VOXL2 package that ships today: PX4 on both cores, the DSP
# image dropped where the DSP loader expects it, vendor data directory.
TWO_CORES = {
    'package': {
        'name': 'voxl-px4', 'description': 'PX4 Autopilot for ACME Widget',
        'maintainer': 'ACME <support@acme.example>',
        'formats': {'deb': {'depends': ['voxl-reset-slpi (>= 0.0.3)', 'modalai-slpi (>= 1.2.3) | modalai-adsp (>= 1.0.7)'],
                            'conflicts': ['px4-rb5-flight'], 'replaces': ['px4-rb5-flight']}},
    },
    'service': {'exec_start': '/usr/bin/voxl-px4', 'after': ['sscrpcd.service'], 'requires': ['sscrpcd.service']},
    'components': [
        {'name': 'apps', 'role': 'host', 'target': 'acme_widget_default'},
        {'name': 'slpi', 'role': 'coprocessor', 'target': 'acme_widget_dsp',
         'artifact': 'platforms/qurt/libpx4.so', 'install': '/usr/lib/rfsa/adsp/libpx4.so', 'apply': 'none'},
    ],
    'files': [{'source': 'target/voxl-px4', 'destination': '/usr/bin/'},
              {'source': 'target/px4.config', 'destination': '/etc/modalai/px4.config'}],
}

# Shaped like a board that runs nothing of PX4 on Linux: one RTOS image for a
# remoteproc core, no host binary.
HOST_LESS = {
    'package': {'description': 'PX4 Autopilot for ACME Widget M7', 'maintainer': 'ACME <support@acme.example>'},
    'components': [
        {'name': 'm7', 'role': 'coprocessor', 'target': 'acme_widget_dsp', 'artifact': 'platforms/qurt/libpx4.so',
         'install': '/lib/firmware/px4/acme-widget-m7.bin', 'apply': 'remoteproc', 'remoteproc': 'imx95-cm7'},
    ],
}


def cpack_has_deb():
    cpack = shutil.which('cpack')
    return bool(cpack) and 'DEB ' in subprocess.run([cpack, '--help'], capture_output=True, text=True).stdout


def read_deb(path):
    """{'control': bytes, 'data': bytes} tar payloads of a .deb (an ar archive).

    CPack picks the compression per member (control.tar.gz, data.tar.xz), so
    members are keyed by their stem.
    """
    data = Path(path).read_bytes()
    assert data[:8] == b'!<arch>\n', 'not an ar archive'
    members, pos = {}, 8
    while pos + 60 <= len(data):
        header = data[pos:pos + 60]
        name = header[:16].decode().strip().rstrip('/')
        size = int(header[48:58])
        members[name.split('.')[0]] = data[pos + 60:pos + 60 + size]
        pos += 60 + size + size % 2
    return members


def tar_members(payload):
    """{path: TarInfo} of a tar payload, paths without the leading ./"""
    with tarfile.open(fileobj=io.BytesIO(payload), mode='r:*') as archive:
        return {m.name.lstrip('./'): m for m in archive.getmembers() if m.name not in ('.', './')}


def tar_file(payload, name):
    with tarfile.open(fileobj=io.BytesIO(payload), mode='r:*') as archive:
        member = next(m for m in archive.getmembers() if m.name.lstrip('./') == name)
        return archive.extractfile(member).read().decode()


class BoardTree:
    """A PX4-shaped tree with one board and its fake build outputs."""

    def __init__(self, descriptor, hooks=None):
        self.root = Path(tempfile.mkdtemp(prefix='px4-package-'))
        self.board_dir = self.root / 'boards' / 'acme' / 'widget'
        self.board_dir.mkdir(parents=True)
        (self.board_dir / 'default.px4board').write_text(HOST_BOARD)
        (self.board_dir / 'dsp.px4board').write_text(DSP_BOARD)
        self.write(self.board_dir / 'target' / 'voxl-px4', '#!/bin/sh\nexec /usr/bin/px4 -d\n', executable=True)
        self.write(self.board_dir / 'target' / 'px4.config', 'uorb start\n')
        for point, scripts in (hooks or {}).items():
            for name, body in scripts.items():
                self.write(self.board_dir / 'hooks' / f'{point}.d' / name, body, executable=True)
        self.descriptor_path = self.board_dir / 'package.yaml'
        self.descriptor_path.write_text(yaml.safe_dump(descriptor))

        host = self.root / 'build' / 'acme_widget_default'
        self.write(host / 'bin' / 'px4', '#!/bin/sh\necho px4\n', executable=True)
        self.write(host / 'bin' / 'px4-alias.sh', "alias commander='px4-commander --instance $px4_instance'\n")
        os.symlink('px4', host / 'bin' / 'px4-commander')
        self.write(host / 'etc' / 'init.d-posix' / 'rcS', '# rcS\n')
        self.write(host / 'parameters.json.xz', 'params')
        self.write(self.root / 'build' / 'acme_widget_dsp' / 'platforms' / 'qurt' / 'libpx4.so', 'dsp image')

    @staticmethod
    def write(path, content, executable=False):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
        if executable:
            path.chmod(0o755)

    def descriptor(self):
        return pd.Descriptor(self.descriptor_path).validate()

    def problems(self):
        """Validation messages, joined; '' when the descriptor is valid."""
        try:
            self.descriptor()
        except pd.DescriptorError as error:
            return str(error)
        return ''

    def cleanup(self):
        shutil.rmtree(self.root)


def with_tree(descriptor, **kwargs):
    """Decorator: run the test with a fresh BoardTree, cleaned up afterwards."""
    def wrap(test):
        def run(self):
            tree = BoardTree(descriptor, **kwargs)
            try:
                test(self, tree)
            finally:
                tree.cleanup()
        return run
    return wrap


class DebianVersionTest(unittest.TestCase):
    def test_git_describe_shapes(self):
        # the shapes git describe produces across a release cycle; the order
        # matters because apt picks the highest version
        for tag, version in (('v1.17.0', '1.17.0'), ('v1.17.0-beta1', '1.17.0~beta1'),
                             ('v1.17.0-beta1-42-gabcdef', '1.17.0~beta1.42.gabcdef'),
                             ('v1.17.0-42-gabcdef', '1.17.0.42.gabcdef'),
                             ('v1.17.0-acme-rc1-3-g0123ab', '1.17.0~acme~rc1.3.g0123ab'),
                             ('v0.0.0', '0.0.0')):
            self.assertEqual(pd.debian_version(tag), version, tag)

    def test_rejects_tags_it_cannot_order(self):
        # a silently mangled version would install as an unrelated package
        with self.assertRaises(ValueError):
            pd.debian_version('release-2026')


class DescriptorRulesTest(unittest.TestCase):
    @with_tree(TWO_CORES)
    def test_two_core_descriptor_derives_what_voxl2_hardcodes(self, tree):
        descriptor = tree.descriptor()
        self.assertEqual(descriptor.package['architecture'], 'arm64')
        self.assertEqual(descriptor.layout['data_dir'], '/data/px4')
        self.assertEqual(descriptor.service['name'], 'voxl-px4')
        installs = {(rule.destination, rule.rename) for rule in descriptor.install_rules(tree.root / 'build')}
        self.assertIn(('/usr/bin', None), installs)
        self.assertIn(('/usr/lib/rfsa/adsp', 'libpx4.so'), installs)
        self.assertIn(('/etc/modalai', 'px4.config'), installs)

    @with_tree(HOST_LESS)
    def test_host_less_package_is_architecture_all_and_oneshot(self, tree):
        # nothing in the package runs on Linux, so it must build on any CI host
        # and its unit must still apply the firmware without an ExecStart
        descriptor = tree.descriptor()
        self.assertEqual(descriptor.package['name'], 'px4-acme-widget')
        self.assertEqual(descriptor.package['architecture'], 'all')
        self.assertEqual(descriptor.layout['data_dir'], '/var/lib/px4')
        unit = descriptor.unit()
        self.assertIn('Type=oneshot', unit)
        self.assertNotIn('ExecStart=', unit)
        self.assertIn('ExecStartPre=+/usr/lib/px4/run-hooks apply-firmware', unit)

    @with_tree(TWO_CORES)
    def test_unit_wraps_the_vendor_exec_start_with_hooks(self, tree):
        unit = tree.descriptor().unit()
        self.assertIn('After=sscrpcd.service', unit)
        self.assertIn('ExecStart=/usr/bin/voxl-px4', unit)
        self.assertIn('ExecStopPost=+/usr/lib/px4/run-hooks post-stop', unit)
        self.assertIn('Restart=on-failure', unit)

    @with_tree(dict(TWO_CORES, components=TWO_CORES['components'] + [dict(TWO_CORES['components'][1], name='slpi2', apply='mcuboot')]))
    def test_rejects_apply_methods_without_an_implementation(self, tree):
        # mcuboot is reserved; accepting it would fail on the vehicle, not here
        self.assertIn('apply', tree.problems())

    @with_tree(dict(TWO_CORES, components=TWO_CORES['components'] + [dict(TWO_CORES['components'][0], name='apps2')]))
    def test_rejects_a_second_host_component(self, tree):
        self.assertIn('more than one host component', tree.problems())

    @with_tree(dict(HOST_LESS, components=[dict(HOST_LESS['components'][0], install='/opt/firmware/m7.bin')]))
    def test_remoteproc_image_must_live_in_the_firmware_dir(self, tree):
        # the kernel only loads firmware from its search path
        self.assertIn('/lib/firmware/px4', tree.problems())

    @with_tree(dict(TWO_CORES, layout={'data_dir': '/var/lib/px4'}))
    def test_data_dir_must_match_the_board_root_path(self, tree):
        # PX4 writes where CONFIG_BOARD_ROOT_PATH says; a different data_dir
        # would be created, owned and backed up for nothing
        self.assertIn('CONFIG_BOARD_ROOT_PATH', tree.problems())

    @with_tree(dict(HOST_LESS, service={'exec_start': '/usr/bin/px4'}))
    def test_exec_start_needs_a_host_component(self, tree):
        self.assertIn('exec_start', tree.problems())

    @with_tree(dict(TWO_CORES, service={}))
    def test_host_component_needs_exec_start(self, tree):
        # there is no generic way to start PX4 on Linux hardware; the vendor says how
        self.assertIn('exec_start', tree.problems())

    @with_tree(TWO_CORES)
    def test_rejects_legacy_cmake_overrides_next_to_a_descriptor(self, tree):
        tree.write(tree.board_dir / 'cmake' / 'package.cmake', '# legacy\n')
        self.assertIn('cmake/package.cmake', tree.problems())

    @with_tree(TWO_CORES, hooks={'post-install': {'10-dirs': '#!/bin/bash\nmkdir -p /data/px4/slpi\n'}})
    def test_hooks_are_posix_sh(self, tree):
        # Yocto images and minimal rootfs ship dash or busybox, not bash
        self.assertIn('#!/bin/sh', tree.problems())

    @with_tree(TWO_CORES, hooks={'post-install': {'10-dirs.sh': '#!/bin/sh\n'}})
    def test_hook_names_have_no_extension(self, tree):
        # run-hooks skips names with a dot so editor backups never run
        self.assertIn('no extension', tree.problems())

    @with_tree(HOST_LESS)
    def test_missing_coprocessor_image_fails_the_package(self, tree):
        # the CI-green trap: a package without its firmware must not build
        (tree.root / 'build' / 'acme_widget_dsp' / 'platforms' / 'qurt' / 'libpx4.so').unlink()
        with self.assertRaises(pd.DescriptorError):
            tree.descriptor().check_artifacts(tree.root / 'build')

    @with_tree(TWO_CORES)
    def test_environment_is_sourceable_by_sh(self, tree):
        # run-hooks and every vendor hook read these facts through the shell
        env = tree.descriptor().environment('1.17.0~beta1', 'v1.17.0-beta1')
        out = subprocess.run(['sh', '-c', f'{env}\necho "$PX4_DATA_DIR $PX4_COPROCESSORS $PX4_COPROCESSOR_slpi_FIRMWARE"'],
                             capture_output=True, text=True, check=True).stdout.split()
        self.assertEqual(out, ['/data/px4', 'slpi', '/usr/lib/rfsa/adsp/libpx4.so'])

    @with_tree(HOST_LESS, hooks={'post-install': {'10-dirs': '#!/bin/sh\nexit 0\n'}})
    def test_manifest_lists_components_and_hooks(self, tree):
        manifest = tree.descriptor().manifest('1.17.0', 'v1.17.0')
        self.assertEqual(manifest['package']['architecture'], 'all')
        self.assertEqual(manifest['components'][0]['remoteproc'], 'imx95-cm7')
        self.assertEqual(manifest['hooks'], {'post-install': ['10-dirs']})

    @unittest.skipUnless(shutil.which('shellcheck'), 'shellcheck not installed')
    @with_tree(TWO_CORES)
    def test_generated_maintainer_scripts_pass_shellcheck(self, tree):
        # the templates are checked as files; substitution must not break quoting
        out = tree.root / 'out'
        tree.descriptor().emit(out, tree.root / 'build', '1.17.0', 'v1.17.0')
        subprocess.run(['shellcheck', '--shell=dash', out / 'postinst', out / 'prerm', out / 'postrm'], check=True)


@unittest.skipUnless(cpack_has_deb(), 'CPack has no DEB generator here')
class DebBuildTest(unittest.TestCase):
    @with_tree(TWO_CORES, hooks={'post-install': {'10-dirs': '#!/bin/sh\nmkdir -p "$PX4_DATA_DIR/slpi"\n'}})
    def test_deb_holds_the_contract_tree(self, tree):
        package_dir = tree.root / 'build' / 'acme_widget_package'
        subprocess.run(['cmake', '-S', pd.PACKAGING_DIR, '-B', package_dir, f'-DPX4_SOURCE_DIR={tree.root}',
                        '-DPX4_PACKAGE_BOARD=acme_widget', '-DPX4_PACKAGE_GIT_TAG=v1.17.0-beta1-3-gabcdef',
                        f'-DPython3_EXECUTABLE={sys.executable}'], check=True, capture_output=True, text=True)
        subprocess.run(['cpack', '-G', 'DEB'], cwd=package_dir, check=True, capture_output=True, text=True)
        deb = package_dir / 'voxl-px4_1.17.0~beta1.3.gabcdef_arm64.deb'
        self.assertTrue(deb.exists(), sorted(p.name for p in package_dir.iterdir()))

        members = read_deb(deb)
        control = tar_members(members['control'])
        fields = dict(line.split(': ', 1) for line in tar_file(members['control'], 'control').splitlines()
                      if ': ' in line)
        self.assertEqual(fields['Package'], 'voxl-px4')
        self.assertEqual(fields['Version'], '1.17.0~beta1.3.gabcdef')
        self.assertEqual(fields['Architecture'], 'arm64')
        self.assertEqual(fields['Depends'], 'voxl-reset-slpi (>= 0.0.3), modalai-slpi (>= 1.2.3) | modalai-adsp (>= 1.0.7)')
        self.assertEqual(fields['Conflicts'], 'px4-rb5-flight')
        for script in ('postinst', 'prerm', 'postrm'):
            self.assertTrue(control[script].mode & 0o111, f'{script} not executable')

        data = tar_members(members['data'])
        expected = {
            'usr/bin/px4', 'usr/bin/px4-alias.sh', 'usr/bin/px4-commander', 'usr/bin/voxl-px4',
            'usr/share/px4/etc/init.d-posix/rcS', 'usr/share/px4/etc/extras/parameters.json.xz',
            'usr/share/px4/package.json', 'usr/lib/rfsa/adsp/libpx4.so', 'etc/modalai/px4.config',
            'usr/lib/px4/run-hooks', 'usr/lib/px4/apply/none', 'usr/lib/px4/apply/remoteproc',
            'usr/lib/px4/package.env', 'usr/lib/px4/hooks/post-install.d/10-dirs',
            'lib/systemd/system/voxl-px4.service',
        }
        files = {name for name, member in data.items() if not member.isdir()}
        self.assertEqual(files, expected)
        self.assertTrue(data['usr/bin/px4-commander'].issym())
        self.assertEqual(data['usr/bin/px4-commander'].linkname, 'px4')
        for program in ('usr/bin/px4', 'usr/bin/voxl-px4', 'usr/lib/px4/run-hooks', 'usr/lib/px4/hooks/post-install.d/10-dirs'):
            self.assertTrue(data[program].mode & 0o111, f'{program} not executable')


if __name__ == '__main__':
    unittest.main()
