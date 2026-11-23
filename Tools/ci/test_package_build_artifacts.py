"""Check what package_build_artifacts.py publishes from a build tree."""

import contextlib
import io
import shutil
import tempfile
import unittest
from pathlib import Path

import yaml

from package_build_artifacts import CONFIG, EVENTS, METADATA, package, target_labels

TARGET_METADATA = ('airframes.xml', *METADATA, EVENTS)
LABELS = {
    'px4_fmu-v6x_default': 'default',
    'px4_fmu-v6x_allyes': 'allyes',
    'px4_fmu-v5_debug': 'debug',
    'accton-godwit_gfh7_debug': 'debug',
}
UNPUBLISHED = {
    'labels': {'allyes': 'compile coverage'},
    'targets': {'px4_fmu-v5_debug': 'NuttX debug build'},
}


class PackageTest(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.build = Path(directory.name) / 'build'
        self.artifacts = Path(directory.name) / 'artifacts'

    def add_target(self, target, *, mavlink=True, extra=()):
        """A build directory holding every file a target can publish."""
        directory = self.build / target
        (directory / 'events').mkdir(parents=True)
        for name in (f'{target}.px4', f'{target}.elf', f'{target}.bin', f'{target}.sbom.spdx.json',
                     'airframes.xml', *METADATA, f'events/{EVENTS}', *extra):
            (directory / name).write_text(name)
        (directory / 'boardconfig').write_text('CONFIG_MODULES_MAVLINK=y\n' if mavlink else '# no mavlink\n')

    def published(self):
        with contextlib.redirect_stdout(io.StringIO()):
            package(self.build, self.artifacts, LABELS, UNPUBLISHED)
        return {str(path.relative_to(self.artifacts)) for path in self.artifacts.rglob('*') if path.is_file()}

    def test_each_kind_of_target_publishes_its_own_set(self):
        # The published set is the contract with S3, the GitHub Release globs
        # and the metadata URIs compiled into firmware.
        self.add_target('px4_fmu-v6x_default', extra=('toc.elf',))
        self.add_target('px4_fmu-v6x_bootloader', mavlink=False)
        self.add_target('ark_can-flow_default', mavlink=False, extra=('ark_can-flow_default.uavcan.bin',))
        self.add_target('px4_io-v2_default', mavlink=False)
        self.add_target('modalai_voxl2-io_default', mavlink=False)
        self.add_target('px4_sitl_default')
        (self.build / 'px4_sitl_default' / 'boardconfig').unlink()

        def with_metadata(target):
            return {f'{target}.px4', f'{target}.elf', f'{target}/{target}.sbom.spdx.json',
                    *(f'{target}/{name}' for name in TARGET_METADATA)}

        self.assertEqual(self.published(), {
            *with_metadata('px4_fmu-v6x_default'),
            *with_metadata('px4_sitl_default'),
            *(f'_general/{name}' for name in TARGET_METADATA),
            'px4_fmu-v6x_bootloader.bin', 'px4_fmu-v6x_bootloader.elf',
            'ark_can-flow_default.uavcan.bin', 'ark_can-flow_default.elf',
            'ark_can-flow_default/ark_can-flow_default.sbom.spdx.json',
            'px4_io-v2_default.bin', 'px4_io-v2_default.elf',
            'px4_io-v2_default/px4_io-v2_default.sbom.spdx.json',
            'modalai_voxl2-io_default.px4', 'modalai_voxl2-io_default.elf',
            'modalai_voxl2-io_default/modalai_voxl2-io_default.sbom.spdx.json',
        })

    def test_unpublished_targets_publish_nothing(self):
        # A label hides every target carrying it; a target entry hides only
        # that target and leaves the label's other targets published.
        for target in LABELS:
            self.add_target(target)
        self.assertEqual({path.split('/')[0].split('.')[0] for path in self.published()},
                         {'px4_fmu-v6x_default', 'accton-godwit_gfh7_debug'})

    def test_incomplete_general_metadata_fails_the_job(self):
        # Flight Review and pyulog read _general/; publishing it without the
        # events or airframes would break them silently.
        for missing in ('airframes.xml', f'events/{EVENTS}'):
            with self.subTest(missing=missing):
                for directory in (self.build, self.artifacts):
                    shutil.rmtree(directory, ignore_errors=True)
                self.add_target('px4_sitl_default')
                (self.build / 'px4_sitl_default' / missing).unlink()
                with self.assertRaises(SystemExit):
                    self.published()


class UnpublishedConfigTest(unittest.TestCase):
    def test_every_entry_matches_a_target_in_the_tree(self):
        # An entry for a label or target that was renamed or removed would
        # silently stop hiding anything, and the list would rot. An entry
        # without a reason hides nothing either.
        unpublished = yaml.safe_load(CONFIG.read_text())['unpublished']
        labels = target_labels()
        self.assertEqual(set(unpublished['labels']) - set(labels.values()), set())
        self.assertEqual(set(unpublished['targets']) - set(labels), set())
        for entry, reason in (*unpublished['labels'].items(), *unpublished['targets'].items()):
            self.assertTrue(reason and isinstance(reason, str), entry)


if __name__ == '__main__':
    unittest.main()
