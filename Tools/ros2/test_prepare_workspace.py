from contextlib import ExitStack, redirect_stdout
import io
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import yaml

import prepare_workspace as tool


class PrepareWorkspaceTest(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        directory = self.stack.enter_context(tempfile.TemporaryDirectory())
        self.root = Path(directory)
        self.repo = self.root / "px4"
        self.manifest_path = self.repo / "Tools/ros2/ros2.repos"
        self.manifest_path.parent.mkdir(parents=True)
        self.manifest = {"repositories": {
            "px4_msgs": {
                "type": "git", "url": "https://example.invalid/messages.git",
                "version": "a" * 40,
            },
            "px4-ros2-interface-lib": {
                "type": "git", "url": "https://example.invalid/interface.git",
                "version": "b" * 40,
            },
        }}
        self.save_manifest()
        self.workspace = self.root / "workspace"
        self.run = self.stack.enter_context(
            patch.object(tool.subprocess, "run")
        )
        self.run.return_value = subprocess.CompletedProcess(
            [], 0, stdout="c" * 40, stderr=""
        )
        self.git = self.stack.enter_context(
            patch.object(tool.subprocess, "check_output")
        )
        self.git.side_effect = self.requested_revision
        self.which = self.stack.enter_context(
            patch.object(tool.shutil, "which", return_value="/usr/bin/vcs")
        )
        self.stack.enter_context(redirect_stdout(io.StringIO()))

    def save_manifest(self):
        self.manifest_path.write_text(yaml.safe_dump(self.manifest))

    def requested_revision(self, command, **kwargs):
        resolved = yaml.safe_load((self.workspace / "ros2.repos").read_text())
        repository = resolved["repositories"][Path(command[2]).name]
        return repository["version"] + "\n"

    def test_imports_pins_and_refreshes_messages(self):
        tool.prepare_workspace(self.workspace, "", self.repo)
        self.assertEqual(
            yaml.safe_load((self.workspace / "ros2.repos").read_text()),
            self.manifest,
        )
        self.assertEqual(self.git.call_count, 2)
        self.assertEqual(self.run.call_args_list[0].args[0], [
            "vcs", "import", str(self.workspace / "src"),
        ])
        self.assertEqual(self.run.call_args_list[1].args[0], [
            "bash",
            str(self.repo / "Tools/packaging/containers/prepare_context.sh"),
            "--messages-only", str(self.workspace / "src/px4_msgs"),
        ])
        self.assertFalse((self.workspace / "ros2-import.repos").exists())

    def test_empty_directory_and_uppercase_override(self):
        self.workspace.mkdir()
        tool.prepare_workspace(self.workspace, "D" * 40, self.repo)
        resolved = yaml.safe_load((self.workspace / "ros2.repos").read_text())
        self.assertEqual(
            resolved["repositories"]["px4-ros2-interface-lib"]["version"],
            "d" * 40,
        )
        self.assertEqual(
            yaml.safe_load(self.manifest_path.read_text()), self.manifest
        )

    def test_rejects_unsafe_destinations(self):
        occupied = self.root / "occupied"
        occupied.mkdir()
        marker = occupied / "keep"
        marker.write_text("untouched")
        symlink = self.root / "symlink"
        symlink.symlink_to(occupied, target_is_directory=True)
        dangling = self.root / "dangling"
        dangling.symlink_to(self.root / "absent", target_is_directory=True)
        for destination in (occupied, marker, symlink, dangling):
            with self.subTest(destination=destination):
                with self.assertRaisesRegex(ValueError, "new or empty"):
                    tool.prepare_workspace(destination, "", self.repo)
        self.assertEqual(marker.read_text(), "untouched")
        self.run.assert_not_called()

    def test_rejects_invalid_override_before_creating_workspace(self):
        for version in ("main", "1234", "z" * 40):
            with self.subTest(version=version):
                with self.assertRaisesRegex(ValueError, "40-hex"):
                    tool.prepare_workspace(self.workspace, version, self.repo)
        self.assertFalse(self.workspace.exists())

    def test_rejects_mutable_or_non_git_manifest(self):
        for change in ({"version": "main"}, {"version": 123}, {"type": "svn"}):
            with self.subTest(change=change):
                repository = self.manifest["repositories"]["px4_msgs"]
                original = repository.copy()
                repository.update(change)
                self.save_manifest()
                with self.assertRaisesRegex(ValueError, "immutable git"):
                    tool.prepare_workspace(self.workspace, "", self.repo)
                repository.update(original)
        self.assertFalse(self.workspace.exists())

    def test_rejects_mismatched_checkout(self):
        self.git.side_effect = None
        self.git.return_value = "0" * 40
        with self.assertRaisesRegex(ValueError, "expected.*got"):
            tool.prepare_workspace(self.workspace, "", self.repo)
        self.assertEqual(self.run.call_count, 1)

    def test_stops_on_import_failure(self):
        self.run.side_effect = subprocess.CalledProcessError(
            1, ["vcs", "import"]
        )
        with self.assertRaises(subprocess.CalledProcessError):
            tool.prepare_workspace(self.workspace, "", self.repo)
        self.git.assert_not_called()

    def test_missing_vcs_does_not_create_workspace(self):
        self.which.return_value = None
        with self.assertRaisesRegex(FileNotFoundError, "vcstool"):
            tool.prepare_workspace(self.workspace, "", self.repo)
        self.assertFalse(self.workspace.exists())

    def test_cli_rejects_empty_workspace(self):
        with patch.object(tool.sys, "argv", ["prepare_workspace.py", ""]):
            with patch.object(tool.sys, "stderr", io.StringIO()):
                with self.assertRaises(SystemExit) as result:
                    tool.main()
        self.assertEqual(result.exception.code, 2)
        self.run.assert_not_called()


if __name__ == "__main__":
    unittest.main()
