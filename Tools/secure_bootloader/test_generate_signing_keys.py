#!/usr/bin/env python3

import json
import os
import stat
import tempfile
import unittest

import nacl.signing

import generate_signing_keys


class SigningKeyFileTest(unittest.TestCase):

    def setUp(self):
        self.signing_key = nacl.signing.SigningKey.generate()

    def test_private_key_is_owner_only_and_usable(self):
        with tempfile.TemporaryDirectory() as directory:
            key_name = os.path.join(directory, "signing-key")
            previous_umask = os.umask(0o022)

            try:
                generate_signing_keys.write_key_json(self.signing_key, key_name)
            finally:
                os.umask(previous_umask)

            key_path = key_name + ".json"
            self.assertEqual(stat.S_IMODE(os.stat(key_path).st_mode), 0o600)

            with open(key_path, encoding="utf-8") as key_file:
                keys = json.load(key_file)

            self.assertEqual(keys["private"], self.signing_key._seed.hex())
            self.assertEqual(keys["public"], self.signing_key.verify_key.encode().hex())

    def test_existing_key_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            key_name = os.path.join(directory, "signing-key")
            key_path = key_name + ".json"

            with open(key_path, "w", encoding="utf-8") as key_file:
                key_file.write("existing")

            with self.assertRaises(SystemExit):
                generate_signing_keys.write_key_json(self.signing_key, key_name)

            with open(key_path, encoding="utf-8") as key_file:
                self.assertEqual(key_file.read(), "existing")

    def test_dangling_symlink_is_not_followed(self):
        with tempfile.TemporaryDirectory() as directory:
            key_name = os.path.join(directory, "signing-key")
            key_path = key_name + ".json"
            symlink_target = os.path.join(directory, "attacker-target")
            os.symlink(symlink_target, key_path)

            with self.assertRaises(SystemExit):
                generate_signing_keys.write_key_json(self.signing_key, key_name)

            self.assertFalse(os.path.exists(symlink_target))


if __name__ == "__main__":
    unittest.main()
