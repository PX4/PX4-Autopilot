#!/usr/bin/env python3
"""Cross-compile both dialects and exchange real MAVLink frames between them."""

import argparse
import hashlib
import importlib.util
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
SETTINGS = None
HARNESS = r"""
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <mavlink.h>

static void expected(mavlink_amov_rid_identity_request_t *r,
                     mavlink_amov_rid_identity_t *i, mavlink_amov_uom_status_t *s)
{
    memset(r, 0, sizeof(*r)); memset(i, 0, sizeof(*i)); memset(s, 0, sizeof(*s));
    r->session_id = UINT64_C(0xfedcba9876543210); r->request_id = 0x89abcdef;
    r->target_system = 42; r->target_component = 1; r->protocol_version = 1; r->flags = 1;
    i->boot_id = UINT64_C(0xabcdef0123456789); i->session_id = r->session_id;
    i->request_id = r->request_id; i->identity_revision = 0xaabbccdd;
    i->rid_boot_id = 0xffeeddcc; i->pending_revision = 0xaabbccde;
    i->target_system = 255; i->target_component = 190; i->protocol_version = 1;
    i->result = AMOV_RID_IDENTITY_RESULT_OK; i->activation_state = AMOV_RID_ACTIVATION_STATE_ACTIVE;
    i->rid_node_id = 125; i->aircraft_class = 4; i->activation_error = 7;
    memcpy(i->rid_uid, "AABBCCDDEEFF", sizeof(i->rid_uid));
    memcpy(i->uas_id, "AMOV0123456789012345", sizeof(i->uas_id));
    memcpy(i->uin, "12345678", sizeof(i->uin));
    for (unsigned n = 0; n < sizeof(i->registration_id); ++n) i->registration_id[n] = 'A' + n % 26;
    for (unsigned n = 0; n < sizeof(i->identity_hash); ++n) i->identity_hash[n] = 0x80 + n;
    memset(i->error, 'E', sizeof(i->error));
    s->boot_id = i->boot_id; s->session_id = i->session_id;
    s->sequence = UINT32_MAX; s->identity_revision = i->identity_revision; s->rid_boot_id = i->rid_boot_id;
    s->last_ack_age_ms = UINT32_MAX; s->error_code = AMOV_UOM_ERROR_TIMEOUT;
    s->target_system = 42; s->target_component = 1; s->protocol_version = 1;
    s->environment = AMOV_UOM_ENVIRONMENT_MOCK; s->state = AMOV_UOM_STATE_DEGRADED;
    s->flags = AMOV_UOM_FLAGS_IDENTITY_VALID | AMOV_UOM_FLAGS_ACTIVATION_VALID |
               AMOV_UOM_FLAGS_TELEMETRY_VALID | AMOV_UOM_FLAGS_AUTHENTICATED;
    memcpy(s->identity_hash, i->identity_hash, sizeof(s->identity_hash));
}

static int send_message(mavlink_message_t *message)
{
    uint8_t wire[MAVLINK_MAX_PACKET_LEN];
    unsigned length = mavlink_msg_to_send_buffer(wire, message);
    return fwrite(wire, 1, length, stdout) == length ? 0 : 1;
}

int main(int argc, char **argv)
{
    (void)argv;
    mavlink_amov_rid_identity_request_t request;
    mavlink_amov_rid_identity_t identity;
    mavlink_amov_uom_status_t status;
    expected(&request, &identity, &status);
    mavlink_message_t message;
    if (argc == 1) {
        mavlink_msg_amov_rid_identity_request_encode(255, 190, &message, &request);
        if (send_message(&message)) return 1;
        mavlink_msg_amov_rid_identity_encode(42, 1, &message, &identity);
        if (send_message(&message)) return 1;
        mavlink_msg_amov_uom_status_encode(255, 190, &message, &status);
        return send_message(&message);
    }
    int count = 0, byte;
    mavlink_status_t parse_status;
    memset(&parse_status, 0, sizeof(parse_status));
    while ((byte = getchar()) != EOF) {
        if (!mavlink_parse_char(MAVLINK_COMM_0, (uint8_t)byte, &message, &parse_status)) continue;
        if (argc == 3) { ++count; continue; }
        if (count == 0 && message.msgid == MAVLINK_MSG_ID_AMOV_RID_IDENTITY_REQUEST) {
            mavlink_amov_rid_identity_request_t value;
            mavlink_msg_amov_rid_identity_request_decode(&message, &value);
            if (memcmp(&value, &request, MAVLINK_MSG_ID_AMOV_RID_IDENTITY_REQUEST_LEN)) return 2;
        } else if (count == 1 && message.msgid == MAVLINK_MSG_ID_AMOV_RID_IDENTITY) {
            mavlink_amov_rid_identity_t value;
            mavlink_msg_amov_rid_identity_decode(&message, &value);
            if (memcmp(&value, &identity, MAVLINK_MSG_ID_AMOV_RID_IDENTITY_LEN)) return 3;
        } else if (count == 2 && message.msgid == MAVLINK_MSG_ID_AMOV_UOM_STATUS) {
            mavlink_amov_uom_status_t value;
            mavlink_msg_amov_uom_status_decode(&message, &value);
            if (memcmp(&value, &status, MAVLINK_MSG_ID_AMOV_UOM_STATUS_LEN)) return 4;
        } else return 5;
        ++count;
    }
    if (argc == 3) { printf("%d\n", count); return 0; }
    return count == 3 ? 0 : 6;
}
"""


def digest_tree(directory):
    return {p.name: hashlib.sha256(p.read_bytes()).digest() for p in directory.glob("*.xml")}


class ProtocolTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="amov-protocol-")
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.build = Path(cls.temporary.name)
        cls.executables = []
        source = cls.build / "exchange.c"
        source.write_text(HARNESS, encoding="utf-8")
        for label, definitions, dialect, generator_root in (
            ("px4", SETTINGS.px4_definitions, "common", SETTINGS.pymavlink_root),
            (
                "qgc",
                SETTINGS.qgc_definitions,
                "all",
                SETTINGS.qgc_pymavlink_root or SETTINGS.pymavlink_root,
            ),
        ):
            before = digest_tree(definitions)
            output = cls.build / label
            command = [
                sys.executable,
                str(HERE / "generate.py"),
                "--definitions",
                str(definitions),
                "--dialect",
                dialect,
                "--output",
                str(output),
                "--work-dir",
                str(cls.build / (label + "-xml")),
            ]
            if generator_root:
                command += ["--pymavlink-root", str(generator_root)]
            subprocess.run(command, check=True, stdout=subprocess.DEVNULL)
            if before != digest_tree(definitions):
                raise AssertionError("generator changed upstream definitions")
            executable = cls.build / (label + "-exchange")
            subprocess.run(
                [
                    SETTINGS.cc,
                    "-std=c11",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-Wno-address-of-packed-member",
                    "-I" + str(output / dialect),
                    str(source),
                    "-o",
                    str(executable),
                ],
                check=True,
            )
            cls.executables.append(executable)

    def test_bidirectional_frames(self):
        frames = []
        for encoder in self.executables:
            wire = subprocess.check_output([str(encoder)])
            frames.append(wire)
            for decoder in self.executables:
                subprocess.run([str(decoder), "decode"], input=wire, check=True)
        self.assertEqual(frames[0], frames[1])

    def test_corrupt_frame_rejected(self):
        wire = bytearray(subprocess.check_output([str(self.executables[0])]))
        wire[12] ^= 0x40
        for decoder in self.executables:
            count = subprocess.check_output([str(decoder), "decode", "count"], input=wire)
            self.assertEqual(count.strip(), b"2")

    def test_wire_contract(self):
        for label, dialect in (("px4", "common"), ("qgc", "all")):
            for name, length, crc in (
                ("AMOV_RID_IDENTITY_REQUEST", 16, 26),
                ("AMOV_RID_IDENTITY", 224, 250),
                ("AMOV_UOM_STATUS", 72, 206),
            ):
                header = (
                    self.build / label / dialect / ("mavlink_msg_" + name.lower() + ".h")
                ).read_text()
                self.assertIn("#define MAVLINK_MSG_ID_" + name + "_LEN " + str(length), header)
                self.assertIn("#define MAVLINK_MSG_ID_" + name + "_CRC " + str(crc), header)

    def test_repository_mirrors(self):
        for mirror in SETTINGS.mirror:
            for name in (
                "amov.xml",
                "generate.py",
                "test_protocol.py",
                "README.md",
                "INTEGRATION.md",
            ):
                self.assertEqual(
                    (HERE / name).read_bytes(), (mirror / name).read_bytes(), str(mirror / name)
                )

    def test_collision_rejected(self):
        spec = importlib.util.spec_from_file_location("amov_generate", HERE / "generate.py")
        generator = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(generator)
        upstream = self.build / "conflict"
        upstream.mkdir(exist_ok=True)
        shutil.copyfile(HERE / "amov.xml", upstream / "common.xml")
        with self.assertRaisesRegex(ValueError, "collision"):
            generator.overlay(
                upstream, "common", HERE / "amov.xml", self.build / "collision-output"
            )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4-definitions", required=True, type=Path)
    parser.add_argument("--qgc-definitions", required=True, type=Path)
    parser.add_argument(
        "--pymavlink-root", type=Path, help="PX4 pinned generator (also the QGC fallback)"
    )
    parser.add_argument(
        "--qgc-pymavlink-root", type=Path, help="QGC pinned generator when different from PX4"
    )
    parser.add_argument("--mirror", action="append", type=Path, default=[])
    parser.add_argument("--cc", default="cc")
    SETTINGS, rest = parser.parse_known_args()
    unittest.main(argv=[sys.argv[0], *rest])
