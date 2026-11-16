import struct
from pathlib import Path
from typing import Dict, List, NamedTuple, Tuple

import pytest

from autopilot_tester import AutopilotTester

# ArduPilot packed parameter format, as served by PX4 at @PARAM/param.pck.
PCK_MAGIC = (0x671B, 0x671C)
PCK_TYPE_INT32 = 3
PCK_TYPE_FLOAT = 4


class PackedParam(NamedTuple):
    type: int
    value_bits: int


def decode_param_pck(data: bytes) -> Tuple[int, int, Dict[str, PackedParam]]:
    """Return (num, total, params) of a param.pck file."""
    assert len(data) >= 6, "param.pck too short"
    magic, num, total = struct.unpack_from("<HHH", data, 0)
    assert magic in PCK_MAGIC, "Unexpected magic 0x{:04X}".format(magic)

    params: Dict[str, PackedParam] = {}
    prev = ""
    i = 6
    while i < len(data):
        if data[i] == 0:
            i += 1  # padding
            continue

        assert i + 2 <= len(data), "Truncated parameter header"
        param_type = data[i] & 0x0F
        has_default = bool((data[i] >> 4) & 0x01)
        common_len = data[i + 1] & 0x0F
        name_len = (data[i + 1] >> 4) + 1
        i += 2

        assert common_len <= len(prev) and common_len + name_len <= 16 \
            and i + name_len + 4 <= len(data), "Malformed parameter at offset {}".format(i)

        name = prev[:common_len] + data[i:i + name_len].decode("ascii")
        i += name_len
        (value_bits,) = struct.unpack_from("<I", data, i)
        i += 4
        if has_default:
            assert i + 4 <= len(data), "Truncated default value"
            i += 4

        params[name] = PackedParam(param_type, value_bits)
        prev = name

    assert len(params) == num, "Decoded {} parameters, header says {}".format(len(params), num)
    return num, total, params


def encode_one_float(name: str, value: float) -> bytes:
    body = bytes([PCK_TYPE_FLOAT, (len(name) - 1) << 4]) + name.encode("ascii") \
        + struct.pack("<f", value)
    return struct.pack("<HHH", PCK_MAGIC[0], 1, 6 + len(body)) + body


def int_bits(value: int) -> int:
    return int(struct.unpack("<I", struct.pack("<i", value))[0])


def float_bits(value: float) -> int:
    return int(struct.unpack("<I", struct.pack("<f", value))[0])


def compare_downloads(tester: AutopilotTester, local_dir: Path) -> List[str]:
    """Download all parameters with PARAM_VALUE and with FTP, and return the differences."""
    # The reference: the conventional PARAM_VALUE stream.
    all_params = tester.get_all_params()
    assert len(all_params.int_params) + len(all_params.float_params) > 100

    local_dir.mkdir()
    tester.ftp_download("@PARAM/param.pck?withdefaults=1", str(local_dir))
    data = (local_dir / "param.pck?withdefaults=1").read_bytes()

    num, total, packed = decode_param_pck(data)
    assert num == total

    expected = {p.name: PackedParam(PCK_TYPE_INT32, int_bits(p.value))
                for p in all_params.int_params if p.name != "_HASH_CHECK"}
    expected.update({p.name: PackedParam(PCK_TYPE_FLOAT, float_bits(p.value))
                     for p in all_params.float_params})

    differences = ["{}: PARAM_VALUE {}, FTP {}".format(name, value, packed.get(name))
                   for name, value in expected.items() if packed.get(name) != value]
    differences += ["{}: only in FTP".format(name) for name in packed if name not in expected]
    print("Compared {} parameters, {} bytes packed".format(len(expected), len(data)))
    return differences


@pytest.mark.multicopter
def test_parameters_download_via_ftp(tester: AutopilotTester, tmp_path: Path) -> None:
    # The baro GNSS auto-calibration writes CAL_BAROx_OFF shortly after boot, which can land
    # between the two downloads compared below.
    tester.set_param_int("SENS_BAR_AUTOCAL", 0)

    differences = compare_downloads(tester, tmp_path / "param_ftp")
    assert not differences, "{} parameters differ: {}".format(len(differences), differences)


@pytest.mark.multicopter
def test_parameters_upload_via_ftp(tester: AutopilotTester, tmp_path: Path) -> None:
    name = "MPC_XY_P"
    original = tester.get_param_float(name)
    updated = original + 1.0

    local_file = tmp_path / "param.pck"
    local_file.write_bytes(encode_one_float(name, updated))

    tester.ftp_upload(str(local_file), "@PARAM")

    assert tester.get_param_float(name) == pytest.approx(updated)

    tester.set_param_float(name, original)
