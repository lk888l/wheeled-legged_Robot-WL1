"""Validate actual build artifacts without third-party Python packages.

Usage: python tests/check_firmware_images.py build/Release
"""
import argparse
import math
import struct
import zlib
from pathlib import Path

FLASH = 0x08000000
PARAMS = 0x08060000
RECORD_BYTES = 84


def read_hex(path):
    memory = {}
    base = 0
    ended = False
    for line in path.read_text(encoding="ascii").splitlines():
        assert not ended and line.startswith(":"), f"Invalid HEX line in {path}"
        record = bytes.fromhex(line[1:])
        assert len(record) == record[0] + 5 and sum(record) % 256 == 0, "HEX checksum/length"
        count, offset, kind = record[0], int.from_bytes(record[1:3], "big"), record[3]
        data = record[4:4 + count]
        if kind == 0:
            for index, value in enumerate(data):
                address = base + offset + index
                assert address not in memory, "Overlapping HEX data"
                memory[address] = value
        elif kind == 1:
            assert count == 0
            ended = True
        elif kind == 2:
            assert count == 2
            base = int.from_bytes(data, "big") << 4
        elif kind == 4:
            assert count == 2
            base = int.from_bytes(data, "big") << 16
        else:
            assert kind in (3, 5) and count == 4, "Unknown HEX record"
    assert ended and memory, "Missing EOF or empty HEX"
    return memory


def check(build):
    prefix = build / "WL1_F411CEU6"
    artifact = lambda suffix: Path(str(prefix) + suffix)
    update = read_hex(artifact("_update.hex"))
    factory = read_hex(artifact("_factory.hex"))
    update_bin = artifact("_update.bin").read_bytes()
    factory_bin = artifact("_factory.bin").read_bytes()
    seed = artifact("_defaults.bin").read_bytes()
    assert min(update) == FLASH and max(update) < PARAMS, "Update reaches parameter sector"
    assert len(update_bin) == max(update) - FLASH + 1
    assert all(update_bin[address - FLASH] == value for address, value in update.items())
    assert len(seed) == RECORD_BYTES
    words = struct.unpack("<21I", seed)
    assert words[0] == 0x574C3150 and words[3] == 1, "Factory magic/sequence"
    assert words[1] & 0xFFFF == 3 and words[1] >> 16 <= 1000, "Factory schema/dead zone"
    assert words[2] & ~(1 << 16) == 15, "Factory field count/mode"
    values = struct.unpack('<15f', seed[16:76])
    assert all(math.isfinite(value) for value in values) and 44.5 <= values[13] <= 78.5
    assert words[19] == zlib.crc32(seed[:76]) and words[20] == 0x434F4D54, "Factory CRC/commit"
    assert factory == update | {PARAMS + i: value for i, value in enumerate(seed)}, "Factory content"
    assert len(factory_bin) == PARAMS - FLASH + RECORD_BYTES
    assert factory_bin[:len(update_bin)] == update_bin, "Both images must contain the same application"
    assert all(factory_bin[address - FLASH] == value for address, value in factory.items())
    assert factory_bin[PARAMS - FLASH:] == seed
    assert all(value == 0xFF for value in factory_bin[len(update_bin):PARAMS - FLASH]), "Factory gap fill"
    assert artifact(".bin").read_bytes() == update_bin, "Legacy BIN must retain update semantics"
    assert artifact(".hex").read_bytes() == artifact("_update.hex").read_bytes()
    for mode, last in (("update", 6), ("factory", 7)):
        script = (build / f"flash_{mode}.cfg").read_text()
        assert f"flash erase_sector 0 0 {last}\n" in script
        assert f"WL1_F411CEU6_{mode}.hex" in script
        assert "flash write_image $wl1_image 0 ihex" in script
        assert "verify_image $wl1_image 0 ihex\nreset run" in script
    print(f"PASS: {build}: update={len(update_bin)} B; factory={len(factory_bin)} B; "
          "default CRC, image boundaries, gap fill and erase policies verified")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("build", type=Path)
    check(parser.parse_args().build)
