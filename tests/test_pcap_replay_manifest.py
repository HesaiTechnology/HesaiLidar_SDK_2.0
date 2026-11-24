import copy
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from pcap_replay_manifest import inventory, manifest, comparable


def capture(endian="<", nano=False, reverse=False):
    magic = 0xa1b23c4d if nano else 0xa1b2c3d4
    header = struct.pack(endian+"IHHiiII", magic, 2, 4, 0, 0, 65535, 1)
    times = [(10, 1), (9 if reverse else 11, 2)]
    return header + b"".join(struct.pack(endian+"IIII", sec, frac, 4, 4)+b"test" for sec, frac in times)


class ManifestTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.pcap = self.root / "test.pcap"
        self.pcap.write_bytes(capture())
        self.cal = self.root / "correction.csv"
        self.cal.write_text("synthetic calibration")
        self.config = self.root / "config.ini"
        self.config.write_text("synthetic configuration")

    def build(self):
        return manifest(self.pcap, self.cal, self.config, "synthetic_revision", "sensor", 1.0)

    def test_manifest(self):
        report = self.build()
        self.assertEqual(report["capture"]["records"], 2)
        self.assertEqual(len(report["inputs"]["pcap"]["sha256"]), 64)

    def test_all_classic_formats(self):
        for endian in ("<", ">"):
            for nano in (True, False):
                self.pcap.write_bytes(capture(endian, nano))
                self.assertEqual(inventory(self.pcap)["ticks_per_second"], 1000000000 if nano else 1000000)

    def test_truncated_payload(self):
        self.pcap.write_bytes(capture()[:-1])
        with self.assertRaises(ValueError):
            inventory(self.pcap)

    def test_truncated_record_header(self):
        self.pcap.write_bytes(capture()+b"xx")
        with self.assertRaises(ValueError):
            inventory(self.pcap)

    def test_capture_clock_reversal_is_reported(self):
        self.pcap.write_bytes(capture(reverse=True))
        self.assertEqual(inventory(self.pcap)["capture_time_reversals"], 1)

    def test_calibration_change_changes_comparability(self):
        before = self.build()
        self.cal.write_text("changed")
        self.assertFalse(comparable(before, self.build()))

    def test_filename_does_not_change_comparability(self):
        before = self.build()
        after = copy.deepcopy(before)
        after["inputs"]["pcap"]["name"] = "renamed.pcap"
        self.assertTrue(comparable(before, after))

    def test_timestamp_source_changes_comparability(self):
        before = self.build()
        after = copy.deepcopy(before)
        after["declared_settings"]["timestamp_source"] = "receive"
        self.assertFalse(comparable(before, after))

    def test_invalid_rate(self):
        with self.assertRaises(ValueError):
            manifest(self.pcap, self.cal, self.config, "revision", "sensor", float("nan"))

    def test_unsupported_format(self):
        self.pcap.write_bytes(b"not a pcap"*3)
        with self.assertRaises(ValueError):
            inventory(self.pcap)


if __name__ == "__main__":
    unittest.main()
