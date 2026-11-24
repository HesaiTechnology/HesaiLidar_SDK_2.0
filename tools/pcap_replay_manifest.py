"""Record replay input digests, declared settings and classic-PCAP statistics."""
import argparse
import hashlib
import json
from pathlib import Path
import struct


def digest(path):
    hasher = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(65536), b""):
            hasher.update(chunk)
    return dict(name=Path(path).name, bytes=Path(path).stat().st_size, sha256=hasher.hexdigest())


def inventory(path):
    formats = {b"\xd4\xc3\xb2\xa1": ("<", 1000000), b"\xa1\xb2\xc3\xd4": (">", 1000000),
               b"\x4d\x3c\xb2\xa1": ("<", 1000000000), b"\xa1\xb2\x3c\x4d": (">", 1000000000)}
    with open(path, "rb") as stream:
        header = stream.read(24)
        if len(header) != 24 or header[:4] not in formats:
            raise ValueError("expected classic PCAP (microsecond or nanosecond precision)")
        endian, precision = formats[header[:4]]
        major, minor, zone, sigfigs, snaplen, linktype = struct.unpack(endian+"HHiiII", header[4:])
        if (major, minor) != (2, 4) or snaplen == 0:
            raise ValueError("unsupported PCAP version or snap length")
        count, captured_bytes, reversals = 0, 0, 0
        first = last = minimum = maximum = None
        while True:
            record = stream.read(16)
            if not record:
                break
            if len(record) != 16:
                raise ValueError("truncated PCAP record header")
            seconds, fraction, included, original = struct.unpack(endian+"IIII", record)
            if fraction >= precision or included > snaplen or included > original:
                raise ValueError("invalid PCAP timestamp or captured length")
            remaining = included
            while remaining:
                chunk = stream.read(min(remaining, 65536))
                if not chunk:
                    raise ValueError("truncated PCAP payload")
                remaining -= len(chunk)
            tick = seconds * precision + fraction
            if last is not None and tick < last:
                reversals += 1
            first = tick if first is None else first
            minimum = tick if minimum is None else min(minimum, tick)
            maximum = tick if maximum is None else max(maximum, tick)
            last = tick
            count += 1
            captured_bytes += included
    return dict(records=count, captured_bytes=captured_bytes, capture_time_reversals=reversals,
                ticks_per_second=precision, first_tick=first, last_tick=last,
                minimum_tick=minimum, maximum_tick=maximum, linktype=linktype)


def manifest(pcap, correction, config, sdk_revision, timestamp_source, play_rate, firetimes=None):
    if not sdk_revision or not isinstance(sdk_revision, str):
        raise ValueError("SDK revision required")
    if timestamp_source not in ("sensor", "receive") or not isinstance(play_rate, (int, float)) or isinstance(play_rate, bool) or not 0 < play_rate < float("inf"):
        raise ValueError("invalid timestamp source or playback rate")
    assets = dict(pcap=digest(pcap), correction=digest(correction), config=digest(config))
    if firetimes is not None:
        assets["firetimes"] = digest(firetimes)
    return dict(schema=1, inputs=assets, capture=inventory(pcap),
                declared_settings=dict(sdk_revision=sdk_revision, timestamp_source=timestamp_source,
                                       play_rate=play_rate), scope="replay_inputs_and_capture_inventory")


def comparable(left, right):
    """Compare input contents/settings independently of local filenames."""
    return (left["schema"] == right["schema"]
            and left["declared_settings"] == right["declared_settings"]
            and {k: v["sha256"] for k, v in left["inputs"].items()}
            == {k: v["sha256"] for k, v in right["inputs"].items()})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcap")
    parser.add_argument("--correction", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--sdk-revision", required=True)
    parser.add_argument("--timestamp-source", required=True, choices=["sensor", "receive"])
    parser.add_argument("--play-rate", type=float, default=1.0)
    parser.add_argument("--firetimes")
    args = parser.parse_args()
    try:
        result = manifest(args.pcap, args.correction, args.config, args.sdk_revision,
                          args.timestamp_source, args.play_rate, args.firetimes)
    except (OSError, ValueError, TypeError) as error:
        print(json.dumps({"error": str(error)}))
        return 2
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
