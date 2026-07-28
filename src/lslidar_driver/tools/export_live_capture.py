#!/usr/bin/env python3
"""Export recorded /scan messages into the deterministic LCP replay format."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import rclpy.serialization
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    scan_type = get_message(topic_types["/scan"])
    scans = []
    while reader.has_next():
        topic, payload, _timestamp = reader.read_next()
        if topic != "/scan":
            continue
        message = rclpy.serialization.deserialize_message(payload, scan_type)
        metadata = (
            float(message.angle_min),
            float(message.angle_increment),
            float(message.range_min),
            float(message.range_max),
        )
        scans.append((metadata, [float(value) for value in message.ranges]))
    if not scans:
        raise RuntimeError("the bag contains no /scan messages")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        stream.write("# lcp live-capture replay v1\n")
        writer = csv.writer(stream)
        for metadata, scan in scans:
            writer.writerow(
                [*metadata, len(scan),
                 *(format(value, ".9g") if math.isfinite(value) else "nan" for value in scan)]
            )
    counts = sorted({len(scan) for _metadata, scan in scans})
    print(f"exported {len(scans)} scans with range counts {counts} to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
