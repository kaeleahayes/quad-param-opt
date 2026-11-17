#!/usr/bin/env python3
"""Export each topic in a ROS 2 bag to its own CSV file."""

import argparse
import csv
import json
import re
from contextlib import ExitStack
from pathlib import Path


def flatten_message(message, prefix=""):
    """Return message fields as a flat mapping, using dots for nested fields."""
    flattened = {}
    for field_name in message.get_fields_and_field_types():
        value = getattr(message, field_name)
        column_name = f"{prefix}.{field_name}" if prefix else field_name

        if hasattr(value, "get_fields_and_field_types"):
            flattened.update(flatten_message(value, column_name))
        elif isinstance(value, (list, tuple, bytes, bytearray)):
            flattened[column_name] = json.dumps(list(value))
        else:
            flattened[column_name] = value
    return flattened


def topic_csv_name(topic, used_names):
    name = re.sub(r"[^A-Za-z0-9_.-]+", "_", topic.strip("/")) or "topic"
    candidate = name
    suffix = 2
    while candidate in used_names:
        candidate = f"{name}_{suffix}"
        suffix += 1
    used_names.add(candidate)
    return f"{candidate}.csv"


def export_bag(bag_path, output_dir):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise SystemExit(
            "ROS 2 Python packages are required. Run this script in a ROS 2 environment."
        ) from exc

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )

    try:
        topics = reader.get_all_topics_and_types()
        topic_types = {topic.name: topic.type for topic in topics}
        output_dir.mkdir(parents=True, exist_ok=True)

        used_names = set()
        with ExitStack() as stack:
            writers = {}
            for topic, type_name in topic_types.items():
                try:
                    message_type = get_message(type_name)
                except (ImportError, AttributeError) as exc:
                    raise RuntimeError(
                        f"Cannot load {type_name} for topic {topic}. "
                        "Source the ROS 2 workspace that defines this message type."
                    ) from exc
                columns = list(flatten_message(message_type()).keys())
                filename = topic_csv_name(topic, used_names)
                csv_file = stack.enter_context(
                    (output_dir / filename).open("w", newline="", encoding="utf-8")
                )
                writer = csv.DictWriter(
                    csv_file, fieldnames=["timestamp_ns", *columns]
                )
                writer.writeheader()
                writers[topic] = (writer, message_type)

            while reader.has_next():
                topic, serialized_message, timestamp_ns = reader.read_next()
                writer, message_type = writers[topic]
                message = deserialize_message(serialized_message, message_type)
                writer.writerow(
                    {
                        "timestamp_ns": timestamp_ns,
                        **flatten_message(message),
                    }
                )
    finally:
        reader.close()


def main():
    parser = argparse.ArgumentParser(
        description="Export each topic in a ROS 2 bag to a separate CSV file."
    )
    parser.add_argument("bag", type=Path, help="path to a ROS 2 bag directory")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="output directory (default: <bag-name>_csv next to the bag)",
    )
    args = parser.parse_args()

    bag_path = args.bag.expanduser().resolve()
    if not bag_path.exists():
        parser.error(f"bag path does not exist: {bag_path}")
    if not bag_path.is_dir():
        parser.error(f"expected a ROS 2 bag directory: {bag_path}")

    output_dir = (
        args.output.expanduser().resolve()
        if args.output
        else bag_path.with_name(f"{bag_path.name}_csv")
    )
    try:
        export_bag(bag_path, output_dir)
    except RuntimeError as exc:
        parser.error(str(exc))
    print(f"CSV files written to {output_dir}")


if __name__ == "__main__":
    main()