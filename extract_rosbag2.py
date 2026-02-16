#!/usr/bin/env python3
"""
Extract images and IMU data from a ROS2 bag (.db3) into EuRoC-like format
expected by ORB-SLAM3's mono_inertial_euroc.

Usage:
    python3 extract_rosbag2.py \
        --bag /path/to/rosbag_dir_or_file.db3 \
        --output /path/to/output_dataset \
        --image_topic /cam0/image_raw/compressed \
        --imu_topic /imu/data_raw

Output structure (EuRoC format):
    <output>/
      mav0/
        cam0/
          data/
            <timestamp_ns>.png
            ...
          data.csv          # image timestamps
        imu0/
          data.csv          # IMU measurements
      timestamps.txt        # one nanosecond timestamp per line

Requirements:
    pip install opencv-python numpy
"""

import argparse
import os
import sys
from pathlib import Path

try:
    import cv2
    import numpy as np
    import sqlite3
    import struct
except ImportError:
    print("Error: Required packages not found. Install with:")
    print("  pip install opencv-python numpy")
    sys.exit(1)


def parse_cdr_stamp(data, offset):
    """Parse a builtin_interfaces/Time from CDR bytes: sec (int32) + nanosec (uint32)."""
    sec, nanosec = struct.unpack_from('<iI', data, offset)
    return sec, nanosec, offset + 8


def parse_cdr_string(data, offset):
    """Parse a CDR string: uint32 length (including null) + chars + null."""
    length = struct.unpack_from('<I', data, offset)[0]
    s = data[offset + 4: offset + 4 + length - 1].decode('utf-8', errors='replace')
    return s, offset + 4 + length


def parse_compressed_image_cdr(data):
    """Parse sensor_msgs/msg/CompressedImage from raw CDR bytes.
    Layout: [CDR header 4B] [Header] [format string] [data sequence]
    Header: stamp (sec i32 + nanosec u32) + frame_id (string)
    """
    off = 4  # skip CDR encapsulation header
    sec, nanosec, off = parse_cdr_stamp(data, off)
    _frame_id, off = parse_cdr_string(data, off)
    # Align to 4 bytes before next field
    off = (off + 3) & ~3
    _format, off = parse_cdr_string(data, off)
    # Align to 4 bytes before sequence
    off = (off + 3) & ~3
    # data: sequence<uint8> = uint32 length + bytes
    data_len = struct.unpack_from('<I', data, off)[0]
    off += 4
    img_bytes = data[off: off + data_len]
    return sec, nanosec, img_bytes


def parse_imu_cdr(data):
    """Parse sensor_msgs/msg/Imu from raw CDR bytes.
    Layout: [CDR header 4B] [Header] [orientation quat 4×f64] [orientation_cov 9×f64]
            [angular_velocity 3×f64] [angular_velocity_cov 9×f64]
            [linear_acceleration 3×f64] [linear_acceleration_cov 9×f64]
    """
    off = 4  # skip CDR encapsulation header
    sec, nanosec, off = parse_cdr_stamp(data, off)
    _frame_id, off = parse_cdr_string(data, off)
    # Align to 4 bytes (not 8) before the Quaternion struct
    off = (off + 3) & ~3
    # orientation: 4 doubles (x,y,z,w)
    off += 4 * 8
    # orientation_covariance: 9 doubles
    off += 9 * 8
    # angular_velocity: 3 doubles (x,y,z)
    gx, gy, gz = struct.unpack_from('<3d', data, off)
    off += 3 * 8
    # angular_velocity_covariance: 9 doubles
    off += 9 * 8
    # linear_acceleration: 3 doubles (x,y,z)
    ax, ay, az = struct.unpack_from('<3d', data, off)
    return sec, nanosec, gx, gy, gz, ax, ay, az


def extract_dataset(bag_path, output_dir, image_topic, imu_topic):
    """Extract images and IMU from ROS2 bag into EuRoC layout."""

    bag_path = Path(bag_path)
    output_dir = Path(output_dir)

    # If user passed a .db3 file, use it directly
    if bag_path.is_dir():
        # Find the .db3 file inside the directory
        db3_files = list(bag_path.glob("*.db3"))
        if not db3_files:
            print(f"Error: no .db3 file found in {bag_path}")
            return 0, 0
        db3_path = db3_files[0]
        print(f"Found .db3 file: {db3_path}")
    else:
        db3_path = bag_path

    # Create output directories
    cam_data_dir = output_dir / "mav0" / "cam0" / "data"
    imu_dir = output_dir / "mav0" / "imu0"
    cam_data_dir.mkdir(parents=True, exist_ok=True)
    imu_dir.mkdir(parents=True, exist_ok=True)

    image_timestamps = []  # list of int (nanoseconds)
    imu_rows = []  # list of (timestamp_ns, gx, gy, gz, ax, ay, az)
    img_count = 0
    imu_count = 0

    print(f"Opening ROS2 bag: {db3_path}")
    print(f"Image topic:      {image_topic}")
    print(f"IMU topic:        {imu_topic}")
    print(f"Output dir:       {output_dir}")
    print()

    conn = sqlite3.connect(str(db3_path))
    cursor = conn.cursor()

    # Get topic name → id mapping
    cursor.execute("SELECT id, name, type FROM topics")
    topics = {row[1]: (row[0], row[2]) for row in cursor.fetchall()}

    print("Available topics:")
    for name, (tid, ttype) in topics.items():
        print(f"  {name}  [{ttype}]")
    print()

    if image_topic not in topics:
        print(f"Error: image topic '{image_topic}' not found in bag")
        conn.close()
        return 0, 0

    if imu_topic not in topics:
        print(f"Warning: IMU topic '{imu_topic}' not found in bag")

    # Extract images
    image_topic_id = topics[image_topic][0]
    image_topic_type = topics[image_topic][1]
    print(f"Extracting images from topic '{image_topic}' (type: {image_topic_type}) ...")

    cursor.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
        (image_topic_id,)
    )

    for row in cursor:
        bag_timestamp_ns, rawdata = row
        try:
            if "CompressedImage" in image_topic_type:
                sec, nanosec, img_bytes = parse_compressed_image_cdr(rawdata)
                np_arr = np.frombuffer(img_bytes, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
                stamp_ns = int(sec * 1_000_000_000 + nanosec)
            else:
                print(f"  Warning: unsupported image type '{image_topic_type}', skipping")
                continue

            if cv_image is None:
                print(f"  Warning: failed to decode image #{img_count}")
                continue

            filename = f"{stamp_ns}.png"
            cv2.imwrite(str(cam_data_dir / filename), cv_image)
            image_timestamps.append(stamp_ns)
            img_count += 1

            if img_count % 100 == 0:
                print(f"  Extracted {img_count} images ...")

        except Exception as e:
            print(f"  Warning: image extraction error: {e}")
            continue

    # Extract IMU
    if imu_topic in topics:
        imu_topic_id = topics[imu_topic][0]
        imu_topic_type = topics[imu_topic][1]
        print(f"\nExtracting IMU from topic '{imu_topic}' (type: {imu_topic_type}) ...")

        cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
            (imu_topic_id,)
        )

        for row in cursor:
            bag_timestamp_ns, rawdata = row
            try:
                sec, nanosec, gx, gy, gz, ax, ay, az = parse_imu_cdr(rawdata)
                stamp_ns = int(sec * 1_000_000_000 + nanosec)
                imu_rows.append((stamp_ns, gx, gy, gz, ax, ay, az))
                imu_count += 1

                if imu_count % 5000 == 0:
                    print(f"  Extracted {imu_count} IMU samples ...")

            except Exception as e:
                print(f"  Warning: IMU extraction error: {e}")
                continue

    conn.close()

    # Sort both by timestamp
    image_timestamps.sort()
    imu_rows.sort(key=lambda r: r[0])

    # ---- Write image timestamps.txt (one ns-timestamp per line) ----
    ts_file = output_dir / "timestamps.txt"
    with open(ts_file, "w") as f:
        for ts in image_timestamps:
            f.write(f"{ts}\n")

    # ---- Write cam0 data.csv (optional, for reference) ----
    cam_csv = output_dir / "mav0" / "cam0" / "data.csv"
    with open(cam_csv, "w") as f:
        f.write("#timestamp [ns],filename\n")
        for ts in image_timestamps:
            f.write(f"{ts},{ts}.png\n")

    # ---- Write imu0 data.csv ----
    imu_csv = imu_dir / "data.csv"
    with open(imu_csv, "w") as f:
        f.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
                "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
                "a_RS_S_z [m s^-2]\n")
        for row in imu_rows:
            f.write(f"{row[0]},{row[1]:.10f},{row[2]:.10f},{row[3]:.10f},"
                    f"{row[4]:.10f},{row[5]:.10f},{row[6]:.10f}\n")

    print()
    print("=" * 60)
    print(f"  Images extracted: {img_count}")
    print(f"  IMU samples:      {imu_count}")
    print(f"  Output directory:  {output_dir}")
    print()
    print("  Files created:")
    print(f"    {ts_file}")
    print(f"    {cam_csv}")
    print(f"    {imu_csv}")
    print(f"    {cam_data_dir}/<timestamp>.png")
    print("=" * 60)
    print()
    print("Run ORB-SLAM3 with:")
    print(f"  cd /home/samuelecentanni/github/ORB_SLAM3")
    print(f"  export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH")
    print(f"  ./Examples/Monocular-Inertial/mono_inertial_euroc \\")
    print(f"      ./Vocabulary/ORBvoc.txt \\")
    print(f"      ./Examples/Monocular-Inertial/Insta360.yaml \\")
    print(f"      {output_dir} \\")
    print(f"      {ts_file}")

    return img_count, imu_count


def main():
    parser = argparse.ArgumentParser(
        description="Extract ROS2 bag to EuRoC format for ORB-SLAM3"
    )
    parser.add_argument("--bag", required=True,
                        help="Path to ROS2 bag directory or .db3 file")
    parser.add_argument("--output", required=True,
                        help="Output dataset directory")
    parser.add_argument("--image_topic", default="/cam0/image_raw/compressed",
                        help="Image topic (default: /cam0/image_raw/compressed)")
    parser.add_argument("--imu_topic", default="/imu/data_raw",
                        help="IMU topic (default: /imu/data_raw)")

    args = parser.parse_args()

    if not os.path.exists(args.bag):
        print(f"Error: bag not found: {args.bag}")
        sys.exit(1)

    img_n, imu_n = extract_dataset(args.bag, args.output,
                                    args.image_topic, args.imu_topic)
    if img_n == 0:
        print("No images extracted!")
        sys.exit(1)
    if imu_n == 0:
        print("Warning: no IMU data extracted (monocular-only will still work)")


if __name__ == "__main__":
    main()
