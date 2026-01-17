#!/usr/bin/env python3
import argparse
import csv
import os
import sqlite3

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def find_db3(bag_path: str) -> str:
    # If it's already a .db3 file, return it
    if bag_path.endswith(".db3") and os.path.isfile(bag_path):
        return bag_path
    # Otherwise treat as directory and find .db3 inside
    if os.path.isdir(bag_path):
        for fn in os.listdir(bag_path):
            if fn.endswith(".db3"):
                return os.path.join(bag_path, fn)
    raise FileNotFoundError(f"No .db3 found in {bag_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_path", help="Path to bag folder or .db3 file, e.g. bags/flight.db3")
    ap.add_argument("--topic", default="/fmu/out/vehicle_local_position", help="Topic name to export")
    ap.add_argument("--out", default="vehicle_position.csv", help="Output CSV filename")
    args = ap.parse_args()

    db3 = find_db3(args.bag_path)

    con = sqlite3.connect(db3)
    cur = con.cursor()

    # Find topic
    cur.execute("SELECT id, name, type FROM topics WHERE name=?", (args.topic,))
    row = cur.fetchone()
    if not row:
        raise SystemExit(f"Topic not found in bag: {args.topic}")
    topic_id, topic_name, topic_type = row

    # Should be px4_msgs/msg/VehicleLocalPosition
    msg_cls = get_message(topic_type)

    cur.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp ASC",
        (topic_id,)
    )

    # --- Position CSV ---
    f_out = open(args.out, "w", newline="")
    w_out = csv.writer(f_out)
    w_out.writerow([
        "bag_timestamp_ns",
        "msg_timestamp_us",      # PX4 timestamp in microseconds
        "timestamp_sample_us",   # Raw sensor timestamp
        
        # Position validity flags
        "xy_valid",
        "z_valid",
        "v_xy_valid",
        "v_z_valid",
        
        # Position (NED frame)
        "x_m",
        "y_m",
        "z_m",
        
        # Velocity (NED frame)
        "vx_m_s",
        "vy_m_s",
        "vz_m_s",
        "z_deriv_m_s",
        
        # Acceleration (NED frame)
        "ax_m_s2",
        "ay_m_s2",
        "az_m_s2",
        
        # Heading
        "heading_rad",
        "heading_var",
        "unaided_heading_rad",
        "heading_good_for_control",
        
        # Reset counters
        "xy_reset_counter",
        "z_reset_counter",
        "vxy_reset_counter",
        "vz_reset_counter",
        "heading_reset_counter",
    ])

    n_rows = 0

    for ts_ns, blob in cur:
        try:
            msg = deserialize_message(blob, msg_cls)
        except Exception as e:
            # skip corrupted entries
            print(f"Warning: Failed to deserialize message at {ts_ns}: {e}")
            continue

        w_out.writerow([
            int(ts_ns),
            msg.timestamp,
            msg.timestamp_sample,
            
            msg.xy_valid,
            msg.z_valid,
            msg.v_xy_valid,
            msg.v_z_valid,
            
            msg.x,
            msg.y,
            msg.z,
            
            msg.vx,
            msg.vy,
            msg.vz,
            msg.z_deriv,
            
            msg.ax,
            msg.ay,
            msg.az,
            
            msg.heading,
            getattr(msg, 'heading_var', None),
            getattr(msg, 'unaided_heading', None),
            getattr(msg, 'heading_good_for_control', None),
            
            msg.xy_reset_counter,
            msg.z_reset_counter,
            msg.vxy_reset_counter,
            msg.vz_reset_counter,
            getattr(msg, 'heading_reset_counter', None),
        ])
        n_rows += 1

    f_out.close()
    con.close()

    print(f"Exported position data: {n_rows} rows -> {args.out}")


if __name__ == "__main__":
    main()
