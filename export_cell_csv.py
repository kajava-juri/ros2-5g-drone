#!/usr/bin/env python3
import argparse
import csv
import json
import os
import sqlite3

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def find_db3(bag_dir: str) -> str:
    for fn in os.listdir(bag_dir):
        if fn.endswith(".db3"):
            return os.path.join(bag_dir, fn)
    raise FileNotFoundError(f"No .db3 found in {bag_dir}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_dir", help="Path to bag folder, e.g. bags/flight_20260116_120000")
    ap.add_argument("--topic", default="/cell/qmi_json", help="Topic name to export")
    ap.add_argument("--out_main", default="cell_measurements.csv")
    ap.add_argument("--out_neighbors", default="cell_neighbors.csv")
    args = ap.parse_args()

    db3 = find_db3(args.bag_dir)

    con = sqlite3.connect(db3)
    cur = con.cursor()

    # Find topic
    cur.execute("SELECT id, name, type FROM topics WHERE name=?", (args.topic,))
    row = cur.fetchone()
    if not row:
        raise SystemExit(f"Topic not found in bag: {args.topic}")
    topic_id, topic_name, topic_type = row

    # Should be std_msgs/msg/String
    msg_cls = get_message(topic_type)

    cur.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp ASC",
        (topic_id,)
    )

    # --- Main measurement CSV ---
    f_main = open(args.out_main, "w", newline="")
    w_main = csv.writer(f_main)
    w_main.writerow([
        "bag_timestamp_ns",
        "utc_t",
        "ros_stamp_ns",

        "mcc", "mnc",
        "reg", "ps", "roam", "rat",

        "plmn", "tac", "gcid", "serv_cell",

        "rssi", "rsrp", "rsrq", "sinr", "snr",

        "intra_earfcn", "intra_band",
        "intra_cells_count",
        "inter_freqs_count",
        "inter_cells_count",
    ])

    # --- Neighbors CSV ---
    f_nb = open(args.out_neighbors, "w", newline="")
    w_nb = csv.writer(f_nb)
    w_nb.writerow([
        "bag_timestamp_ns",
        "utc_t",
        "ros_stamp_ns",

        "type",          # intra/inter
        "earfcn",
        "band",
        "pci",

        "rssi",
        "rsrp",
        "rsrq",

        # optional context fields
        "plmn",
        "tac",
        "serv_cell",
    ])

    n_main = 0
    n_nb = 0

    for ts_ns, blob in cur:
        try:
            s = deserialize_message(blob, msg_cls).data
            d = json.loads(s)
        except Exception:
            # skip corrupted entries
            continue

        sig = d.get("sig", {}) or {}
        cell = d.get("cell", {}) or {}

        intra = cell.get("intra", {}) or {}
        inter = cell.get("inter", []) or []

        intra_cells = intra.get("cells", []) or []

        # count inter cells
        inter_cells_count = 0
        for f in inter:
            inter_cells_count += len((f.get("cells") or []))

        # write main row
        w_main.writerow([
            int(ts_ns),
            d.get("t"),
            d.get("ros_stamp_ns"),

            d.get("mcc"),
            d.get("mnc"),
            d.get("reg"),
            d.get("ps"),
            d.get("roam"),
            d.get("rat"),

            cell.get("plmn"),
            cell.get("tac"),
            cell.get("gcid"),
            cell.get("serv_cell"),

            sig.get("rssi"),
            sig.get("rsrp"),
            sig.get("rsrq"),
            sig.get("sinr"),
            sig.get("snr"),

            intra.get("earfcn"),
            intra.get("band"),

            len(intra_cells),
            len(inter),
            inter_cells_count,
        ])
        n_main += 1

        # write intra neighbors rows
        for c in intra_cells:
            if c is None:
                continue
            w_nb.writerow([
                int(ts_ns),
                d.get("t"),
                d.get("ros_stamp_ns"),

                "intra",
                intra.get("earfcn"),
                intra.get("band"),
                c.get("pci"),

                c.get("rssi"),
                c.get("rsrp"),
                c.get("rsrq"),

                cell.get("plmn"),
                cell.get("tac"),
                cell.get("serv_cell"),
            ])
            n_nb += 1

        # write inter neighbors rows
        for f in inter:
            earfcn = f.get("earfcn")
            band = f.get("band")
            for c in (f.get("cells") or []):
                if c is None:
                    continue
                w_nb.writerow([
                    int(ts_ns),
                    d.get("t"),
                    d.get("ros_stamp_ns"),

                    "inter",
                    earfcn,
                    band,
                    c.get("pci"),

                    c.get("rssi"),
                    c.get("rsrp"),
                    c.get("rsrq"),

                    cell.get("plmn"),
                    cell.get("tac"),
                    cell.get("serv_cell"),
                ])
                n_nb += 1

    f_main.close()
    f_nb.close()
    con.close()

    print(f"Exported main measurements: {n_main} rows -> {args.out_main}")
    print(f"Exported neighbors:         {n_nb} rows -> {args.out_neighbors}")


if __name__ == "__main__":
    main()
