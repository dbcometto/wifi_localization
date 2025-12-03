#!/usr/bin/env python3

import os
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from wifi_interface.msg import WifiList

OUTPUT_DIR_NAME = "runtime_1D_fingerprint_data"

# ----------------------------
# Read WiFi messages from a bag (robust version)
# ----------------------------
def read_wifi_from_bag(bag_folder_path):
    wifi_msgs = []

    # Find all .mcap files in the folder
    mcap_files = [f for f in os.listdir(bag_folder_path) if f.endswith(".mcap")]
    if not mcap_files:
        print(f"No .mcap files found in {bag_folder_path}")
        return []

    for f in mcap_files:
        bag_file = os.path.join(bag_folder_path, f)

        # Skip tiny/empty files
        if os.path.getsize(bag_file) < 1024:
            print(f"Skipping file {bag_file} (too small or empty)")
            continue

        storage_options = StorageOptions(uri=bag_file, storage_id="mcap")
        converter_options = ConverterOptions("", "")
        reader = SequentialReader()
        
        try:
            reader.open(storage_options, converter_options)
        except RuntimeError as e:
            print(f"Skipping file {bag_file} (cannot open): {e}")
            continue

        count = 0
        while reader.has_next():
            topic, data, t = reader.read_next()
            if topic != "/wifi":
                continue
            msg = deserialize_message(data, WifiList)
            wifi_msgs.append(msg)
            count += 1

        print(f"Read {count} messages from {bag_file}")

    return wifi_msgs


# ----------------------------
# Vectorize WiFi message
# ----------------------------
def vectorize_wifi(msg, bssids):
    rss_map = {m.bssid: m.rssi for m in msg.measurements}
    vec = [rss_map.get(b, -100) for b in bssids]
    return np.array(vec)


# ----------------------------
# Main function
# ----------------------------
def main():
    # Determine workspace root relative to this script
    script_dir = os.path.dirname(os.path.realpath(__file__))  # .../wifi_predict/scripts
    workspace_root = os.path.abspath(os.path.join(script_dir, "..", ".."))  # go up 2 levels

    # Bags folder in workspace
    bags_root = os.path.join(workspace_root, "bags")  # wifi_localization/bags

    # Output folder in the package share directory (where predictor expects it)
    from ament_index_python.packages import get_package_share_directory
    pkg_share = get_package_share_directory('wifi_predict')
    output_dir = os.path.join(pkg_share, OUTPUT_DIR_NAME)  # install/.../share/wifi_predict/runtime_1D_fingerprint_data

    if not os.path.exists(bags_root):
        print(f"Error: Bags folder not found at {bags_root}")
        return

    # Warn if output already exists
    if os.path.exists(output_dir):
        print(f"Warning: Output directory {output_dir} already exists. Files inside will be overwritten.")

    os.makedirs(output_dir, exist_ok=True)

    all_wifi_msgs = []
    labels = []

    for pos_folder in sorted(os.listdir(bags_root)):
        pos_path = os.path.join(bags_root, pos_folder)
        if not os.path.isdir(pos_path):
            continue

        # Determine 1D or 2D label
        parts = pos_folder.split("_")
        try:
            label = [int(parts[0])]
            if len(parts) > 1:
                label.append(int(parts[1]))
        except ValueError:
            print(f"Skipping folder {pos_folder} (cannot parse label)")
            continue

        msgs = read_wifi_from_bag(pos_path)
        all_wifi_msgs.extend(msgs)
        labels.extend([label]*len(msgs))
        print(f"Read {len(msgs)} messages from folder '{pos_folder}' with label {label}")

    # Discover all unique BSSIDs
    all_bssids = sorted({m.bssid for msg in all_wifi_msgs for m in msg.measurements})
    print(f"Found {len(all_bssids)} unique BSSIDs")

    # Build X and y
    X = np.vstack([vectorize_wifi(msg, all_bssids) for msg in all_wifi_msgs])
    y = np.array(labels)

    # Save fingerprint DB
    for fname, data in [
        ("fingerprint_X.npy", X),
        ("fingerprint_y.npy", y),
        ("fingerprint_bssids.yaml", {"bssids": all_bssids})
    ]:
        fpath = os.path.join(output_dir, fname)
        if os.path.exists(fpath):
            print(f"Overwriting existing file: {fpath}")
        if fname.endswith(".yaml"):
            with open(fpath, "w") as f:
                yaml.dump(data, f)
        else:
            np.save(fpath, data)

    print(f"Done building WiFi fingerprint DB!")
    print(f"Fingerprint files saved to: {output_dir}")
    print(f"- fingerprint_X.npy {X.shape}")
    print(f"- fingerprint_y.npy {y.shape}")
    print(f"- fingerprint_bssids.yaml ({len(all_bssids)} BSSIDs)")



if __name__ == "__main__":
    main()
