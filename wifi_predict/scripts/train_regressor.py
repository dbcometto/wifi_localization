#!/usr/bin/env python3

import os
import sys
import yaml
import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from wifi_interface.msg import WifiList
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel
import joblib
from ament_index_python.packages import get_package_share_directory
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error

OUTPUT_DIR_NAME = "runtime_1D_fingerprint_data"


# ----------------------------
# Parse numeric label from folder name
# ----------------------------
def parse_label(folder_name):
    parts = folder_name.split("_")
    label = []
    for p in parts:
        try:
            label.append(int(p))
        except ValueError:
            break
    return label if label else None


# ----------------------------
# Read WiFi messages from a bag
# ----------------------------
def read_wifi_from_bag(bag_folder_path):
    wifi_msgs = []
    mcap_files = [f for f in os.listdir(bag_folder_path) if f.endswith(".mcap")]
    if not mcap_files:
        print(f"[WARN] No .mcap files found in {bag_folder_path}")
        return []

    for f in mcap_files:
        bag_file = os.path.join(bag_folder_path, f)
        if os.path.getsize(bag_file) < 1024:
            print(f"[WARN] Skipping tiny file {bag_file}")
            continue

        storage_options = StorageOptions(uri=bag_file, storage_id="mcap")
        converter_options = ConverterOptions("", "")
        reader = SequentialReader()
        try:
            reader.open(storage_options, converter_options)
        except RuntimeError as e:
            print(f"[WARN] Cannot open {bag_file}: {e}")
            continue

        count = 0
        while reader.has_next():
            topic, data, t = reader.read_next()
            if topic != "/wifi":
                continue
            msg = deserialize_message(data, WifiList)
            wifi_msgs.append(msg)
            count += 1
        print(f"[INFO] Read {count} messages from {bag_file}")

    return wifi_msgs


# ----------------------------
# Vectorize WiFi message
# ----------------------------
def vectorize_wifi(msg, bssids):
    rss_map = {m.bssid: m.rssi for m in msg.measurements}
    vec = [rss_map.get(b, -100) for b in bssids]
    return np.array(vec)


# ----------------------------
# Load WiFi data from bags
# ----------------------------
def load_wifi_data(bags_root):
    all_wifi_msgs = []
    labels = []

    for pos_folder in sorted(os.listdir(bags_root)):
        pos_path = os.path.join(bags_root, pos_folder)
        if not os.path.isdir(pos_path):
            continue

        label = parse_label(pos_folder)
        if not label:
            print(f"[WARN] Skipping folder '{pos_folder}', no numeric label found")
            continue

        msgs = read_wifi_from_bag(pos_path)
        if not msgs:
            continue

        all_wifi_msgs.extend(msgs)
        labels.extend([label] * len(msgs))
        print(f"[INFO] Read {len(msgs)} messages from folder '{pos_folder}' with label {label}")

    # Discover all unique BSSIDs
    all_bssids = sorted({m.bssid for msg in all_wifi_msgs for m in msg.measurements})
    print(f"[INFO] Found {len(all_bssids)} unique BSSIDs")

    return all_wifi_msgs, labels, all_bssids


# ----------------------------
# Prepare training/testing datasets
# ----------------------------
def prepare_datasets(all_wifi_msgs, labels, all_bssids):
    X = []

    for msg in all_wifi_msgs:
        feature_vector = []
        bssid_to_rssi = {m.bssid: m.rssi for m in msg.measurements}
        for bssid in all_bssids:
            feature_vector.append(bssid_to_rssi.get(bssid, -100))
        X.append(feature_vector)

    X = np.array(X)
    y = np.array([lbl + [0]*(3 - len(lbl)) for lbl in labels])

    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=42
    )

    print(f"[INFO] Train samples: {X_train.shape}, Test samples: {X_test.shape}")
    return X_train, y_train, X_test, y_test


# ----------------------------
# Train Gaussian Process Regressor
# ----------------------------
def train_regressor(X_train, y_train):
    kernel = RBF(length_scale=1.0) + WhiteKernel(noise_level=1)
    model = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=3)
    model.fit(X_train, y_train)
    print("[INFO] Regression model trained successfully")
    return model


# ----------------------------
# Main
# ----------------------------
def main():
    if len(sys.argv) < 2:
        print("Usage: train_regressor.py <bags_root>")
        return

    bags_root = sys.argv[1]
    print(f"[INFO] Using bags directory: {bags_root}")

    all_wifi_msgs, labels, all_bssids = load_wifi_data(bags_root)
    if not all_wifi_msgs:
        print("[ERROR] No WiFi messages found. Exiting.")
        return

    X_train, y_train, X_test, y_test = prepare_datasets(all_wifi_msgs, labels, all_bssids)
    if X_train.size == 0 or y_train.size == 0:
        print("[ERROR] No training data available. Exiting.")
        return

    model = train_regressor(X_train, y_train)

    # ----------------------------
    # Evaluate on test set
    # ----------------------------
    y_pred = model.predict(X_test)
    mse = mean_squared_error(y_test, y_pred)
    errors = np.linalg.norm(y_pred - y_test, axis=1)
    print(f"[INFO] Test MSE: {mse:.3f}")
    print(f"[INFO] Mean Euclidean distance error: {np.mean(errors):.2f} units")
    for i in range(min(5, len(y_test))):
        print(f"True: {y_test[i]}, Predicted: {y_pred[i]}")

    # ----------------------------
    # Save model & BSSIDs
    # ----------------------------
    pkg_share = get_package_share_directory('wifi_predict')
    output_dir = os.path.join(pkg_share, OUTPUT_DIR_NAME)
    os.makedirs(output_dir, exist_ok=True)

    joblib.dump(model, os.path.join(output_dir, "gpr_model.pkl"))
    with open(os.path.join(output_dir, "fingerprint_bssids.yaml"), "w") as f:
        yaml.dump({"bssids": all_bssids}, f)

    print(f"[INFO] Model and BSSIDs saved to {output_dir}")


if __name__ == "__main__":
    main()
