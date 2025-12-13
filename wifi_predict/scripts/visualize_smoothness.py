#!/usr/bin/env python3
import os
import sys
import yaml
import numpy as np
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from wifi_interface.msg import WifiList

from sklearn.gaussian_process import GaussianProcessRegressor
from ament_index_python.packages import get_package_share_directory
import joblib


# ----------------------------
# Read WiFi messages
# ----------------------------
def read_wifi_from_bag(folder):
    msgs = []
    files = [f for f in os.listdir(folder) if f.endswith(".mcap")]

    for f in files:
        p = os.path.join(folder, f)
        if os.path.getsize(p) < 1024:
            continue

        reader = SequentialReader()
        reader.open(StorageOptions(uri=p, storage_id="mcap"),
                    ConverterOptions("", ""))

        while reader.has_next():
            topic, data, t = reader.read_next()
            if topic != "/wifi":
                continue
            msg = deserialize_message(data, WifiList)
            msgs.append(msg)

    return msgs


# ----------------------------
# Convert RSSI vector
# ----------------------------
def vectorize(msg, bssids):
    rss = {m.bssid: m.rssi for m in msg.measurements}
    return np.array([rss.get(b, -100) for b in bssids])


# ----------------------------
# MAIN
# ----------------------------
def main():
    if len(sys.argv) < 2:
        print("Usage: visualize_smoothness.py <bags_root>")
        return

    bags_root = sys.argv[1]

    # --------------------------------–
    # Load regressor + BSSIDs
    # --------------------------------–
    pkg_share = get_package_share_directory("wifi_predict")
    model_dir = os.path.join(pkg_share, "runtime_1D_fingerprint_data")
    with open(os.path.join(model_dir, "fingerprint_bssids.yaml")) as f:
        bssids = yaml.safe_load(f)["bssids"]
    model = joblib.load(os.path.join(model_dir, "gpr_model.pkl"))


    # --------------------------------–
    # Gather all fingerprints
    # --------------------------------–
    positions = []
    preds = []
    raw_rssi = []

    for folder in sorted(os.listdir(bags_root)):
        full = os.path.join(bags_root, folder)
        if not os.path.isdir(full):
            continue

        try:
            pos = int(folder)
        except:
            continue

        msgs = read_wifi_from_bag(full)
        for m in msgs:
            vec = vectorize(m, bssids)
            positions.append(pos)
            raw_rssi.append(vec)
            preds.append(model.predict(vec.reshape(1, -1))[0])

    positions = np.array(positions)
    raw_rssi = np.array(raw_rssi)
    preds = np.array(preds)

    # --------------------------------–
    # Plot A: Raw RSSI Smoothness
    # --------------------------------–
    plt.figure(figsize=(12, 4))
    plt.imshow(raw_rssi.T, aspect="auto", cmap="inferno")
    plt.colorbar(label="RSSI (dBm)")
    plt.title("RSSI Smoothness Across Position")
    plt.ylabel("BSSID Index")
    plt.xlabel("Sample Index")
    plt.tight_layout()

    # --------------------------------–
    # Plot B: Regression Smoothness
    # --------------------------------–
    plt.figure(figsize=(12, 4))
    plt.plot(positions, label="Ground Truth", linewidth=2)
    plt.plot(preds, label="Predicted", linewidth=2)
    plt.title("Regression Smoothness: Predicted vs Actual")
    plt.xlabel("Sample Index")
    plt.ylabel("1D Position")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    plt.show()


if __name__ == "__main__":
    main()
