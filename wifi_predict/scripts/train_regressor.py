#!/usr/bin/env python3

import os
import sys
import yaml
import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from wifi_interface.msg import WifiList
from wifi_predict.gpr_utils import ProgressGPR, MultiGPR
from sklearn.preprocessing import StandardScaler
from sklearn.impute import SimpleImputer
from sklearn.decomposition import PCA
from sklearn.feature_selection import VarianceThreshold
from sklearn.model_selection import GroupKFold
from sklearn.metrics import mean_squared_error
from sklearn.pipeline import Pipeline
import joblib
from ament_index_python.packages import get_package_share_directory
from tqdm import tqdm

OUTPUT_DIR_NAME = "runtime_1D_fingerprint_data"

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
def read_wifi_from_bag(bag_folder_path):
    wifi_msgs = []
    mcap_files = [f for f in os.listdir(bag_folder_path) if f.endswith(".mcap")]
    for f in mcap_files:
        bag_file = os.path.join(bag_folder_path, f)
        if os.path.getsize(bag_file) < 1024:
            continue
        storage_options = StorageOptions(uri=bag_file, storage_id="mcap")
        converter_options = ConverterOptions("", "")
        reader = SequentialReader()
        try:
            reader.open(storage_options, converter_options)
        except RuntimeError as e:
            print(f"[WARN] Cannot open {bag_file}: {e}")
            continue
        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic != "/wifi":
                continue
            msg = deserialize_message(data, WifiList)
            wifi_msgs.append(msg)
    return wifi_msgs

# ----------------------------
def load_wifi_data(bags_root):
    all_wifi_msgs, labels, groups = [], [], []
    bag_idx = 0
    for pos_folder in sorted(os.listdir(bags_root)):
        pos_path = os.path.join(bags_root, pos_folder)
        if not os.path.isdir(pos_path):
            continue
        label = parse_label(pos_folder)
        if not label:
            continue
        msgs = read_wifi_from_bag(pos_path)
        if not msgs:
            continue
        all_wifi_msgs.extend(msgs)
        labels.extend([label] * len(msgs))
        groups.extend([bag_idx] * len(msgs))
        bag_idx += 1
    all_bssids = sorted({m.bssid for msg in all_wifi_msgs for m in msg.measurements})
    return all_wifi_msgs, labels, all_bssids, groups

# ----------------------------
def prepare_dataset(all_wifi_msgs, labels, all_bssids, groups):
    X, group_ids = [], []
    for msg, grp in zip(all_wifi_msgs, groups):
        bssid_to_rssi = {m.bssid: m.rssi for m in msg.measurements}
        X.append([bssid_to_rssi.get(b, np.nan) for b in all_bssids])
        group_ids.append(grp)
    X = np.array(X)
    y = np.array([lbl[:2] for lbl in labels])  # only x, y
    group_ids = np.array(group_ids)
    return X, y, group_ids

# ----------------------------
def train_single_gp(X_train, y_train):
    from sklearn.gaussian_process.kernels import Matern, RationalQuadratic, ConstantKernel, WhiteKernel
    n_features = X_train.shape[1]
    matern = Matern(length_scale=np.ones(n_features), nu=1.5, length_scale_bounds=(0.1, 20.0))
    rq = RationalQuadratic(length_scale=1.0, alpha=1.0, length_scale_bounds=(0.1, 20.0))
    kernel = ConstantKernel(1.0, (0.01, 100)) * (matern + rq) + WhiteKernel(noise_level=0.2, noise_level_bounds=(0.01, 5.0))
    gp = ProgressGPR(kernel=kernel, n_restarts_optimizer=5, alpha=1e-5, normalize_y=True)
    gp.fit(X_train, y_train)
    return gp

def train_gaussian_regressor(X_train, y_train):
    gps = []
    for i in range(2):
        gp = train_single_gp(X_train, y_train[:, i])
        gps.append(gp)
    return gps

# ----------------------------
def main():
    if len(sys.argv) < 2:
        print("Usage: train_regressor.py <bags_root>")
        return

    bags_root = sys.argv[1]
    all_wifi_msgs, labels, all_bssids, groups = load_wifi_data(bags_root)
    if not all_wifi_msgs:
        print("[ERROR] No WiFi messages found. Exiting.")
        return

    X, y, groups = prepare_dataset(all_wifi_msgs, labels, all_bssids, groups)

    # ----------------------------
    # Preprocessing pipeline
    imputer = SimpleImputer(strategy='mean')
    selector = VarianceThreshold(threshold=0.1)
    scaler = StandardScaler()
    pca = PCA(n_components=min(20, X.shape[1]))
    preprocessor = Pipeline([
        ('imputer', imputer),
        ('selector', selector),
        ('scaler', scaler),
        ('pca', pca)
    ])
    X_proc = preprocessor.fit_transform(X)

    # Train GPR on processed features
    final_gps_list = train_gaussian_regressor(X_proc, y)
    final_model = MultiGPR(final_gps_list)

    pkg_share = get_package_share_directory('wifi_predict')
    output_dir = os.path.join(pkg_share, OUTPUT_DIR_NAME)
    os.makedirs(output_dir, exist_ok=True)

    joblib.dump(final_model, os.path.join(output_dir, "gpr_model.pkl"))
    joblib.dump(preprocessor, os.path.join(output_dir, "preprocessor.pkl"))
    with open(os.path.join(output_dir, "fingerprint_bssids.yaml"), "w") as f:
        yaml.dump({"bssids": all_bssids}, f)

    print(f"[INFO] Model, pipeline, and BSSIDs saved to {output_dir}")


if __name__ == "__main__":
    main()
