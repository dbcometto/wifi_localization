#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
import numpy as np
import yaml
from scipy.spatial.distance import cdist

from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from wifi_interface.msg import WifiList
from ament_index_python.packages import get_package_share_directory

OUTPUT_DIR_NAME = "runtime_1D_fingerprint_data"  # must match build_fingerprint_db.py

class WifiPredictor(Node):

    def __init__(self):
        super().__init__('wifi_predict_node')

        # ----------------------------
        # Load fingerprint DB
        # ----------------------------
        pkg_share = get_package_share_directory('wifi_predict')
        data_dir = os.path.join(pkg_share, OUTPUT_DIR_NAME)

        self.X = np.load(os.path.join(data_dir, 'fingerprint_X.npy'))
        self.y = np.load(os.path.join(data_dir, 'fingerprint_y.npy'))
        with open(os.path.join(data_dir, 'fingerprint_bssids.yaml'), "r") as f:
            self.bssids = yaml.safe_load(f)["bssids"]

        # Ensure y is numeric
        if self.y.dtype == object:
            self.y = np.vstack([np.array(v, dtype=int) for v in self.y])

        # Detect 1D vs 2D
        self.is_2D = self.y.shape[1] == 2 if len(self.y.shape) > 1 else False

        self.get_logger().info(f"Loaded fingerprint DB with {len(self.X)} samples and {len(self.bssids)} BSSIDs.")
        self.get_logger().info(f"Mode: {'2D' if self.is_2D else '1D'}")

        # ----------------------------
        # Subscriber & Publisher
        # ----------------------------
        self.sub = self.create_subscription(
            WifiList,
            "/wifi",
            self.wifi_callback,
            10
        )

        self.pub = self.create_publisher(Point if self.is_2D else Int32, "/wifi_position", 10)

    # ------------------------------------------------------------
    # Convert WifiList msg → aligned RSSI vector
    # ------------------------------------------------------------
    def vectorize(self, msg: WifiList):
        rss_map = {m.bssid: m.rssi for m in msg.measurements}
        vec = [rss_map.get(b, -100) for b in self.bssids]
        return np.array(vec).reshape(1, -1)

    # ------------------------------------------------------------
    # Handle incoming WifiList message
    # ------------------------------------------------------------
    def wifi_callback(self, msg: WifiList):
        vec = self.vectorize(msg)

        # Compute distances and find nearest fingerprint
        distances = cdist(vec, self.X, metric="euclidean")[0]
        idx = np.argmin(distances)
        predicted_pos = self.y[idx]
        min_dist = float(distances[idx])

        # Publish prediction
        if self.is_2D:
            out = Point()
            out.x, out.y = predicted_pos
            out.z = 0.0
        else:
            out = Int32()
            out.data = int(predicted_pos)

        self.pub.publish(out)
        self.get_logger().info(f"[WiFi Position] Predicted = {predicted_pos}, distance={min_dist:.2f}")

# ------------------------------------------------------------
# Main
# ------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = WifiPredictor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()