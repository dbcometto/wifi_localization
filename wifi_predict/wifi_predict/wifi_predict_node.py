#!/usr/bin/env python3

import os
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from wifi_interface.msg import WifiList
import joblib
from ament_index_python.packages import get_package_share_directory
from wifi_predict.gpr_utils import MultiGPR
from sklearn.pipeline import Pipeline

OUTPUT_DIR_NAME = "runtime_1D_fingerprint_data"

class WifiPredictorNode(Node):
    def __init__(self):
        super().__init__('wifi_predict_node')

        pkg_share = get_package_share_directory('wifi_predict')
        model_dir = os.path.join(pkg_share, OUTPUT_DIR_NAME)

        # Load trained model
        model_path = os.path.join(model_dir, "gpr_model.pkl")
        self.model = joblib.load(model_path)

        # Load preprocessing pipeline
        preproc_path = os.path.join(model_dir, "preprocessor.pkl")
        self.preprocessor = joblib.load(preproc_path)

        # Load BSSIDs
        bssid_path = os.path.join(model_dir, "fingerprint_bssids.yaml")
        with open(bssid_path, "r") as f:
            self.all_bssids = yaml.safe_load(f)["bssids"]

        # Publisher & subscriber
        self.pred_pub = self.create_publisher(Point, '/wifi_predicted_position', 10)
        self.sub = self.create_subscription(WifiList, '/wifi', self.wifi_callback, 10)

    def wifi_to_vector(self, msg):
        rss_map = {m.bssid: m.rssi for m in msg.measurements}
        vec = [rss_map.get(b, np.nan) for b in self.all_bssids]
        return np.array(vec).reshape(1, -1)

    def wifi_callback(self, msg):
        if not msg.measurements:
            self.get_logger().warn("Received empty WifiList")
            return
        X_raw = self.wifi_to_vector(msg)
        try:
            X = self.preprocessor.transform(X_raw)
            pred = self.model.predict(X)
            point = Point()
            point.x = float(pred[0, 0])
            point.y = float(pred[0, 1]) if pred.shape[1] > 1 else 0.0
            point.z = 0.0
            self.pred_pub.publish(point)
            self.get_logger().info(f"Predicted position: {point}")
        except Exception as e:
            self.get_logger().error(f"Prediction failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WifiPredictorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
