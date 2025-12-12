#!/usr/bin/env python3

import os
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from wifi_interface.msg import WifiList, WifiPosition
import joblib
from ament_index_python.packages import get_package_share_directory
# from wifi_predict.gpr_utils import MultiGPR
# from sklearn.pipeline import Pipeline

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
        self.kf_pub = self.create_publisher(WifiPosition, '/wifi_kf_position', 10)
        self.sub = self.create_subscription(WifiList, '/wifi', self.wifi_callback, 10)

        self.get_logger().info("wifi_predict_node ready.")

    # Convert WifiList to RSSI vector in correct BSSID order
    def wifi_to_vector(self, msg):
        rss_map = {m.bssid: m.rssi for m in msg.measurements}
        vec = [rss_map.get(b, np.nan) for b in self.all_bssids]
        return np.array(vec).reshape(1, -1)

    def wifi_callback(self, msg):
        if not msg.measurements:
            self.get_logger().warn("Received empty WifiList")
            return
        
        # Preprocess
        X_raw = self.wifi_to_vector(msg)
        try:
            X = self.preprocessor.transform(X_raw)

            # -------------------------------
            # GPR prediction WITH covariance
            # -------------------------------
            mean, cov = self.model.predict(X, return_cov=True)

            # mean: shape (1, 2)
            # cov: shape (1, 2, 2)

            x_mean = float(mean[0, 0])
            y_mean = float(mean[0, 1])

            xx = float(cov[0][0, 0])
            xy = float(cov[0][0, 1])
            yx = float(cov[0][1, 0])
            yy = float(cov[0][1, 1])

            # -------------------------------
            # Publish backward-compatible message
            # -------------------------------
            point = Point()
            point.x = x_mean
            point.y = y_mean
            point.z = 0.0
            self.pred_pub.publish(point)

            # -------------------------------
            # Publish new Kalman-compatible message
            # -------------------------------
            kf_msg = WifiPosition()
            kf_msg.x = x_mean
            kf_msg.y = y_mean
            kf_msg.covariance = [xx, xy, yx, yy]
            self.kf_pub.publish(kf_msg)

            self.get_logger().info(
                f"Predicted: ({x_mean:.2f}, {y_mean:.2f})  "
                f"Cov=[[{xx:.3f}, {xy:.3f}], [{yx:.3f}, {yy:.3f}]]"
            )
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
