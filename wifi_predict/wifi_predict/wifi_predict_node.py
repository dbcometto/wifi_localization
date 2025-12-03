#!/home/anandasangli/wifi-localization-venv/bin/python3

import os
import yaml
import rclpy
from rclpy.node import Node
from wifi_interface.msg import WifiList
from geometry_msgs.msg import Point
import numpy as np
import joblib
from ament_index_python.packages import get_package_share_directory

OUTPUT_DIR_NAME = "runtime_1D_fingerprint_data"


class WifiPredictorNode(Node):
    def __init__(self):
        super().__init__('wifi_predict_node')

        # Get package share directory for model
        pkg_share = get_package_share_directory('wifi_predict')
        model_dir = os.path.join(pkg_share, OUTPUT_DIR_NAME)

        # Load trained GPR model
        model_path = os.path.join(model_dir, "gpr_model.pkl")
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file not found: {model_path}")
            raise FileNotFoundError(f"{model_path} does not exist")
        self.model = joblib.load(model_path)
        self.get_logger().info("Loaded GPR model")

        # Load BSSIDs
        bssid_path = os.path.join(model_dir, "fingerprint_bssids.yaml")
        with open(bssid_path, "r") as f:
            self.all_bssids = yaml.safe_load(f)["bssids"]
        self.get_logger().info(f"Loaded {len(self.all_bssids)} BSSIDs")

        # ROS publisher for predicted positions
        self.pred_pub = self.create_publisher(Point, '/wifi_predicted_position', 10)

        # ROS subscriber for WiFi scans
        self.sub = self.create_subscription(WifiList, '/wifi', self.wifi_callback, 10)
        self.get_logger().info("Subscribed to /wifi topic")

    def wifi_to_vector(self, msg):
        """Convert WifiList message to feature vector matching trained model"""
        rss_map = {m.bssid: m.rssi for m in msg.measurements}
        vec = [rss_map.get(b, -100) for b in self.all_bssids]
        return np.array(vec).reshape(1, -1)

    def wifi_callback(self, msg):
        """Predict position from incoming WiFi scan and publish"""
        if not msg.measurements:
            self.get_logger().warn("Received empty WifiList message")
            return

        X = self.wifi_to_vector(msg)
        try:
            pred = self.model.predict(X)
            point_msg = Point()
            point_msg.x = float(pred[0, 0])
            point_msg.y = float(pred[0, 1]) if pred.shape[1] > 1 else 0.0
            point_msg.z = 0.0
            self.pred_pub.publish(point_msg)
            self.get_logger().info(f"Predicted position: {point_msg}")
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