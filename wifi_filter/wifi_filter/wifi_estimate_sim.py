import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
import random
import numpy as np




class DriverSim(Node):

    def __init__(self):
        super().__init__('estimate_sim')

        self.get_logger().info("Starting up")

        output_topic = (
            self.declare_parameter("output_topic","/pose_estimate")
            .get_parameter_value()
            .string_value
        )

        # Establish subscriber
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, output_topic, 10)

        # Establish timer/publisher
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Set up and working")

        self.starting = True





    def timer_callback(self):

        msg = PoseWithCovarianceStamped()
        # measure = WifiMeasurement()
        # measure.bssid = "11:22:33:44:55:66"
        # measure.rssi = random.gauss(1,0.5)
        # measure.variance = 0.5
        # msg.measurements.append(measure)
        timestamp = self.get_clock().now().to_msg()

        msg.header.frame_id = "base"
        msg.header.stamp = timestamp

        variance = 0.5
        x = 1
        y = 1
        z = 0

        msg.pose.pose.position.x = random.gauss(x,variance)
        msg.pose.pose.position.y = random.gauss(y,variance)
        msg.pose.pose.position.z = random.gauss(z,variance)



        covariance = np.zeros((6, 6), dtype=np.float64)
        covariance[0][0] = variance
        covariance[1][1] = variance
        covariance[2][2] = variance

        flat_covariance = covariance.flatten()

        msg.pose.covariance = flat_covariance.tolist()



        self.publisher.publish(msg)

        self.get_logger().info(f"Tx Estimate: {msg}")
        






def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = DriverSim()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        print("Shutting down!")

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()