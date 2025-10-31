import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.signal import firwin




class WifiKF(Node):

    def __init__(self):
        super().__init__('wifi_kf')

        self.get_logger().info("Starting up")

        input_topic = (
            self.declare_parameter("input_topic","/pose_estimate")
            .get_parameter_value()
            .string_value
        )

        output_topic = (
            self.declare_parameter("output_topic","/pose")
            .get_parameter_value()
            .string_value
        )


        # Establish publisher & Subscriber
        self.subscriber = self.create_subscription(PoseWithCovarianceStamped, input_topic, self.input_callback, 10)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, output_topic, 10)

        self.get_logger().info("Set up and working")





    def input_callback(self, msg):
        # self.get_logger().info(f"Recv Data: {msg}")


        # Publish
        self.get_logger().info(f"Output Pose: {msg}")
        self.publisher.publish(msg)
        






def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = WifiKF()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        print("Shutting down!")

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()