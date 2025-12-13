import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from wifi_interface.msg import WifiPosition
import numpy as np




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


        # self.x = np.zeros((3,1)) # Starting state
        # self.P = 1e6*np.eye(3) # Starting state covariance
        
        # self.Q = 1e-6*np.eye(3) # Minimal process noise

        # self.F = np.eye(3) # Stationary model
        # self.H = np.eye(3) # Directly observe all states

        self.x = np.zeros((2,1)) # Starting state
        self.P = 1e6*np.eye(2) # Starting state covariance
        
        self.Q = 1e-6*np.eye(2) # Minimal process noise

        self.F = np.eye(2) # Stationary model
        self.H = np.eye(2) # Directly observe all states
        

        # Establish publisher & Subscriber
        self.subscriber = self.create_subscription(WifiPosition, input_topic, self.input_callback, 10)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, output_topic, 10)

        self.get_logger().info("Set up and working")





    def input_callback(self, msg: WifiPosition): # PoseWithCovarianceStamped
        # self.get_logger().info(f"Recv Data: {msg}")

        # Get data from msg
        # z = np.array([msg.pose.pose.position.x,
        #      msg.pose.pose.position.y,
        #      msg.pose.pose.position.z]).reshape(-1,1)
        
        z = np.array([msg.x,
             msg.y]).reshape(-1,1)
        
        # R = np.zeros((3,3))
        # R[0,0] = msg.pose.covariance[0]
        # R[1,1] = msg.pose.covariance[7]
        # R[2,2] = msg.pose.covariance[14]

        R = np.zeros((2,2))
        R[0,0] = msg.covariance[0]
        R[1,1] = msg.covariance[3]


        # Predict

        x_hat = self.F @ self.x
        P_hat = self.F @ self.P @ self.F.T + self.Q

        # Update
        K = P_hat @ self.H.T @ np.linalg.inv((self.H @ P_hat @ self.H.T + R))
        self.x = x_hat + K @ (z - self.H @ x_hat)
        # self.P = (np.eye(3) - K @ self.H) @ P_hat
        self.P = (np.eye(2) - K @ self.H) @ P_hat


        # Create message
        timestamp = self.get_clock().now().to_msg()
        out_msg = PoseWithCovarianceStamped()

        out_msg.header.frame_id = "base"
        out_msg.header.stamp = timestamp

        out_msg.pose.pose.position.x = self.x[0,0]
        out_msg.pose.pose.position.y = self.x[1,0]
        # out_msg.pose.pose.position.z = self.x[2,0]
        out_msg.pose.pose.position.z = 0.0

        out_covariance = np.zeros((6, 6), dtype=np.float64)
        # out_covariance[0:3,0:3] = self.P
        out_covariance[0:2,0:2] = self.P
        flat_out_covariance= out_covariance.flatten()

        out_msg.pose.covariance = flat_out_covariance.tolist()

        # Publish
        self.get_logger().info(f"Output Pose: {out_msg}")
        self.publisher.publish(out_msg)
        






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