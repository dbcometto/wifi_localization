import rclpy
from rclpy.node import Node

from wifi_interface.msg import WifiList, WifiMeasurement
import serial
import time
import scipy as sp
import random




class DriverSim(Node):

    def __init__(self):
        super().__init__('node')

        self.get_logger().info("Starting up")

        output_topic = (
            self.declare_parameter("output_topic","/raw_wifi_data")
            .get_parameter_value()
            .string_value
        )
        # Establish subscriber
        self.publisher = self.create_publisher(WifiList, output_topic, 10)

        # Establish timer/publisher
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Set up and working")





    def timer_callback(self):

        measure = WifiMeasurement()

        measure.bssid = "11:22:33:44:55:66"
        measure.rssi = random.gauss(1,0.5)
        measure.variance = 0.5
        

        msg = WifiList()
        msg.measurements.append(measure)

        self.publisher.publish(msg)

        # self.get_logger().info(f"Tx Data: {msg}")
        






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