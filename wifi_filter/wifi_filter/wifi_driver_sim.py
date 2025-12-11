import rclpy
from rclpy.node import Node

from wifi_interface.msg import WifiList, WifiMeasurement
import serial
import time
import scipy as sp
import random
import numpy as np




class DriverSim(Node):

    def __init__(self):
        super().__init__('driver_sim')

        self.get_logger().info("Starting up")

        output_topic = (
            self.declare_parameter("output_topic","/wifi")
            .get_parameter_value()
            .string_value
        )

        self.numSignals = (
            self.declare_parameter("number_signals",3)
            .get_parameter_value()
            .integer_value
        )

        # Establish subscriber
        self.publisher = self.create_publisher(WifiList, output_topic, 10)

        # Establish timer/publisher
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Set up and working")

        self.starting = True





    def timer_callback(self):

        msg = WifiList()
        # measure = WifiMeasurement()
        # measure.bssid = "11:22:33:44:55:66"
        # measure.rssi = random.gauss(1,0.5)
        # measure.variance = 0.5
        # msg.measurements.append(measure)


        # Test calculation
        for i in range(self.numSignals):
            measure = WifiMeasurement()

            measure.bssid = "11:22:33:44:55:" + f"{i:0{2}}"
            measure.rssi = int(np.clip(round(100*random.gauss(1,0.5)), 0, 100))
            measure.variance = 0.5
            

            
            msg.measurements.append(measure)



        # Test forgetting old BSSIDs
        if self.starting:
            measure = WifiMeasurement()

            measure.bssid = "00:00:00:00:00"
            measure.rssi = 1
            measure.variance = 10.0

            msg.measurements.append(measure)
            self.starting = False



        self.publisher.publish(msg)

        self.get_logger().info(f"Tx RSSI: {msg.measurements}")
        






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