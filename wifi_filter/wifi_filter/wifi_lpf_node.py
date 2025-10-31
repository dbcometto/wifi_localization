import rclpy
from rclpy.node import Node

# from vn_interface.msg import Vectornav
from wifi_interface.msg import WifiList # temp
import serial
import time
import scipy as sp
import numpy as np




class WifiLPF(Node):

    def __init__(self):
        super().__init__('node')

        self.get_logger().info("Starting up")

        input_topic = (
            self.declare_parameter("input_topic","/raw_wifi_data")
            .get_parameter_value()
            .string_value
        )

        output_topic = (
            self.declare_parameter("output_topic","/filtered_wifi_data")
            .get_parameter_value()
            .string_value
        )

        # Establish publisher & Subscriber
        self.subscriber = self.create_subscription(WifiList, input_topic, self.input_callback, 10)
        self.publisher = self.create_publisher(WifiList, output_topic, 10)

        # TODO: parametrize this and have it auto generate taps
        self.taps = [1,1,1,1]
        self.n = len(self.taps)

        self.memory = {} # dictionary where key is bssid and value is list of past values
        self.index = 0 # which value is the most recent

        self.get_logger().info("Set up and working")





    def input_callback(self, msg):
        # self.get_logger().info(f"Recv Data: {msg}")

        # Maybe get rid of this later
        new_data = {}
        
        # Read through measurements
        for measurement in msg.measurements:
            # Init new bssid
            if measurement.bssid not in self.memory.keys():
                self.memory[measurement.bssid] = [0]*self.n

            # store measurement data for easy use
            new_data[measurement.bssid] = measurement.rssi

        output = {}
        # Perform logic on all memory signals
        for bssid, signal in self.memory.items():

            signal[self.index] = new_data[bssid] if new_data[bssid] else 0

            if np.all(signal==0):
                del self.memory[bssid]
            else:
                output[bssid] = sum([signal[(self.index+i) % self.n] for i in range(0,self.n-1)])

            self.index += 1


        self.get_logger().info(f"LPF Signals: {output}")



        out_msg = msg

        self.publisher.publish(out_msg)
        






def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = WifiLPF()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        print("Shutting down!")

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()