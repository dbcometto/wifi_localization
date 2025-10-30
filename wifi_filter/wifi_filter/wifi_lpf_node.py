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
        self.taps = np.array([1])
        self.n = len(self.taps)

        self.memory = {}

        self.get_logger().info("Set up and working")





    def input_callback(self, msg):
        # self.get_logger().info(f"Recv Data: {msg}")
        new_bssids = [x.bssid for x in msg.measurements]

        for bssid, signal in self.memory.items():
            

            if bssid in new_bssids:
                measure_list.append(measurement.rssi)
                measure_list.pop(0)
                

                # signal = np.array(measure_list)
                
                # out_signal = np.convolve(signal,self.taps)
                self.get_logger().info(f"Measure List: {measure_list}")


        



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