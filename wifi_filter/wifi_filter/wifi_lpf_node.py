import rclpy
from rclpy.node import Node

from wifi_interface.msg import WifiList, WifiMeasurement
from scipy.signal import firwin
import numpy as np




class WifiLPF(Node):

    def __init__(self):
        super().__init__('wifi_lpf')

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

        num_taps = (
            self.declare_parameter("num_taps",11)
            .get_parameter_value()
            .integer_value
        )

        fs = (
            self.declare_parameter("sampling_freq",1.0)
            .get_parameter_value()
            .double_value
        )

        fc = (
            self.declare_parameter("cutoff_freq",0.4)
            .get_parameter_value()
            .double_value
        )

        window = (
            self.declare_parameter("window","boxcar")
            .get_parameter_value()
            .string_value
        )


        # Set up filter
        self.n = num_taps
        self.taps = firwin(num_taps, fc, window=window, pass_zero=True, fs=fs)

        # Set up tracking
        self.memory = {} # dictionary where key is bssid and value is tuple of lists of past rssi and variance values
        self.index = 0 # which value is the most recent (for circular rolling buffer)

        # Establish publisher & Subscriber
        self.subscriber = self.create_subscription(WifiList, input_topic, self.input_callback, 10)
        self.publisher = self.create_publisher(WifiList, output_topic, 10)

        self.get_logger().info("Set up and working")





    def input_callback(self, msg):
        # self.get_logger().info(f"Recv Data: {msg}")

        # Read through all measurements
        new_data = {}
        for measurement in msg.measurements:
            # Init tracking of new bssid
            if measurement.bssid not in self.memory.keys():
                self.memory[measurement.bssid] = ([0]*self.n,[0]*self.n)

            # store measurement data for easy use
            new_data[measurement.bssid] = (measurement.rssi, measurement.variance)

        
        # Perform logic on all memory signals
        output = {}
        for bssid, (signal, variance) in self.memory.items():

            if bssid in new_data.keys():
                (new_signal, new_variance) = new_data[bssid]
                signal[self.index] = new_signal
                variance[self.index] = new_variance 
            else:
                signal[self.index] =  0
                variance[self.index] =  0

            output[bssid] = (sum([self.taps[i] * signal[(self.index-i) % (self.n)] for i in range(0,self.n)]),
                                 sum([self.taps[i]**2 * variance[(self.index-i) % (self.n)] for i in range(0,self.n)]))


        # Update rolling index
        self.index = (self.index + 1) % (self.n)


        # Delete noncurrent bssids
        for bssid in list(self.memory.keys()):
            (signal, variance) = self.memory[bssid]
            if all([x==0 for x in signal]) and all([x==0 for x in variance]):
                    del self.memory[bssid]


        # Create message
        timestamp = self.get_clock().now().to_msg()
        out_msg = WifiList()
        out_msg.header.frame_id = "base"
        out_msg.header.stamp = timestamp

        for bssid, (signal,variance) in output.items():
            new_measurement = WifiMeasurement()
            new_measurement.header.frame_id = "base"
            new_measurement.header.stamp = timestamp

            new_measurement.bssid = bssid
            new_measurement.rssi = int(np.clip(round(signal), 0, 255))
            new_measurement.variance = variance

            out_msg.measurements.append(new_measurement)


        # Publish
        self.get_logger().info(f"LPF Signals: {output} {self.memory}")
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