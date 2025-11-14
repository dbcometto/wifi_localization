import rclpy
from rclpy.node import Node

from wifi_interface.msg import WifiList, WifiMeasurement
from queue import Queue
import serial
import time
import scipy as sp


class Network_Info():
    """
    Class for storing info on a network's RSSI values as they are measured multiple times
    """

    def __init__(self):
        measurements = 0
        total = -1
        rssis = Queue(maxsize=5)
        curr_rssi = -1
        variance = -1
        active = False

    def _calc_variance(self):
        """
        Calculates the variance of the given rssis queue
        """

        if self.measurements > 0:
            if self.measurements > 1:
                diffs = 0
                for rssi in rssis:
                    if rssi != -1:
                        diffs += (rssi - (self.total / self.measurements)) ** 2
                self.variance = diffs / self.measurements
            else:
                self.variance = 0
        else:
            self.variance = -1

    def update_info(self, rssi_measurement=-1):
        """
        Updates the attributes of the `Network_Info` class given a new rssi measurement

        Parameters
        ----------
        rssi_measurement : int, default=-1
            RSSI measurement to be added to the given `Network_Info` class
        """
            
        out_val = self.rssis.get()

        if out_val > 0 and rssi_measurement > 0:
            self.total += (rssi_measurement - out_val)
            self.active = True
        else if out_val > 0 and rssi_measurement < 0:
            self.total -= out_val
            self.measurements -= 1
            self.active = False
        else if out_val < 0 and rssi_measurement > 0:
            self.total += rssi_measurement
            self.measurements += 1
            self.active = True
        else:
            self.active = False
   
        self.rssis.put(rssi_measurement)
        self.curr_rssi = rssi_measurement

        self._calc_variance()

    def is_active(self):
        """
        Returns if the network is currently active (meaning the last RSSI measurement is valid)

        Returns
        -------
        bool
            True if the network is active
        """

        return self.active

    def is_unresponsive(self):
        """
        Returns if the network is currently unresponsive (meaning the last 5 RSSI measurements are invalid)

        Returns
        -------
        bool
            True if the network is unresponsive
        """

        return (self.measurements == 0)

    def get_info(self):
        """
        Gets a useful status of the current network info

        Returns
        -------
        Tuple[bool, int, float]
            Tuple containing whether the network is active or not, the current measured RSSI, and the measured variance
        """

        return (self.active, self.curr_rssi, self.variance
            
    
class WiFi_Manager():

    def __init__(self):
        wifi_bssids = {}
        networks = 0

    def scan_for_networks(self):
        pass

    def purge_unresponsive_networks(self):



class WifiDriver(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        

        # self.get_logger().info("Starting up!")

        # pub_topic = (
        #     self.declare_parameter("pub_topic","/imu")
        #     .get_parameter_value()
        #     .string_value
        # )

        # port = (
        #     self.declare_parameter("port","/dev/pts/8")
        #     .get_parameter_value()
        #     .string_value
        # )

        # # Open serial port
        # self.connected = False
        # while not self.connected:
        #     try:
        #         # self.get_logger().info("Trying to connect to port")
        #         self.serial = serial.Serial(port, 115200, timeout=0.1)
        #         self.connected = True
        #         self.get_logger().info("Connected to port")
        #     except Exception as e:
        #         self.get_logger().info("Failed to open port... trying again in 2 sec")
        #         time.sleep(2)


        # # Set IMU settings
        # self.serial.write(b"$VNWRG,06,14*59\n")
        # self.serial.write(b"$VNWRG,07,40*59\n")

        # # Establish timer/publisher
        # timer_period = 0.01  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        # self.publisher = self.create_publisher(Vectornav, pub_topic, 10)

        # self.get_logger().info("Set up and working!")





    def timer_callback(self):
        pass

        # self.get_logger().info("Timer CB")
        # while self.serial.in_waiting > 0:
        #     serialRead = self.serial.readline().decode('utf-8')
        #     # self.get_logger().info(f"Reading: {serialRead}")

        #     data = self.processString(serialRead)
        #     # self.get_logger().info(f"Data: {data}")

        #     if data:
        #         msg = self.make_message(data)

        #         msg.raw_string = serialRead

        #         self.publisher.publish(msg)
        #         self.get_logger().info(f"Publishing: {msg}")








def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = WifiDriver()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        print("Shutting down!")

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
