import rclpy
from rclpy.node import Node

from wifi_interface.msg import WifiList, WifiMeasurement
from queue import Queue
import serial
import time
import subprocess
from math import floor


class Network_Info():
    """
    Class for storing info on a network's RSSI values as they are measured multiple times
    """
    def __init__(self, initial_measurement):
        self.measurements = 0
        self.total = -1
        self.rssis = Queue(maxsize=5)
        self.curr_rssi = -1
        self.variance = -1
        self.active = False

        for i in range(5):
            self.rssis.put(-1)
        self.update_info(initial_measurement)

    def _calc_variance(self):
        """
        Calculates the variance of the given rssis queue
        """
        if self.measurements > 0:
            if self.measurements > 1:
                diffs = 0
                for i in range(5):
                    rssi = self.rssis.get()
                    if rssi != -1:
                        diffs += (rssi - (self.total / self.measurements)) ** 2
                    self.rssis.put(rssi)
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

        # Update the values of the network info based on the new
        # rssi measurement as well as the one that is being bumped
        # for being too old
        if out_val > 0 and rssi_measurement > 0:
            self.total += (rssi_measurement - out_val)
            self.active = True
        elif out_val > 0 and rssi_measurement < 0:
            self.total -= out_val
            self.measurements -= 1
            self.active = False
        elif out_val < 0 and rssi_measurement > 0:
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
        Tuple[int, float]
            Tuple containing the current measured RSSI, and the measured variance
        """
        return (self.curr_rssi, self.variance)
            
    
class WiFi_Manager():
    """
    Class responsible for scanning Wi-Fi networks and maintaining info on these networks for the life of the node
    """
    def __init__(self):
        self.wifi_bssids = {}
        self.networks = 0

    def scan_for_networks(self):
        """
        Scans the available Wi-Fi networks and updates the internal list of networks by adding new networks, updating existing ones, and purging unresponsive networks
        """
        # Execute Wi-Fi network scan using the nmcli tool on Linux
        formatted_cmd = "/usr/bin/nmcli dev wifi list --rescan yes".split()
        formatted_output = subprocess.Popen(formatted_cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT).stdout.read().decode('utf-8').split("\n")
        header = formatted_output[0]
        network_list = formatted_output[1:-1]

        # Since Wi-Fi network names are unpredictable, use the spacing of the header
        # message returned from the scan to identify boundaries where substrings can
        # be taken to get the different parts of the scan output
        section_boundaries = [0]
        for i in range(len(header)-1):
            if header[i] == " " and header[i+1] != " ":
                section_boundaries.append(i+1)
        section_boundaries.append(len(header))
    
        # Extract the bssids and rssi values from the scan results
        scan_results = {}
        for network in network_list:
            bssid = network[section_boundaries[1]:section_boundaries[2]].rstrip()
            rssi  = int(network[section_boundaries[6]:section_boundaries[7]].rstrip())
            scan_results[bssid] = rssi
        
        # Go through the current list of bssids we have stored, if found in
        # the latest scan, update their value and remove them from the scan.
        # If not, update with a no-response. If 5 no-responses, remove them
        # from the internal list
        unresponsive_networks = []
        for bssid in self.wifi_bssids.keys():
            if bssid in scan_results.keys():
                self.wifi_bssids[bssid].update_info(scan_results[bssid])
                del scan_results[bssid]
            else:
                self.wifi_bssids[bssid].update_info(-1)
                if self.wifi_bssids[bssid].is_unresponsive():
                    unresponsive_networks.append(bssid)
        for bssid in unresponsive_networks:
            del self.wifi_bssids[bssid]
            self.networks -= 1

        # Go through all the Wi-Fi networks we have never seen before and
        # add them to the internal list
        for bssid in scan_results.keys():
            self.wifi_bssids[bssid] = Network_Info(scan_results[bssid])
            self.networks += 1

    def get_active_networks(self):
        """
        Gets a list of the active networks from the managed Wi-Fi network list

        Returns
        -------
        list[Tuple[string, int, float]]
            List of tuples containing a Wi-Fi BSSID and corresponding RSSI measurement and variance
        """
        # Looks through internal list of Wi-Fi networks and gets
        # networks that are active to add to return
        active_networks = []
        for bssid in self.wifi_bssids.keys():
            if self.wifi_bssids[bssid].is_active():
                rssi, var = self.wifi_bssids[bssid].get_info()
                active_networks.append((bssid, rssi, var))

        return active_networks


class WifiDriver(Node):
    """
    Wi-Fi Driver node for ROS2 to scan active Wi-Fi network BSSIDs and RSSI measurements and publish on '/wifi' topic
    """
    def __init__(self):
        super().__init__('WifiDriver')

        self.get_logger().info("Starting Wi-Fi driver node")
        # Setup publisher
        self._publisher = self.create_publisher(WifiList, 'wifi', 10)

        # Establish timer and associated callback
        timer_period = 5.0 # seconds
        self._timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize Wifi_Manager instance
        self._wifi_manager = WiFi_Manager()

        self.get_logger().info("Wi-Fi driver initialized")

    def timer_callback(self):
        """
        Uses the class's instance of a WiFiManager to scan broadcasting Wi-Fi networks and construct and publish a WifiList message to the '/wifi' topic
        """

        # Scan for broadcasting Wi-Fi networks and get list of networks that responded
        self._wifi_manager.scan_for_networks()
        responses = self._wifi_manager.get_active_networks()

        # Create list of WiFiMeasurement sub-messages
        measurements = []
        for response in responses:
            measurements.append(WifiMeasurement(bssid=str(response[0]),rssi=int(response[1]),variance=float(response[2])))

        # Create overall WifiList message
        msg = WifiList()

        epoch_time_nsec = time.time_ns()
        seconds = int(floor(epoch_time_nsec / 1_000_000_000))
        nsecs   = int(epoch_time_nsec % 1_000_000_000)

        msg.header.stamp.sec = seconds
        msg.header.stamp.nanosec = nsecs
        msg.header.frame_id = "wifi_frame"

        msg.networks = len(measurements)
        msg.measurements = measurements

        # Publish constructed WiFiList message
        self._publisher.publish(msg)


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
