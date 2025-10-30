import rclpy
from rclpy.node import Node

# from vn_interface.msg import Vectornav
import serial
import time
import scipy as sp




class WifiKF(Node):

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
        # # self.get_logger().info("Timer CB")
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

    minimal_publisher = WifiKF()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        print("Shutting down!")

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()