import rclpy
from rclpy.node import Node

from vn_interface.msg import Vectornav
import serial
import time
import scipy as sp




class VNDriver(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self.get_logger().info("Starting up!")

        pub_topic = (
            self.declare_parameter("pub_topic","/imu")
            .get_parameter_value()
            .string_value
        )

        port = (
            self.declare_parameter("port","/dev/pts/8")
            .get_parameter_value()
            .string_value
        )

        # Open serial port
        self.connected = False
        while not self.connected:
            try:
                # self.get_logger().info("Trying to connect to port")
                self.serial = serial.Serial(port, 115200, timeout=0.1)
                self.connected = True
                self.get_logger().info("Connected to port")
            except Exception as e:
                self.get_logger().info("Failed to open port... trying again in 2 sec")
                time.sleep(2)


        # Set IMU settings
        self.serial.write(b"$VNWRG,06,14*59\n")
        self.serial.write(b"$VNWRG,07,40*59\n")

        # Establish timer/publisher
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.publisher = self.create_publisher(Vectornav, pub_topic, 10)

        self.get_logger().info("Set up and working!")





    def timer_callback(self):
        # self.get_logger().info("Timer CB")
        while self.serial.in_waiting > 0:
            serialRead = self.serial.readline().decode('utf-8')
            # self.get_logger().info(f"Reading: {serialRead}")

            data = self.processString(serialRead)
            # self.get_logger().info(f"Data: {data}")

            if data:
                msg = self.make_message(data)

                msg.raw_string = serialRead

                self.publisher.publish(msg)
                self.get_logger().info(f"Publishing: {msg}")



    def processString(self,readString,type="$VNYMR"):
        data = readString.split('*')[0].split(',')
        # self.get_logger().info(f"Split: {data}")

        if data[0]==type:
            output = {}

            output["Yaw_deg"] = float(data[1][1:]) if data[1][0]=='+' else -float(data[1][1:])
            output["Pitch_deg"] = float(data[2][1:]) if data[2][0]=='+' else -float(data[2][1:])
            output["Roll_deg"] = float(data[3][1:]) if data[3][0]=='+' else -float(data[3][1:])

            output["MagX_gauss"] = float(data[4][1:]) if data[4][0]=='+' else -float(data[4][1:])
            output["MagY_gauss"] = float(data[5][1:]) if data[5][0]=='+' else -float(data[5][1:])
            output["MagZ_gauss"] = float(data[6][1:]) if data[6][0]=='+' else -float(data[6][1:])

            output["AccelX_mps2"] = float(data[7][1:]) if data[7][0]=='+' else -float(data[7][1:])
            output["AccelY_mps2"] = float(data[8][1:]) if data[8][0]=='+' else -float(data[8][1:])
            output["AccelZ_mps2"] = float(data[9][1:]) if data[9][0]=='+' else -float(data[9][1:])

            output["AngX_radps"] = float(data[10][1:]) if data[10][0]=='+' else -float(data[10][1:])
            output["AngY_radps"] = float(data[11][1:]) if data[11][0]=='+' else -float(data[11][1:])
            output["AngZ_radps"] = float(data[12][1:]) if data[12][0]=='+' else -float(data[12][1:])

            return output



    def make_message(self,data):
        msg = Vectornav()

        frame = "imu1_Frame"
        sec = int(time.time_ns() // 1e9)
        nsec = int(time.time_ns() % 1e9)

        msg.header.frame_id = frame
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nsec

        # IMU

        msg.imu.header.frame_id = frame
        msg.imu.header.stamp.sec = sec
        msg.imu.header.stamp.nanosec = nsec

        r = sp.spatial.transform.Rotation.from_euler('zyx',[data["Yaw_deg"],data["Pitch_deg"],data["Roll_deg"]],degrees=True)
        q = r.as_quat()
        # r = sp.spatial.transform.Rotation.from_euler('zyx',[45, 30, 60],degrees=True)
        # self.get_logger().info(f"{r.as_quat()} | {r.as_euler('zyx',degrees=True)}")
        msg.imu.orientation.x = q[0]
        msg.imu.orientation.y = q[1]
        msg.imu.orientation.z = q[2]
        msg.imu.orientation.w = q[3]

        msg.imu.angular_velocity.x = data["AngX_radps"]
        msg.imu.angular_velocity.y = data["AngY_radps"]
        msg.imu.angular_velocity.z = data["AngZ_radps"]

        msg.imu.linear_acceleration.x = data["AccelX_mps2"]
        msg.imu.linear_acceleration.y = data["AccelY_mps2"]
        msg.imu.linear_acceleration.z = data["AccelZ_mps2"] 

        # Magnetic Field

        msg.mag_field.header.frame_id = frame
        msg.mag_field.header.stamp.sec = sec
        msg.mag_field.header.stamp.nanosec = nsec

        gauss2tesla = 1/10000
        msg.mag_field.magnetic_field.x = data["MagX_gauss"]*gauss2tesla
        msg.mag_field.magnetic_field.y = data["MagY_gauss"]*gauss2tesla
        msg.mag_field.magnetic_field.z = data["MagZ_gauss"]*gauss2tesla


        return msg





def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = VNDriver()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        print("Shutting down!")

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()