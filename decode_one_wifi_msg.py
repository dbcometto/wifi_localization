import rosbag2_py
from rclpy.serialization import deserialize_message

# Import message type
from wifi_interface.msg import WifiList

bag_path = "bags/0_0_0_11-17"    # folder containing the .mcap file

storage_options = rosbag2_py.StorageOptions(
    uri=bag_path,
    storage_id="mcap"
)

converter_options = rosbag2_py.ConverterOptions(
    input_serialization_format="cdr",
    output_serialization_format="cdr"
)

reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

# Just read until we get a /wifi message
while reader.has_next():
    topic, data, t = reader.read_next()

    if topic == "/wifi":
        msg = deserialize_message(data, WifiList)
        print("\n=========== DECODED WifiList MESSAGE ===========")
        print(msg)
        print("================================================")
        break