import rosbag2_py
from rclpy.serialization import deserialize_message

bag_path = "bags/0_0_0_11-17"

storage_options = rosbag2_py.StorageOptions(
    uri=bag_path,
    storage_id='mcap'
)

converter_options = rosbag2_py.ConverterOptions(
    input_serialization_format='cdr',
    output_serialization_format='cdr'
)

reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

# Print the topics + types
print("Topics in bag:")
topics = reader.get_all_topics_and_types()
for t in topics:
    print("  ", t.name, "→", t.type)

# Try reading one message
reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

while reader.has_next():
    topic, data, t = reader.read_next()
    print("\nRaw topic:", topic)
    print("Raw data size:", len(data))
    break
