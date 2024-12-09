import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import serialize_message, deserialize_message
from nav_msgs.msg import Odometry
import shutil
import os

def modify_odometry_bag(input_bag_path, output_bag_path):
    
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag_path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', '')
    )

    if os.path.exists(output_bag_path):
        shutil.rmtree(output_bag_path)
        print(f"Deleted existing directory: {output_bag_path}")
        
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=output_bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", "")
    )

    
    metadata = reader.get_metadata()
    topic_type_map = {topic_info.topic_metadata.name: topic_info.topic_metadata.type for topic_info in metadata.topics_with_message_count}
    print(topic_type_map)
    
    for topic_info in metadata.topics_with_message_count:
        writer.create_topic(rosbag2_py._storage.TopicMetadata(
            name=topic_info.topic_metadata.name,
            type=topic_info.topic_metadata.type,
            serialization_format="cdr"
        ))

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = topic_type_map.get(topic)

        
        if msg_type == "nav_msgs/msg/Odometry":
            odometry_msg = deserialize_message(data, Odometry)
            odometry_msg.header.frame_id = "world"
            odometry_msg.child_frame_id = "base"
            data = serialize_message(odometry_msg)
            writer.write(topic, data, timestamp)
        elif topic == '/camera/camera/depth/color/points':
            writer.write(topic, data, timestamp)

    print(f"Modified bag saved to: {output_bag_path}")


input_bag_path = "/home/user/Test/analysis_flight_test/0904_/0904_1"
output_bag_path = "/home/user/Test/analysis_flight_test/0904_/0904_1_mod"
modify_odometry_bag(input_bag_path, output_bag_path)
