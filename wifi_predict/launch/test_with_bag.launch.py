from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('wifi_predict')
    build_script = os.path.join(pkg_share, 'scripts', 'build_fingerprint_db.py')

    workspace_root = os.getcwd()
    bag_path_root = os.path.join(workspace_root, "bags")  # root folder containing all position folders

    return LaunchDescription([
        # 1️⃣ Build fingerprint DB (pass external bags folder)
        ExecuteProcess(
            cmd=["python3", build_script, bag_path_root],
            output="screen"
        ),

        # 2️⃣ Start predictor node after short delay
        TimerAction(
            period=1.0,  # allow DB build to finish
            actions=[Node(
                package="wifi_predict",
                executable="wifi_predictor",
                name="wifi_predict_node",
                output="screen",
                parameters=[{"use_sim_time": True}]
            )]
        ),

        # 3️⃣ Play the rosbag for the first position after node is spinning
        TimerAction(
            period=2.0,
            actions=[ExecuteProcess(
                cmd=["ros2", "bag", "play", os.path.join(bag_path_root, "0_0_0_11-17", "0_0_0_11-17_0.mcap"), "--clock"],
                output="screen"
            )]
        ),
    ])
