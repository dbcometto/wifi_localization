from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('wifi_predict')
    build_script = os.path.join(pkg_share, 'scripts', 'build_fingerprint_db.py')

    return LaunchDescription([
        # Step 1: Build fingerprint DB
        ExecuteProcess(
            cmd=["python3", build_script],
            output="screen"
        ),

        # Step 2: Start predictor node after a short delay
        TimerAction(
            period=0.5,  # wait 0.5s for DB to be ready
            actions=[Node(
                package="wifi_predict",
                executable="wifi_predictor",
                name="wifi_predict_node",
                output="screen"
            )]
        ),
    ])
