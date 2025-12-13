from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers.on_process_exit import OnProcessExit
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # --- Paths ---
    pkg_share = get_package_share_directory("wifi_predict")

    # Training script inside wifi_predict/scripts/
    train_script = os.path.join(pkg_share, "scripts", "train_regressor.py")

    # Project workspace bags directory (one level above src/)
    workspace_root = os.path.abspath(os.path.join(pkg_share, "..", "..", "..", ".."))
    bags_root = os.path.join(workspace_root, "bags/data_collection_2")

    # # --- Nodes ---
    # # 1. WiFi driver node (optional)
    # wifi_driver_node = Node(
    #     package="wifi_driver",         # <--- change to your driver package name
    #     executable="wifi_driver",     # <--- change to your driver executable
    #     name="wifi_driver",
    #     output="screen"
    # )

    # 2. Predictor node (only launched *after* training completes)
    predictor_node = Node(
        package="wifi_predict",
        executable="wifi_predictor",
        name="wifi_predict_node",
        output="screen"
    )

    # 3. Training process
    train_process = ExecuteProcess(
        cmd=["python3", "-u", train_script, bags_root],         # unbuffered stdout for real-time logging
        output="screen"
    )

    # Start predictor only *after training finishes*
    start_predictor_after_training = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=train_process,
            on_exit=[predictor_node]
        )
    )

    return LaunchDescription([
        # wifi_driver_node,                 # start your wifi driver
        train_process,                    # run training (blocking)
        start_predictor_after_training    # start predictor after training
    ])