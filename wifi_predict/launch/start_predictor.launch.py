from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('wifi_predict')
    train_script = os.path.join(pkg_share, "scripts", "train_regressor.py")

    workspace_root = os.path.abspath(os.path.join(pkg_share, "..", "..", "..", ".."))  # go to workspace root
    bag_path_root = os.path.join(workspace_root, "bags")

    return LaunchDescription([
        # Step 1 — Train the model using ROSbags
        ExecuteProcess(
            cmd=["python3", train_script, bag_path_root],
            output="screen"
        ),

        # Step 2 — Start predictor after training
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package="wifi_predict",
                    executable="wifi_predictor",
                    name="wifi_predict_node",
                    output="screen"
                )
            ]
        ),
    ])
