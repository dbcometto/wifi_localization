from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    #======================# Config Files #======================#

    default_config_file = os.path.join(
        get_package_share_directory('wifi_filter'),
        'config',
        'default_kf_config.yaml'
    )



    #======================# Launch Arguments #======================#

    # la_input_topic = DeclareLaunchArgument(
    #     'input_topic', 
    #     default_value = '/pose_estimate',
    #     description = "Input topic to filter"
    # )
    # ld.add_action(la_input_topic)

    # la_output_topic = DeclareLaunchArgument(
    #     'output_topic', 
    #     default_value = '/pose',
    #     description = "Output topic from filter"
    # )
    # ld.add_action(la_output_topic)



    #======================# Nodes #======================#

    lpf_node = Node(
            package='wifi_filter',
            executable='kf_node',
            name='wifi_kf',
            output='screen',
            parameters=[default_config_file, {
                # 'input_topic': LaunchConfiguration('input_topic'),
                # 'output_topic': LaunchConfiguration('output_topic'),
            }]
        )
    ld.add_action(lpf_node)



    #======================# Return #======================#

    return ld
