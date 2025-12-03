from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    #======================# Config Files #======================#

    # config_file = os.path.join(
    #     get_package_share_directory('wifi_filter'),
    #     'config',
    #     'kf_config.yaml'
    # )



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



    #======================# Sub Launches #======================#

    sensor_launch_file = os.path.join(
        get_package_share_directory('wifi_driver'),
        'launch',
        'driver.launch.py'
    )

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensor_launch_file),
        launch_arguments={
            #'port': LaunchConfiguration('vn_port')
        }.items()
    )

    ld.add_action(sensor_launch)




    lpf_launch_file = os.path.join(
        get_package_share_directory('wifi_filter'),
        'launch',
        'lpf.launch.py'
    )

    lpf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lpf_launch_file),
        launch_arguments={
            #'port': LaunchConfiguration('vn_port')
        }.items()
    )

    ld.add_action(lpf_launch)





    # pred_launch_file = os.path.join(
    #     get_package_share_directory('wifi_predict'),
    #     'launch',
    #     'predict.launch.py'
    # )

    # pred_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(pred_launch_file),
    #     launch_arguments={
    #         #'port': LaunchConfiguration('vn_port')
    #     }.items()
    # )

    # ld.add_action(pred_launch)




    kf_launch_file = os.path.join(
        get_package_share_directory('wifi_filter'),
        'launch',
        'kf.launch.py'
    )

    kf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(kf_launch_file),
        launch_arguments={
            #'port': LaunchConfiguration('vn_port')
        }.items()
    )

    ld.add_action(kf_launch)






    #======================# Return #======================#

    return ld