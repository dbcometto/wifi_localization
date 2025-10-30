from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    raise NotImplementedError
    # return LaunchDescription([
    #     DeclareLaunchArgument(
    #     'port',
    #     default_value="/dev/pts/3"
    # ),
    #     Node(
    #         package='vn_driver',
    #         executable='vn_driver',
    #         name='vn_driver',
    #         output='screen',
    #         parameters=[{
    #             "port": LaunchConfiguration('port')
    #         }]
    #     ),
    # ])
