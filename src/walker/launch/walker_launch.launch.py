import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_dir = get_package_share_directory('walker')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')


    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_room.wbt')
    )

    my_robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    record_arg = DeclareLaunchArgument(
        'record',
        default_value='false',
        description='Enable bag record'
    )

    bag_recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record'))
    )
    
    walker_node = Node(
        package='walker',
        executable='walker_node',
    )


    return LaunchDescription([
        record_arg,
        webots,
        my_robot_driver,
        bag_recorder,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])