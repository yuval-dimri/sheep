import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

pkg_path = os.path.join(
    get_package_share_directory('sheep-sim'))

robot_xacro_file_path = os.path.join(pkg_path, 'urdf', 'sheep_basic.xacro')


gazebo_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    launch_arguments={'world': 'worlds/willowgarage.world'}.items()
)


doc = xacro.parse(open(robot_xacro_file_path))
xacro.process_doc(doc)
rsp_params = {'robot_description': doc.toxml()}
node_robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[rsp_params]
)

node_spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                         arguments=['-topic', 'robot_description',
                                    '-entity', 'cartpole'],
                         output='screen')

node_joint_state_controller = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
         'joint_state_broadcaster'],
    output='screen'
)

node_joint_trajectory_controller = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
         'diff_drive_base_controller'],
    output='screen'
)


def generate_launch_description():
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=node_spawn_entity,
                on_exit=[node_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=node_joint_state_controller,
                on_exit=[node_joint_trajectory_controller],
            )
        ),
        gazebo_launch,
        node_robot_state_publisher,
        node_spawn_entity,
    ])
