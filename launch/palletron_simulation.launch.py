import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get the launch directory
    palletron_gazebo_dir = get_package_share_directory('palletron_gazebo')
    launch_dir = os.path.join(palletron_gazebo_dir, 'launch')

    # Get the AWS Robomaker small warehouse directory
    aws_robomaker_small_warehouse_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    
    # Get the Gazebo directory
    gazebo_ros = get_package_share_directory('gazebo_ros')

    # Launch configuration variables
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    robot_urdf = LaunchConfiguration('robot_urdf')
    pose = {'x': LaunchConfiguration('x_pose', default = '0.0'),
            'y': LaunchConfiguration('y_pose', default = '0.0'),
            'z': LaunchConfiguration('z_pose', default = '0.05'),
            'R': LaunchConfiguration('roll', default = '0.00'),
            'P': LaunchConfiguration('pitch', default = '0.00'),
            'Y': LaunchConfiguration('yaw', default = '0.00')}

    # Declare the launch arguments
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value = 'False',
        description = 'Whether to execute gzclient)'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value = os.path.join(aws_robomaker_small_warehouse_dir, 'worlds', 'no_roof_small_warehouse', 'no_roof_small_warehouse.world'),
        description = 'Full path to world model file to load'
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value = 'palletron',
        description = 'Name of the robot'
    )

    declare_robot_urdf_cmd = DeclareLaunchArgument(
        'robot_urdf',
        default_value = os.path.join(palletron_gazebo_dir, 'urdf', 'palletron.urdf.xacro'),
        description = 'Full path to robot urdf file to spawn the robot in gazebo'
    )
    
    # Specify the actions
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
    )

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition = IfCondition(PythonExpression(['not ', headless]))
    )

    start_robot_state_publisher_cmd = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen',
        parameters = [{'robot_description': ParameterValue(Command(['xacro ', robot_urdf]), value_type = str)}]
    )

    start_joint_state_publisher_cmd = Node(
        package = "joint_state_publisher",
        executable = "joint_state_publisher"
    )

    start_gazebo_spawner_cmd = Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        output = 'screen',
        arguments = [
            '-entity', robot_name,
            '-topic', '/robot_description',
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']]
    )
    
    start_rviz_cmd = Node(
       package = 'rviz2',
       executable = 'rviz2',
        arguments = [
           '-d' + os.path.join(get_package_share_directory('palletron_gazebo'), 
           'rviz', 'palletron.rviz')]
    )

    delay_rviz_after_joint_state_publisher_cmd = RegisterEventHandler(
        event_handler = OnProcessStart(
            target_action = start_joint_state_publisher_cmd,
            on_start = [
                TimerAction(period = 5.0, actions = [start_rviz_cmd])],
        )
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Parameters
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_urdf_cmd)
    
    # Actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_gazebo_spawner_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(delay_rviz_after_joint_state_publisher_cmd)

    return ld