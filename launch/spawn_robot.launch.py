from launch import LaunchContext, LaunchDescription

from launch.actions import DeclareLaunchArgument, OpaqueFunction

from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  TextSubstitution)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import yaml


def validate_pose_file(pose_file_path, robot_name):
    try:
        with open(pose_file_path) as file:
            pose_dict = yaml.load(file, Loader=yaml.loader.SafeLoader)
    except Exception as err:
        raise(err) # This will crash the launcher on purpose to stop it

    # TODO(thulioguilherme): Also validate the YAML data structure
    if robot_name in pose_dict:
        return pose_dict[robot_name]

    raise Exception('The robot name is not in the pose file.')


def launch_setup(context: LaunchContext):
    # Initialize arguments
    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')
    robot_urdf = LaunchConfiguration('robot_urdf')
    robot_optional_properties = LaunchConfiguration('robot_optional_properties')
    pose_yaml_file = LaunchConfiguration('pose_yaml_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Validate pose params file
    robot_spawn_pose = validate_pose_file(pose_yaml_file.perform(context), namespace.perform(context))

    # Put namespace on the robot description topic name
    robot_description_topic = namespace.perform(context) + '/robot_description'

    # Load robot description using Xacro
    robot_description = Command(['xacro ', robot_urdf,
                                 ' namespace:=', namespace,
                                 ' optional_properties:=', robot_optional_properties])

    # Declare nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str),
                     'use_sim_time': use_sim_time}])

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-topic', TextSubstitution(text=robot_description_topic),
            '-robot_namespace', namespace,
            '-x', TextSubstitution(text=str(robot_spawn_pose['x'])),
            '-y', TextSubstitution(text=str(robot_spawn_pose['y'])),
            '-z', TextSubstitution(text=str(robot_spawn_pose['z'])),
            '-Y', TextSubstitution(text=str(robot_spawn_pose['yaw']))])

    return [robot_state_publisher_node, joint_state_publisher_node, spawn_robot_node]


def generate_launch_description():
    declared_arguments = []

    # Declare arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='ugv1',
            description='Top-level namespace.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value='palletron',
            description='Name of the robot.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_urdf',
            default_value=PathJoinSubstitution([FindPackageShare('palletron_gazebo'),
                                                'urdf', 'palletron.xacro']),
            description='Full path to the robot URDF/XACRO file.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_optional_properties',
            default_value=PathJoinSubstitution([FindPackageShare('palletron_gazebo'),
                                                'params', 'xacro', 'default_optionals.yaml']),
            description='Full path to the yaml file with the optionals properties of the robot.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'pose_yaml_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('palletron_gazebo'), 'params', 'spawn', 'pose.yaml']),
            description='Full path to the file with the pose to spawn the robot.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Whether use simulation (Gazebo) clock.'))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
