from launch import LaunchDescription, LaunchContext

from launch.actions import OpaqueFunction, DeclareLaunchArgument

from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration, Command

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def launch_setup(context: LaunchContext):
    # Arguments
    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')
    robot_urdf = LaunchConfiguration('robot_urdf')
    robot_optional_properties = LaunchConfiguration('robot_optional_properties')
    pose = {'x': LaunchConfiguration('x_pose', default = '0.0'),
            'y': LaunchConfiguration('y_pose', default = '0.0'),
            'z': LaunchConfiguration('z_pose', default = '0.05'),
            'R': LaunchConfiguration('roll', default = '0.00'),
            'P': LaunchConfiguration('pitch', default = '0.00'),
            'Y': LaunchConfiguration('yaw', default = '0.00')}
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 
    robot_topic = namespace.perform(context) + '/robot_description'

    # Get full path to the robot URDF file
    robot_urdf_path = PathJoinSubstitution(
        [FindPackageShare('palletron_gazebo'), 'urdf', robot_urdf])
    
    # Get full path to the robot optional properties file
    robot_optional_properties_path = PathJoinSubstitution(
        [FindPackageShare('palletron_gazebo'), 'params', 'xacro', robot_optional_properties])

    # Load robot description using Xacro
    robot_description = Command(['xacro ', robot_urdf_path,
        ' namespace:=', namespace,
        ' optional_properties:=', robot_optional_properties_path])

    # Declare nodes
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        namespace = namespace,
        output = 'screen',
        parameters = [{'robot_description': ParameterValue(robot_description, value_type = str),
            'use_sim_time' : use_sim_time}]
    )

    spawn_robot_node = Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        name = 'spawn_entity',
        output = 'screen',
        arguments = [
            '-entity', robot_name,
            '-topic', TextSubstitution(text = robot_topic),
            '-robot_namespace', namespace,
            '-x', pose['x'],
            '-y', pose['y'],
            '-z', pose['z'],
            '-R', pose['R'],
            '-P', pose['P'],
            '-Y', pose['Y']
        ]
    )

    return [robot_state_publisher_node, spawn_robot_node]

def generate_launch_description():
    declared_arguments = []

    # Declare arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value = 'palletron1',
            description = 'Top-level namespace.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value = 'palletron1',
            description = 'Name of the robot.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_urdf',
            default_value = 'palletron.xacro',
            description = 'Robot URDF/XACRO file.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_optional_properties',
            default_value = 'default_optionals.yaml',
            description = 'File with the optionals properties of the robot.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value = 'True',
            description = 'Whether use simulation time.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'x_pose',
            default_value = '0.00',
            description = 'The x-component of the initial position (meters).'
        ) 
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'y_pose',
            default_value = '0.00',
            description = 'The y-component of the initial position (meters).'
        ) 
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'roll',
            default_value = '0.00',
            description = 'The roll angle of the initial position (radians).'
        ) 
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'pitch',
            default_value = '0.00',
            description = 'The pitch angle of the initial position (radians).'
        ) 
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'yaw',
            default_value = '0.00',
            description = 'The yaw angle of the initial position (radians).'
        ) 
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function = launch_setup)])
