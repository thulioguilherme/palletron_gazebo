import yaml

from launch import LaunchDescription, LaunchContext

from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare


def launch_setup(context: LaunchContext):
    # Initialize arguments
    spawn_params_file = LaunchConfiguration('spawn_params_file')

    # Get spawn file path as string
    spawn_param_file_path = spawn_params_file.perform(context)

    # Open spawn configuration file
    with open(spawn_param_file_path) as file:
        spawn_params_dict = yaml.load(file, Loader=yaml.loader.SafeLoader)

    # Declare launches
    spawn_launch_instances = []
    for robot in spawn_params_dict.keys():
        spawn_launch_instances.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('palletron_gazebo'),
                         'launch',
                         'spawn_robot.launch.py'])),
                launch_arguments={
                    'namespace': TextSubstitution(text=robot),
                    'robot_name': TextSubstitution(text=robot),
                    'x_pose': TextSubstitution(text=str(spawn_params_dict[robot]['pose']['x'])),
                    'y_pose': TextSubstitution(text=str(spawn_params_dict[robot]['pose']['y'])),
                    'z_pose': TextSubstitution(text=str(spawn_params_dict[robot]['pose']['z'])),
                    'roll': TextSubstitution(text=str(spawn_params_dict[robot]['pose']['roll'])),
                    'pitch': TextSubstitution(text=str(spawn_params_dict[robot]['pose']['pitch'])),
                    'yaw': TextSubstitution(text=str(spawn_params_dict[robot]['pose']['yaw']))
                }.items())
        )

    return spawn_launch_instances


def generate_launch_description():
    declared_arguments = []

    # Declare arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'spawn_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('palletron_gazebo'),
                 'params',
                 'spawn',
                 'two_robots.yaml']),
            description='Full path to the file with the robots to spawn.'))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
