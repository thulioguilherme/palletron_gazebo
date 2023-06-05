import yaml

from launch import LaunchDescription, LaunchContext

from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare

def launch_setup(context: LaunchContext):
    # Arguments
    spawn_config_file = LaunchConfiguration('spawn_config_file')

    # Get spawn file path as string
    spawn_config_file_path = PathJoinSubstitution(
        [FindPackageShare('palletron_gazebo'),
            'params',
            'spawn',
            spawn_config_file]).perform(context)

    # Open spawn configuration file
    with open(spawn_config_file_path) as file:
        spawn_config_dict = yaml.load(file, Loader = yaml.loader.SafeLoader)

    # Declare launches
    spawn_launch_instances = []
    for robot in spawn_config_dict.keys():
        spawn_launch_instances.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('palletron_gazebo'), 'launch', 'spawn_robot.launch.py'])),
                launch_arguments = {
                    'namespace': TextSubstitution(text = robot),
                    'robot_name': TextSubstitution(text = robot),
                    'x_pose': TextSubstitution(text = str(spawn_config_dict[robot]['pose']['x'])),
                    'y_pose': TextSubstitution(text = str(spawn_config_dict[robot]['pose']['y'])),
                    'roll': TextSubstitution(text = str(spawn_config_dict[robot]['pose']['roll'])),
                    'pitch': TextSubstitution(text = str(spawn_config_dict[robot]['pose']['pitch'])),
                    'yaw': TextSubstitution(text = str(spawn_config_dict[robot]['pose']['yaw']))
                }.items()
            )
        )

    return spawn_launch_instances

def generate_launch_description():
    declared_arguments = []

    # Declare arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'spawn_config_file',
            default_value = 'one_robot.yaml',
            description = 'Name of the file with the robots to spawn.'
        )
    )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function = launch_setup)])