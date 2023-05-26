from launch import LaunchDescription, LaunchContext

from launch.actions import OpaqueFunction, DeclareLaunchArgument

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from nav2_common.launch import ReplaceString

def launch_setup(context: LaunchContext):
    # Arguments
    namespace = LaunchConfiguration('namespace')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    # Get the full path to the Rviz config file
    rviz_config_file_path = PathJoinSubstitution([FindPackageShare('palletron_gazebo'),
        'rviz',
        rviz_config_file])

    # Replace robot namespace in Rviz file
    namespaced_rviz_config_file = ReplaceString(
        source_file = rviz_config_file_path,
        replacements = {'<robot_namespace>': ('/', namespace)})

    # Declare nodes
    rviz2_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        namespace = namespace,
        arguments = ['-d', namespaced_rviz_config_file]
    )

    return [rviz2_node]

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
            'rviz_config_file',
            default_value = 'minimal_robot_view.rviz',
            description = 'Name of the Rviz configuration file to use.'
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function = launch_setup)])