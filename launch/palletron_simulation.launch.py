import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition , UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the launch directory
    palletron_gazebo_dir = get_package_share_directory('palletron_gazebo')
    launch_dir = os.path.join(palletron_gazebo_dir, 'launch')

    # Get the AWS Robomaker small warehouse directory
    aws_robomaker_small_warehouse_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    pose = {'x': LaunchConfiguration('x_pose', default = '0.0'),
            'y': LaunchConfiguration('y_pose', default = '0.0'),
            'z': LaunchConfiguration('z_pose', default = '0.32'),
            'R': LaunchConfiguration('roll', default = '0.00'),
            'P': LaunchConfiguration('pitch', default = '0.00'),
            'Y': LaunchConfiguration('yaw', default = '0.00')}
    robot_name = LaunchConfiguration('robot_name')
    robot_xacro = LaunchConfiguration('robot_xacro')
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value = '',
        description = 'Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value = 'true',
        description = 'Use simulation (Gazebo) clock if true')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value = 'True',
        description = 'Whether to start the simulator')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value = 'True',
        description = 'Whether to start the robot state publisher')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value = os.path.join(aws_robomaker_small_warehouse_dir, 'worlds', 'no_roof_small_warehouse', 'no_roof_small_warehouse.world'),
        description = 'Full path to world model file to load')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value = 'palletron',
        description = 'Name of the robot')

    declare_robot_xacro_cmd = DeclareLaunchArgument(
        'robot_xacro',
        default_value = os.path.join(palletron_gazebo_dir, 'urdf', 'palletron.gazebo.xacro'),
        description = 'Full path to robot xacro file to spawn the robot in gazebo')
    
    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value = 'False',
        description = 'Whether to execute gzclient)')

    # Specify the actions
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
    )

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition = IfCondition(PythonExpression([use_simulator, ' and not ', headless]))
    )

    urdf = os.path.join(palletron_gazebo_dir, 'urdf', 'palletron.gazebo.xacro')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        condition = IfCondition(use_robot_state_pub),
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen',
        parameters = [{'robot_description': ParameterValue(Command(['xacro ', urdf]), value_type = str)}])

    start_gazebo_spawner_cmd = Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        output = 'screen',
        arguments = [
            '-entity', robot_name,
            '-topic', '/robot_description',
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])
    
    start_rviz_cmd = Node(
       package='rviz2',
       namespace= namespace,
       executable='rviz2',
        arguments=[
           '-d' + os.path.join(get_package_share_directory('palletron_gazebo'), 
           'rviz', 'palletron_config.rviz')]
            
    )

    # start_diff_drive_spawner_cmd = Node(
    #     package="controller_manager",
    #     executable="spawner",       
    #     arguments=["diff_cont"],
    # )

    # start_joint_broad_spawner_cmd = Node(
    #     package="controller_manager",
    #     executable="spawner",     
    #     arguments=["joint_broad"],
    # )

#     declare_joint_state_publisher_cmd = Node(
#    	    package="joint_state_publisher",
#    	    executable="joint_state_publisher", 	 
#    	    condition=UnlessCondition(joint_gui)
#    )
    
#     declare_joint_state_publisher_gui_cmd = Node(
#    	    package="joint_state_publisher_gui",
#    	    executable="joint_state_publisher_gui",   
#    	    condition=IfCondition(joint_gui)
#    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)

    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_xacro_cmd)
    # ld.add_action(declare_joint_state_publisher_cmd)
    # ld.add_action(declare_joint_state_publisher_gui_cmd)
    #ld.add_action(declare_joint_gui_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_gazebo_spawner_cmd)
    ld.add_action(start_rviz_cmd)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    # ld.add_action(start_joint_broad_spawner_cmd)
    # ld.add_action(start_diff_drive_spawner_cmd)
    return ld
