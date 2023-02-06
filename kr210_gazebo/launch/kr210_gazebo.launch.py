from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # CONSTANTS
    package = 'kr210_gazebo'
    package_description = 'kr210_description'
    world_name = 'cafe.world'
    pause_gazebo = 'false'
    robot_name = 'kr210'
    robot_xacro = 'kr210.urdf.xacro' # the robot xacro file

    # PATHS
    world_name_path = os.path.join(get_package_share_directory(package), 'worlds', world_name)

    package_path = os.path.join(get_package_share_directory(package_description))
    xacro_file = os.path.join(package_path, 'urdf', robot_xacro)

    # load urdf    
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = doc.toxml()


    # ARGUMENT DECLARATIONS
    arg_world_name = DeclareLaunchArgument(name='world_name', default_value=world_name_path, description='path to the world file')
    arg_paused = DeclareLaunchArgument(name='paused', default_value=pause_gazebo, description='pause gazebo physics')    
    arg_use_sim_time = DeclareLaunchArgument(name='use_sim_time', default_value='false', description='Use simulation time [true, false]')

    # ENV VARIABLES
    
    # LAUNCH CONFIGURATION
    launch_cfg_world_name = LaunchConfiguration('world_name')
    launch_cfg_pause = LaunchConfiguration('paused')
    launch_cfg_use_sim_time = LaunchConfiguration('use_sim_time')

    # LAUNCH FILES TO LAUNCH!
    # launch gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ]),
        ]),
        launch_arguments={
            'world': launch_cfg_world_name,
            'pause': launch_cfg_pause,
        }.items(),
    )

    # NODES TO LAUNCH!
    spawn_robot = Node(package='gazebo_ros',
                       executable='spawn_entity.py',
                       name='urdf_spawner',
                       arguments=['-topic', '/robot_description',
                                  '-entity', 'robot', 
                                  '-x', "0.0", 
                                  '-y', "0.0",
                                  '-z', "0.2"],
                       output='screen')
                       
    # robot state publisher
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': launch_cfg_use_sim_time,
                         'robot_description': robot_description,
                       }]
    )

    # Prepare arguments and node as objects
    arguments = [arg_world_name, arg_paused, arg_use_sim_time]
    nodes = [robot_state_publisher, spawn_robot]
    launch = [gazebo]
    evt = []
    objects = arguments + nodes + launch + evt
    
    # do launch
    return LaunchDescription(objects)