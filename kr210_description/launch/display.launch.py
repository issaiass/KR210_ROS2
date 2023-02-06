from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():
    # CONSTANTS
    package    = 'kr210_description' # the package of the launch file
    robot_xacro = 'kr210.urdf.xacro' # the robot xacro file
    viz_base   = 'kr210.rviz' # the visualization loaded
    
    # PATHS
    package_path = os.path.join(get_package_share_directory(package))
    xacro_file = os.path.join(package_path, 'urdf', robot_xacro)
    rviz_config_path = os.path.join(get_package_share_directory(package), 'rviz', viz_base)

    # load urdf    
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = doc.toxml()


    # ARGUMENT DECLARATIONS
    arg_joint_state_publisher = DeclareLaunchArgument(name='joint_state_publisher_gui_enable', default_value='true', description='Launch joint_states_publisher [true, false]')
    arg_use_sim_time = DeclareLaunchArgument(name='use_sim_time', default_value='false', description='Use simulation time [true, false]')
    arg_rviz_config_path = DeclareLaunchArgument(name='rviz_config_path', default_value=rviz_config_path, description='Run rviz [true, false]')
    arg_use_rviz = DeclareLaunchArgument(name='use_rviz', default_value='true', description='Run rviz [true, false]')

    # LAUNCH CONFIGURATIONS
    launch_cfg_joint_state_publisher = LaunchConfiguration("joint_state_publisher_gui_enable") 
    launch_cfg_use_sim_time = LaunchConfiguration('use_sim_time')
    launch_cfg_rviz_config_path = LaunchConfiguration('rviz_config_path')
    launch_cfg_use_rviz = LaunchConfiguration('use_rviz')

    # NODES TO LAUNCH
    # joint state publisher gui
    joint_state_publisher_gui = Node(
            condition=IfCondition(launch_cfg_joint_state_publisher),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher'
    )
    # joint state publisher
    joint_state_publisher = Node(
            condition=UnlessCondition(launch_cfg_joint_state_publisher),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
    )
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
    # rviz2
    rviz2 = Node(
            condition=IfCondition(launch_cfg_use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', launch_cfg_rviz_config_path],
            parameters=[{'use_sim_time': launch_cfg_use_sim_time}]
            )

    # Prepare arguments and node as objects
    arguments = [arg_joint_state_publisher, arg_use_sim_time, arg_rviz_config_path, arg_use_rviz]
    nodes = [joint_state_publisher, joint_state_publisher_gui, robot_state_publisher, rviz2]
    launch = []
    evt = []
    objects = arguments + nodes + launch + evt
    
    # do launch
    return LaunchDescription(objects)