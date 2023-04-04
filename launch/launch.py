import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'solo12dof_quadruped.urdf'
    urdf = os.path.join(
        get_package_share_directory('solo12dof_quadruped'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='solo12dof_quadruped',
            executable='state_publisher',
            name='state_publisher',
            output='screen'),
        Node(
            package='ignition_gazebo',
            executable='ignition gazebo',
            name='ignition_gazebo',
            output='screen',
            arguments=['-s', 'libgazebo_ros_init.so', urdf],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])



# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node

# def generate_launch_description():

#     use_sim_time = LaunchConfiguration('use_sim_time', default='false')

#     urdf_file_name = 'solo12dof_quadruped.urdf'
#     urdf = os.path.join(
#         get_package_share_directory('solo12dof_quadruped'),
#         urdf_file_name)
#     with open(urdf, 'r') as infp:
#         robot_desc = infp.read()

#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'use_sim_time',
#             default_value='false',
#             description='Use simulation (Gazebo) clock if true'),
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             name='robot_state_publisher',
#             output='screen',
#             parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
#             arguments=[urdf]),
#         Node(
#             package='solo12dof_quadruped',
#             executable='state_publisher',
#             name='state_publisher',
#             output='screen'),
#     ])

    