import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
#    tf2_base_link_lidar_base = Node(
#        package='tf2_ros',
#        executable='static_transform_publisher',
#        arguments=['0.13', '0', '0.6', '1', '0', '0', '0', 'base_link', 'lidar_base']
#    )

    map_yaml_file = os.path.join(
        get_package_share_directory('sentry_master'),
        'map/uedalab.yaml',
    )
    lifecycle_nodes = ['map_server'] 

    map_server = Node(
	package='nav2_map_server',
	executable='map_server',
	name='map_server',
	parameters=[{'yaml_filename': map_yaml_file}],
	output='screen'
    )

    lifecycle_manager = Node(
	package='nav2_lifecycle_manager',
	executable='lifecycle_manager',
	name='lifecycle_manager_localization',
	output='screen',
	parameters=[{'autostart': True},
		    {'node_names': lifecycle_nodes}]
    )

    exec_livox_lidar = ExecuteProcess(
        cmd=[ 'ros2', 'launch',
            'pointcloud_to_laserscan', 'livox2d.launch.py'],
        output='screen',
    )

    exec_lidar_localization = ExecuteProcess(
        cmd=[ 'ros2', 'launch',
            'lidar_localization_ros2', 'lidar_localization.launch.py'],
        output='screen',
    )

#    convert_pose2tf_node = Node(
#            package='convert_pose2tf',
#            executable='convert_pose2tf',
#            name='convert_pose2tf_node',
#            output='screen',
#    )
            
    
    nav2_config = os.path.join(get_package_share_directory('sentry_master'), 'config/nav2_param.yaml')

    nav2_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch/navigation_launch.py'
        ]),
        launch_arguments={
                'use_sim_time': 'False',
                'params_file': nav2_config,
#                'map': map_yaml_file,
        }.items()
    )

    rviz2_config = os.path.join(
        get_package_share_directory('sentry_master'),
        'rviz',
        'sentry_cn.rviz'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[["-d"], [rviz2_config]],
        remappings=[('/move_base_simple/goal','/goal_pose')] #目標位置姿勢を示すTopic
    )

    ld = LaunchDescription()
    ld.add_action(map_server)
    ld.add_action(lifecycle_manager)
    ld.add_action(exec_livox_lidar)
    ld.add_action(exec_lidar_localization)
#    ld.add_action(convert_pose2tf_node)
    ld.add_action(nav2_launch)
    ld.add_action(rviz2_node)
    return ld

