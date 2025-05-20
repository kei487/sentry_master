import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
#    ldlidar_launch = IncludeLaunchDescription(
#        launch_description_source=PythonLaunchDescriptionSource([
#            get_package_share_directory('ldlidar_node'),
#            '/launch/ldlidar_with_mgr.launch.py'
#        ])
#    )
#
#    laser_filter_node = Node(
#            package="laser_filters",
#            executable="scan_to_scan_filter_chain",
#            parameters=[
#                    PathJoinSubstitution([
#                    get_package_share_directory("laser_filters"),
#                    "examples", "box_filter_example.yaml",
#            ])],
#            remappings=[('/scan','/ldlidar_node/scan'),('/scan_filtered','/scan')],
#    )

    tf2_base_link_lidar_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.13', '0', '0.6', '1', '0', '0', '0', 'base_link', 'lidar_base']
    )

    bringup_dir = get_package_share_directory('turtlebot3_cartographer')
    cartographer_config_dir = os.path.join(bringup_dir, 'config')
    cartographer_map_dir = os.path.join(bringup_dir, 'map')
    configuration_basename = 'noimu_lds_2d_nav.lua'

    use_sim_time = False
    resolution = '0.05'
    publish_period_sec = '1.0'

    cartographer_node = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]
    )

    occupancy_grid_node = Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
    )
    
    nav2_config = os.path.join(cartographer_config_dir, 'nav2_param.yaml')
    nav2_map = os.path.join(cartographer_map_dir, 'map.yaml')

    nav2_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch/bringup_launch.py'
        ]),
        launch_arguments={
                'use_sim_time': 'False',
                'params_file': nav2_config,
                'map': nav2_map,
        }.items()
    )

    rviz2_config = os.path.join(
        get_package_share_directory('turtlebot3_cartographer'),
        'rviz',
        'tb3_cartographer_nav.rviz'
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
#    ld.add_action(ldlidar_launch)
#    ld.add_action(laser_filter_node)
    ld.add_action(tf2_base_link_lidar_base)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(nav2_launch)
    ld.add_action(rviz2_node)
    return ld

