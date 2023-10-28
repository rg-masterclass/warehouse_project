import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation clock if true')

    param_substitutions = {
        'use_sim_time': use_sim_time
    }

    map_file_arg = DeclareLaunchArgument(
        "map_file", default_value="warehouse_map_sim.yaml"
    )

    map_yaml = PathJoinSubstitution([
        FindPackageShare('map_server'), 'config', LaunchConfiguration("map_file")
    ])

    nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')

    urdf_file = '/home/user/simulation_ws/src/warehouse_robot_lab/rb1_base_common/rb1_base_description/robots/rb1_base.urdf'

    with open(urdf_file, 'r') as info:
        robot_desc = info.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, 
                    {'yaml_filename':map_yaml}]
    )

    amcl_node_config = RewrittenYaml(
        source_file=nav2_yaml,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_node_config]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]
    )     

    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_state_publisher_node,
        map_file_arg,
        map_server_node,
        amcl_node,
        lifecycle_manager_node
    ])
