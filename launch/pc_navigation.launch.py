from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # パッケージのパスを取得
    package_name = 'mirs_navigation'  # あなたのパッケージ名に変更してください
    package_path = get_package_share_directory(package_name)
    
    # 設定ファイルのパスを設定
    nav2_params_path = os.path.join(package_path, 'params', 'nav2_params.yaml')
    rviz_config_file = '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'
    
    # 引数の宣言
    declare_map_arg = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file to load'
    )

    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',  # エラーメッセージを確認するため追加
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': use_sim_time
        }]
    )

    # AMCL
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[nav2_params_path]
    )

    # Controller
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_path]
    )

    # Planner
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_path]
    )

    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_path]
    )

    # Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',  # 名前を明確に
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 
                        'amcl',
                        'controller_server',
                        'planner_server',
                        'bt_navigator',
                        'global_costmap/global_costmap',
                        'local_costmap/local_costmap']  # costmapノードを追加
        }]
    )

    # RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Create and return launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_map_arg)

    # Add nodes
    ld.add_action(map_server)
    ld.add_action(amcl)
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(bt_navigator)
    ld.add_action(lifecycle_manager)
    ld.add_action(rviz2_node)

    return ld