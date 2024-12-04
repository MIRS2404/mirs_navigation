from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # パッケージのパスを取得
    package_name = 'mirs_navigation'
    package_path = get_package_share_directory(package_name)
    
    # 設定ファイルのパスを設定
    nav2_params_path = os.path.join(package_path, 'config', 'nav2_params.yaml')
    rviz_config_file = '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'
    
    # デフォルトのマップパスを設定
    default_map_path = '/home/omasahiro/maps/hallway.yaml'

    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': default_map_path,
            'use_sim_time': use_sim_time,
            'frame_id': 'map'
        }]
    )

    # AMCL
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_path,
            {'use_sim_time': use_sim_time}]
    )

    # Controller
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_path,
            {'use_sim_time': use_sim_time}]
    )

    # Planner
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_path,
            {'use_sim_time': use_sim_time}]
    )

    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_path,
            {'use_sim_time': use_sim_time}]
    )

    # Lifecycle Manager for Navigation
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }]
    )

    # Lifecycle Manager for Navigation Core
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['amcl',
                          'controller_server',
                          'planner_server',
                          'bt_navigator'],
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

    # Add nodes
    ld.add_action(map_server)
    ld.add_action(amcl)
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(bt_navigator)
    ld.add_action(lifecycle_manager_navigation)  # マップサーバーを先に起動
    ld.add_action(lifecycle_manager_localization)  # その他のナビゲーションノードを後から起動
    ld.add_action(rviz2_node)

    return ld