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
    default_map_path = '/home/omasahiro/mirs2404_ws/src/mirs_slam_navigation/mirs_navigation/maps/hallway.yaml'

    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        emulate_tty=True,
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
        emulate_tty=True,
        parameters=[nav2_params_path]
    )

    # Controller
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        emulate_tty=True,
        parameters=[nav2_params_path]
    )

    # Planner
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        emulate_tty=True,
        parameters=[nav2_params_path]
    )

    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        emulate_tty=True,
        parameters=[nav2_params_path]
    )

    # Single Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl', 'controller_server', 'planner_server', 'bt_navigator', 'behavior_server', 'velocity_smoother'],
            'bond_timeout': 4.0,
            'attempt_respawn_reconnection': True
        }]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_path]
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        emulate_tty=True,
        parameters=[nav2_params_path]
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        emulate_tty=True,
        parameters=[nav2_params_path]
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        emulate_tty=True,
        parameters=[nav2_params_path]
    )

    # Nav2Controller
    nav2_controller_node = Node(
        package='mirs_navigation',
        executable='nav2_controller.py',
        name='nav2_controller',
        output='screen'
    )

    

    nav2_costmap_2d = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='nav2_costmap_2d',
        output='screen',
        parameters=[nav2_params_path]
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

    ld.add_action(map_server)
    ld.add_action(lifecycle_manager)
    #ld.add_action(nav2_costmap_2d)
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(behavior_server)
    ld.add_action(smoother_server)
    ld.add_action(velocity_smoother)
    ld.add_action(waypoint_follower)
    ld.add_action(amcl)
    ld.add_action(bt_navigator)
    ld.add_action(nav2_controller_node)
    ld.add_action(rviz2_node)

    return ld
