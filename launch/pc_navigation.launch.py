from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # デフォルトのRVizの設定ファイルのパスを設定
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
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',
            'set_initial_pose': True,
            'initial_pose.x': 0.0,
            'initial_pose.y': 0.0,
            'initial_pose.z': 0.0,
            'initial_pose.yaw': 0.0,
            'alpha1': 0.2,
            'alpha2': 0.2,
            'alpha3': 0.2,
            'alpha4': 0.2,
            'alpha5': 0.2,
            'laser_model_type': 'likelihood_field',
            'laser_likelihood_max_dist': 2.0,
            'max_beams': 60,
            'min_particles': 500,
            'max_particles': 2000,
            'update_min_d': 0.1,
            'update_min_a': 0.1,
            'resample_interval': 1,
            'transform_tolerance': 0.5,
            'recovery_alpha_slow': 0.001,
            'recovery_alpha_fast': 0.1,
            'initial_pose_topic': 'initialpose',
            'scan_topic': 'scan'
        }]
    )

    # Controller
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'controller_frequency': 20.0,
            'controller_plugins': ['FollowPath'],
            'FollowPath': {
                'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
                'desired_linear_vel': 0.5,
                'max_linear_accel': 2.5,
                'max_linear_decel': 2.5,
                'max_angular_vel': 1.0,
                'max_angular_accel': 3.2,
                'transform_tolerance': 0.1
            }
        }]
    )

    # Planner
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'planner_plugins': ['GridBased'],
            'GridBased': {
                'plugin': 'nav2_navfn_planner/NavfnPlanner',
                'tolerance': 0.5,
                'use_astar': False,
                'allow_unknown': True
            }
        }]
    )

    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'global_frame': 'map',
            'robot_base_frame': 'base_link',
            'transform_tolerance': 0.1,
            'default_bt_xml_filename': os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'
            ),
            'plugin_lib_names': [
                'nav2_compute_path_to_pose_action_bt_node',
                'nav2_compute_path_through_poses_action_bt_node',
                'nav2_follow_path_action_bt_node',
                'nav2_back_up_action_bt_node',
                'nav2_spin_action_bt_node',
                'nav2_wait_action_bt_node',
                'nav2_clear_costmap_service_bt_node',
                'nav2_is_stuck_condition_bt_node',
                'nav2_goal_reached_condition_bt_node',
                'nav2_goal_updated_condition_bt_node',
                'nav2_initial_pose_received_condition_bt_node',
                'nav2_reinitialize_global_localization_service_bt_node',
                'nav2_rate_controller_bt_node',
                'nav2_distance_controller_bt_node',
                'nav2_speed_controller_bt_node',
                'nav2_truncate_path_action_bt_node',
                'nav2_goal_updater_node_bt_node',
                'nav2_recovery_node_bt_node',
                'nav2_pipeline_sequence_bt_node',
                'nav2_round_robin_node_bt_node',
                'nav2_transform_available_condition_bt_node',
                'nav2_time_expired_condition_bt_node',
                'nav2_distance_traveled_condition_bt_node',
                'nav2_single_trigger_bt_node'
            ]
        }]
    )

    # Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl', 'controller_server', 'planner_server', 'bt_navigator']
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