from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the launch directory
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    default_rviz_config_path = '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'  # 修正：直接定義
    package_name = 'mirs_navigation'
    
    # Set default paths
    #default_map_path = os.path.join(
    #    get_package_share_directory(package_name),
    #    'maps',
    #    'mapNIT.yaml'
    #)
     
    default_map_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'maps',
        'turtlebot3_world.yaml'
    )
    
    default_bt_xml_path = os.path.join(
        get_package_share_directory(package_name),
        'btdata',
        'patrol_tree.xml'
    )
    
    # 標準のnav2_params.yamlのパスを追加
    #nav2_params_path = os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')
    nav2_params_path = os.path.join(
        get_package_share_directory(package_name),
        'params',
        'nav2_params.yaml'
    )

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config')
    
    # Launch Arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file to load'
    )
    
    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=default_bt_xml_path,
        description='Full path to behavior tree xml file'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_path,
        description='Full path to the ROS2 parameters file to use'
    )
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use'
    )
    
    # Nav2 Launch
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file,
            'default_bt_xml_filename': default_bt_xml_filename,
            'autostart': 'true'
        }.items()
    )

#    navigation_bt_node = Node(
#        package='mirs_navigation',
#        executable='navigation_bt_node',
#        name='navigation_bt_node',
#        output='screen'
#    ) 
    
    #navigation_control_node = Node(
    #    package='mirs_navigation',
    #    executable='navigation_control_node.py',
    #    name='navigation_control_node',
    #    output='screen'
    #)

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Lifecycle Manager - AMCLを追加
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                   {'autostart': True},
                   {'node_names': ['controller_server', 'planner_server', 'recoveries_server', 'bt_navigator', 'map_server', 'amcl']}]
    )

    bt_ctrl = Node(
        package='behaviortree_cpp_v3',
        executable='bt_ros2_control_node',
        name='bt_ros2_control',
        output='screen',
        parameters=[
            {'publisher_port': 1666},
            {'server_port': 1667}
        ]
    )



    

    # Launch Description
    ld = LaunchDescription()
    
    #ld.add_action(navigation_bt_node)
    #ld.add_action(bt_ctrl)
    #ld.add_action(navigation_control_node)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(nav2_bringup_launch)
    ld.add_action(lifecycle_manager)
    ld.add_action(rviz_node)
    
    return ld