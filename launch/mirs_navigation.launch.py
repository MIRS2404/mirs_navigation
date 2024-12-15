from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the launch directory
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # デフォルトのマップパスとRviz設定ファイルパスを設定
    package_name = 'mirs_navigation'  # あなたのパッケージ名に変更してください
    default_map_path = os.path.join(
        get_package_share_directory(package_name),
        'maps',
        'hallway.yaml'  # あなたのマップファイル名に変更してください
    )
    
    default_rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'default.rviz'  # あなたのRviz設定ファイル名に変更してください
    )
    
    # ビヘイビアツリーのデフォルトパスを設定
    default_bt_xml_path = os.path.join(
        get_package_share_directory(package_name),
        'btdata',
        'sample.xml'
    )
    
    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    rviz_config_file = LaunchConfiguration('rviz_config')
    
    # Declare launch arguments
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
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use'
    )
    
    # Include the Nav2 launch file with our parameters
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'default_bt_xml_filename': default_bt_xml_filename
        }.items()
    )
    
    # Start RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(nav2_bringup_launch)
    ld.add_action(rviz_node)
    
    return ld