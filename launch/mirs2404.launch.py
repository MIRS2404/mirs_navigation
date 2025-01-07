from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ナビゲーションコントロールノード
    navigation_control_node = Node(
        package='navigation_control',
        executable='navigation_control',
        name='navigation_control_node',
        output='screen',  # 標準出力に出力
        emulate_tty=True  # 色付きの出力を有効化
    )

    # Web設定インターフェースノード
    web_config_node = Node(
        package='navigation_control',
        executable='web_config',
        name='web_config_node',
        output='screen',  # 標準出力に出力
        emulate_tty=True  # 色付きの出力を有効化
    )

    raspi_receiver = Node(
        package='websocket',
        executable='raspi_receiver',
        name='raspi_receiver_node',
        output='screen',  # 標準出力に出力
        emulate_tty=True  # 色付きの出力を有効化
    )


    return LaunchDescription([
        navigation_control_node,
        web_config_node,
        raspi_receiver
    ])