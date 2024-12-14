cmake_minimum_required(VERSION 3.5)
project(mirs_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)

# Pythonスクリプトのインストール設定を追加
install(DIRECTORY
  launch
  rviz
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Pythonスクリプトをインストール
install(PROGRAMS
  src/nav2_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()