cmake_minimum_required(VERSION 3.5)
project(mirs_navigation)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Pythonパッケージのセットアップ（もしscriptsディレクトリがある場合）
install(PROGRAMS
  scripts/robot_control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 設定ファイルとlaunchファイルのインストール
install(DIRECTORY
  launch
  rviz
  params
  btdata
  maps
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()