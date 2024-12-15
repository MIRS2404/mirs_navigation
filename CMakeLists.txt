cmake_minimum_required(VERSION 3.5)
project(mirs_navigation)

# コンパイラの設定
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依存パッケージの検索
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)

# インクルードディレクトリの設定
include_directories(
  include
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
  ${nav2_behavior_tree_INCLUDE_DIRS}
)

# BTノードのライブラリをビルド
add_library(mirs_navigation_bt_nodes SHARED
  src/bt_nodes.cpp
)

target_include_directories(mirs_navigation_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# BTノードの依存関係設定
ament_target_dependencies(mirs_navigation_bt_nodes
  rclcpp
  nav2_behavior_tree
  behaviortree_cpp_v3
  std_srvs
)

# インストール設定
install(TARGETS
  mirs_navigation_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY 
  include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  rviz
  config
  btdata
  maps
  DESTINATION share/${PROJECT_NAME}/
)

# テスト設定
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()