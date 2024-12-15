// bt_nodes.cpp
#include "behaviortree_cpp_v3/bt_factory.h"
#include "mirs_navigation/robot_state_checker.hpp"

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<mirs_navigation::RobotStateChecker>("RobotStateChecker");
}