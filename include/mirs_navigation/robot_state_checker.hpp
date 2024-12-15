// include/mirs_navigation/robot_state_checker.hpp
#ifndef ROBOT_STATE_CHECKER_HPP_
#define ROBOT_STATE_CHECKER_HPP_

#include <string>
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace mirs_navigation {

class RobotStateChecker : public BT::ConditionNode
{
public:
    RobotStateChecker(
        const std::string& name,
        const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("robot_state_checker");
        
        // サービスサーバーの作成
        service_ = node_->create_service<std_srvs::srv::SetBool>(
            "/robot_state_control",
            std::bind(&RobotStateChecker::stateCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        // ROSノードのスピナーを開始
        spinner_thread_ = std::thread([this]() {
            rclcpp::spin(node_);
        });
    }

    ~RobotStateChecker()
    {
        if (spinner_thread_.joinable()) {
            rclcpp::shutdown();
            spinner_thread_.join();
        }
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        return is_paused_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }

private:
    void stateCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        is_paused_ = request->data;
        response->success = true;
        response->message = is_paused_ ? "Robot paused" : "Robot resumed";
        RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    std::thread spinner_thread_;
    bool is_paused_{false};
};

}  // namespace mirs_navigation

#endif  // ROBOT_STATE_CHECKER_HPP_