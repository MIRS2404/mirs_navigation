#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>

class Nav2Controller : public rclcpp::Node
{
public:
    Nav2Controller() : Node("nav2_controller")
    {
        // コールバックグループの設定
        callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        
        // サービスサーバの作成
        stop_service_ = create_service<std_srvs::srv::Trigger>(
            "stop_robot_service",
            std::bind(&Nav2Controller::handleStopService, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_
        );
        
        resume_service_ = create_service<std_srvs::srv::Trigger>(
            "resume_robot_service",
            std::bind(&Nav2Controller::handleResumeService, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_
        );
        
        // cmd_velのサブスクライバとパブリッシャー
        rclcpp::QoS qos(10);
        rclcpp::SubscriptionOptions options;
        options.callback_group = callback_group_;
        
        vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_raw",
            qos,
            std::bind(&Nav2Controller::velocityCallback, this, std::placeholders::_1),
            options
        );
        
        vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_modified", 10);
        
        // タイマーの設定（50ms周期）
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&Nav2Controller::timerCallback, this)
        );
        
        // 状態管理
        is_paused_ = false;
        current_scale_ = 1.0;
        target_scale_ = 1.0;
        last_velocity_ = std::make_shared<geometry_msgs::msg::Twist>();
        
        RCLCPP_INFO(get_logger(), "Stop and Resume Services are ready.");
    }

private:
    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_velocity_ = msg;  // 最新の速度指令を保存
        updateVelocity();
    }
    
    void timerCallback()
    {
        if (std::abs(current_scale_ - target_scale_) > 0.01) {
            // 1秒かけて目標値に到達するように更新（50ms × 20 = 1000ms）
            if (current_scale_ < target_scale_) {
                current_scale_ += 0.05;  // 1.0 / 20
                if (current_scale_ > target_scale_) current_scale_ = target_scale_;
            } else {
                current_scale_ -= 0.05;  // 1.0 / 20
                if (current_scale_ < target_scale_) current_scale_ = target_scale_;
            }
            updateVelocity();
        }
    }
    
    void updateVelocity()
    {
        if (last_velocity_) {
            auto scaled_vel = std::make_unique<geometry_msgs::msg::Twist>();
            scaled_vel->linear.x = last_velocity_->linear.x * current_scale_;
            scaled_vel->linear.y = last_velocity_->linear.y * current_scale_;
            scaled_vel->linear.z = last_velocity_->linear.z * current_scale_;
            scaled_vel->angular.x = last_velocity_->angular.x * current_scale_;
            scaled_vel->angular.y = last_velocity_->angular.y * current_scale_;
            scaled_vel->angular.z = last_velocity_->angular.z * current_scale_;
            vel_pub_->publish(*scaled_vel);
        }
    }
    
    void handleStopService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(get_logger(), "Stop service called - Smoothly stopping");
        target_scale_ = 0.0;
        is_paused_ = true;
        
        response->success = true;
        response->message = "Robot is stopping smoothly";
    }
    
    void handleResumeService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(get_logger(), "Resume service called - Smoothly resuming");
        target_scale_ = 1.0;
        is_paused_ = false;
        
        response->success = true;
        response->message = "Robot is resuming smoothly";
    }
    
    // メンバ変数
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool is_paused_;
    double current_scale_;
    double target_scale_;
    geometry_msgs::msg::Twist::SharedPtr last_velocity_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<Nav2Controller>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    try {
        executor.spin();
    } catch (const std::exception & e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in executor: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}