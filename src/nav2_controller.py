import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class Nav2Controller(Node):
    def __init__(self):
        super().__init__('nav2_controller')
        
        # コールバックグループの設定
        callback_group = ReentrantCallbackGroup()
        
        # サービスサーバの作成
        self.stop_service = self.create_service(
            Trigger, 'stop_robot_service', self.handle_stop_service,
            callback_group=callback_group
        )
        self.resume_service = self.create_service(
            Trigger, 'resume_robot_service', self.handle_resume_service,
            callback_group=callback_group
        )
        
        # cmd_velのサブスクライバとパブリッシャー
        self.vel_sub = self.create_subscription(
            Twist, 'cmd_vel_raw', self.velocity_callback,
            10, callback_group=callback_group
        )
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 状態管理
        self.is_paused = False
        self.target_scale = 1.0
        
        self.get_logger().info('Stop and Resume Services are ready.')
    
    def velocity_callback(self, msg):
        """速度指令のコールバック"""
        if not self.is_paused:
            # 通常の速度指令を転送
            self.vel_pub.publish(msg)
        else:
            # 停止中は速度ゼロを送信
            self.publish_zero_velocity()
    
    def handle_stop_service(self, request, response):
        """ロボット停止処理"""
        self.get_logger().info('Stop service called')
        self.target_scale = 0.0
        self.is_paused = True
        self.publish_zero_velocity()
        
        response.success = True
        response.message = 'Robot stopped successfully'
        return response
    
    def handle_resume_service(self, request, response):
        """ロボット再開処理"""
        self.get_logger().info('Resume service called')
        self.target_scale = 1.0
        self.is_paused = False
        
        response.success = True
        response.message = 'Robot resumed successfully'
        return response
    
    def publish_zero_velocity(self):
        """速度をゼロに設定"""
        zero_vel = Twist()
        zero_vel.linear.x = 0.0
        zero_vel.linear.y = 0.0
        zero_vel.angular.z = 0.0
        self.vel_pub.publish(zero_vel)

def main(args=None):
    rclpy.init(args=args)
    
    executor = MultiThreadedExecutor()
    controller = Nav2Controller()
    executor.add_node(controller)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.publish_zero_velocity()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    