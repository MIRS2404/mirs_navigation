#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rcl_interfaces.msg import Parameter, ParameterValue
from rclpy.parameter import Parameter

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        
        try:
            # 一時停止状態の初期化
            self.declare_parameter('is_paused', False)
            self._is_paused = False
            
            # 一時停止サービス
            self.pause_service = self.create_service(
                Trigger, 
                'pause_robot_service', 
                self.pause_callback
            )
            
            # 再開サービス
            self.resume_service = self.create_service(
                Trigger, 
                'resume_robot_service', 
                self.resume_callback
            )
            
            self.get_logger().info('RobotControlNode initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Error in initialization: {str(e)}')
            raise

    def pause_callback(self, request, response):
        try:
            self._is_paused = True
            self.set_parameters([Parameter('is_paused', Parameter.Type.BOOL, True)])
            self.get_logger().info('Robot paused')
            response.success = True
            response.message = 'Robot paused'
            return response
        except Exception as e:
            self.get_logger().error(f'Error in pause_callback: {str(e)}')
            response.success = False
            response.message = f'Error: {str(e)}'
            return response

    def resume_callback(self, request, response):
        try:
            self._is_paused = False
            self.set_parameters([Parameter('is_paused', Parameter.Type.BOOL, False)])
            self.get_logger().info('Robot resumed')
            response.success = True
            response.message = 'Robot resumed'
            return response
        except Exception as e:
            self.get_logger().error(f'Error in resume_callback: {str(e)}')
            response.success = False
            response.message = f'Error: {str(e)}'
            return response

def main():
    rclpy.init()
    try:
        node = RobotControlNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()