import rclpy
from rclpy.node import Node
import json
import sys
import os
import random
import string

# 添加共享模块路径
sys.path.insert(0, '/root/bridge_scripts')

from std_msgs.msg import String
from shared.ppcom_msgs import PPComServiceData

class PPComClientROS2(Node):
    """ROS2 PPCom客户端"""
    
    def __init__(self, robot_id, targets):
        super().__init__(f'{robot_id}_ppcom_client')
        
        self.robot_id = robot_id
        self.targets = targets
        
        self.get_logger().info(f"PPCom客户端 {robot_id} 启动中...")
        
        # PPCom服务接口
        self.service_request_pub = self.create_publisher(
            String, '/create_ppcom_topic_request', 10)
        self.service_response_sub = self.create_subscription(
            String, '/create_ppcom_topic_response', self.service_response_callback, 10)
        
        # 等待服务可用并注册
        self.timer = self.create_timer(1.0, self.initial_registration)
        self.registration_done = False
        
        # 消息发布和接收
        self.message_pub = self.create_publisher(String, '/ping_message', 10)
        self.message_sub = self.create_subscription(
            String, f'/ping_message/{self.robot_id}', self.message_callback, 10)
        
        self.get_logger().info(f"PPCom客户端 {robot_id} 启动完成")
    
    def initial_registration(self):
        """初始注册"""
        if not self.registration_done:
            self.register_ppcom_topic()
            self.registration_done = True
            
            # 注册完成后，开始定时发送消息
            self.destroy_timer(self.timer)
            self.timer = self.create_timer(2.0, self.send_message)
    
    def register_ppcom_topic(self):
        """注册PPCom话题"""
        service_data = PPComServiceData(
            source=self.robot_id,
            targets=self.targets,
            topic_name='/ping_message',
            package_name='std_msgs',
            message_type='String'
        )
        
        request_msg = String()
        request_msg.data = json.dumps(service_data.to_dict())
        self.service_request_pub.publish(request_msg)
        
        self.get_logger().info(f"注册PPCom话题: {self.robot_id} -> {self.targets}")
    
    def service_response_callback(self, msg):
        """处理服务响应"""
        try:
            response_data = json.loads(msg.data)
            self.get_logger().info(f"服务响应: {response_data.get('result', 'unknown')}")
        except Exception as e:
            self.get_logger().error(f"响应处理错误: {e}")
    
    def message_callback(self, msg):
        """处理接收到的消息"""
        self.get_logger().info(f"[{self.robot_id}] 收到: {msg.data}")
    
    def send_message(self):
        """发送消息"""
        # 模拟不同节点的行为
        if self.robot_id == 'jurong':
            length = random.randint(0, 20)
            letters = string.ascii_lowercase
            result_str = ''.join(random.choice(letters) for i in range(length))
        else:
            result_str = "hello_ros2"
        
        msg = String()
        current_time = self.get_clock().now().to_msg().sec
        msg.data = f'{self.get_name()} says hello at time {current_time}. Random Text: {result_str}!'
        
        self.message_pub.publish(msg)
        self.get_logger().info(f"[{self.robot_id}] 发送: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    
    # 机器人配置
    robot_configs = {
        'raffles': ['jurong', 'sentosa'],
        'jurong': ['all'],
        'sentosa': ['raffles'], 
        'changi': ['all'],
        'nanyang': ['all']
    }
    
    if len(sys.argv) > 1:
        robot_id = sys.argv[1]
        targets = robot_configs.get(robot_id, ['all'])
    else:
        robot_id = 'default'
        targets = ['all']
    
    client = PPComClientROS2(robot_id, targets)
    
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().info("接收中断信号")
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()