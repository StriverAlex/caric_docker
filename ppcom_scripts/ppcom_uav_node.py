#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from caric_mission.srv import CreatePPComTopic
import sys
import os
import random
import string
import time

class PPComUAVNode(Node):
    def __init__(self, uav_name, domain_id):
        super().__init__(f'{uav_name}_ppcom_node')
        self.uav_name = uav_name
        self.domain_id = domain_id
        
        # 设置域ID
        os.environ['ROS_DOMAIN_ID'] = str(domain_id)
        
        # 与PPCom Router的通信
        self.router_pub = self.create_publisher(
            String, f'/ppcom_from_{uav_name}', 10)
        
        self.router_sub = self.create_subscription(
            String, f'/ppcom_to_{uav_name}', 
            self.message_callback, 10)

        # 定时发送消息
        self.timer = self.create_timer(1.0, self.send_message)
        
        self.get_logger().info(f'{uav_name} PPCom node started on Domain {domain_id}')
    
    def setup_ppcom_service(self):
        """设置PPCom服务连接"""
        # 注意：这里需要跨域服务调用，可能需要额外配置
        self.ppcom_client = self.create_client(
            CreatePPComTopic, 'create_ppcom_topic')
        
        # 等待服务可用
        self.get_logger().info('Waiting for PPCom service...')
        while not self.ppcom_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('PPCom service not available, waiting...')
        
        # 注册话题
        self.register_topic()
    
    def register_topic(self):
        """向PPCom Router注册话题"""
        request = CreatePPComTopic.Request()
        request.source = self.uav_name
        request.targets = ['all']  # 或指定特定目标
        request.topic_name = '/ping_message'
        request.package_name = 'std_msgs'
        request.message_type = 'String'
        
        future = self.ppcom_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            self.get_logger().info(f'PPCom registration result: {future.result().result}')
    
    def message_callback(self, msg):
        """接收来自其他UAV的消息"""
        self.get_logger().info(f'{self.uav_name} received: {msg.data}')
    
    def send_message(self):
        """定时发送消息"""
        # 生成随机字符串
        length = random.randint(5, 15)
        letters = string.ascii_lowercase
        result_str = ''.join(random.choice(letters) for i in range(length))
        
        # 创建消息
        txt = f"/{self.uav_name}_talker says hello at time {time.time()}. Random Text: {result_str}!"
        
        msg = String()
        msg.data = txt
        
        self.get_logger().info(f'SENDING: {txt}')
        self.router_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: ppcom_uav_node.py <uav_name>")
        return
    
    uav_name = sys.argv[1]
    
    # 域映射
    domain_mapping = {
        'jurong': 1,
        'raffles': 2, 
        'changi': 3,
        'sentosa': 4,
        'nanyang': 5
    }
    
    domain_id = domain_mapping.get(uav_name, 1)
    
    node = PPComUAVNode(uav_name, domain_id)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()