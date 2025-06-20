#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from caric_mission.srv import CreatePPComTopic

class CaricAgentROS2(Node):
    def __init__(self, namespace):
        super().__init__(f'{namespace}_agent')
        self.namespace = namespace
        
        # 创建PPCom客户端
        self.ppcom_client = self.create_client(
            CreatePPComTopic,
            'create_ppcom_topic')
        
        # 等待服务可用
        while not self.ppcom_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('PPCom service not available, waiting...')
        
        # 创建广播话题
        self.setup_broadcast()
        
        # 订阅广播消息
        self.broadcast_sub = self.create_subscription(
            String,
            f'/broadcast/{self.namespace}',
            self.broadcast_callback,
            10)
        
        # 发布广播消息
        self.broadcast_pub = self.create_publisher(String, '/broadcast', 10)
        
        # 定时器发送位置信息
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def setup_broadcast(self):
        request = CreatePPComTopic.Request()
        request.source = self.namespace
        request.targets = ['all']
        request.topic_name = '/broadcast'
        request.package_name = 'std_msgs'
        request.message_type = 'String'
        
        future = self.ppcom_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            self.get_logger().info(f'PPCom setup result: {future.result().result}')
    
    def broadcast_callback(self, msg):
        self.get_logger().info(f'Received broadcast: {msg.data}')
        # 处理接收到的广播消息
        self.process_broadcast_message(msg.data)
    
    def process_broadcast_message(self, data):
        # 解析广播消息 (类似原来的communicate函数)
        parts = data.split(';')
        if len(parts) < 2:
            return
            
        topic = parts[0]
        
        if topic == "position":
            # 处理位置消息
            pass
        elif topic == "state":
            # 处理状态消息
            pass
        elif topic == "map" or topic == "mapglobal":
            # 处理地图消息
            pass
        # ... 其他消息类型处理
    
    def timer_callback(self):
        # 发送位置信息
        msg = String()
        msg.data = f"position;{self.namespace};1.0,2.0,3.0;4.0,5.0,6.0"
        self.broadcast_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    # 创建不同命名空间的代理
    agents = []
    namespaces = ['jurong', 'raffles', 'changi', 'sentosa', 'nanyang']
    
    for ns in namespaces:
        agent = CaricAgentROS2(ns)
        agents.append(agent)
    
    # 使用多线程执行器
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    
    for agent in agents:
        executor.add_node(agent)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for agent in agents:
            agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()