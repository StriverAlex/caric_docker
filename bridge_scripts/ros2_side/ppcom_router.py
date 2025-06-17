import rclpy
from rclpy.node import Node
import json
import sys
import os
import threading
import numpy as np
from threading import Lock

# 添加共享模块路径
sys.path.insert(0, '/root/bridge_scripts')

from std_msgs.msg import String
from shared.ppcom_msgs import PPComTopologyData, PPComMessageTypes, PPComServiceData

class PPComAccess:
    """PPCom网络拓扑管理"""
    
    def __init__(self, topo_data):
        self.lock = Lock()
        self.topo_data = topo_data
        self.node_id = list(topo_data.node_id)
        self.node_alive = list(topo_data.node_alive)
        self.Nnodes = len(topo_data.node_id)
        self.adjacency = self.range_to_link_mat(topo_data.range, self.Nnodes)

    def range_to_link_mat(self, distances, N):    
        link_mat = np.zeros([N, N])
        range_idx = 0
        for i in range(0, N):
            for j in range(i+1, N):
                if range_idx < len(distances):
                    link_mat[i, j] = distances[range_idx]
                    link_mat[j, i] = link_mat[i, j]
                    range_idx += 1
        return link_mat

    def update(self, topo_data):
        with self.lock:
            self.topo_data = topo_data
            self.node_alive = list(topo_data.node_alive)
            self.adjacency = self.range_to_link_mat(topo_data.range, self.Nnodes)

    def get_adj(self):
        with self.lock:
            return self.adjacency.copy()

class Dialogue:
    """话题对话管理"""
    
    def __init__(self, topic_name, source):
        self.topic_name = topic_name
        self.sources = {source}
        self.target_to_pub = {}
        self.permitted_edges = set()
        self.subscription = None

    def add_target_publisher(self, target, pub):
        self.target_to_pub[target] = pub

    def add_source(self, source):
        self.sources.add(source)

    def add_permitted_edge(self, edge):
        self.permitted_edges.add(edge)

class PPComRouterROS2(Node):
    """ROS2 PPCom路由器"""
    
    def __init__(self):
        super().__init__('ppcom_router_ros2')
        
        self.ppcom_topo = None
        self.topic_to_dialogue = {}
        self.dialogue_mutex = threading.Lock()
        
        self.get_logger().info("ROS2 PPCom Router启动中...")
        
        # 订阅转换后的拓扑消息
        self.topology_sub = self.create_subscription(
            String, '/ppcom_topology_converted', self.topology_callback, 10)
        self.topology_doa_sub = self.create_subscription(
            String, '/ppcom_topology_doa_converted', self.topology_doa_callback, 10)
        
        # PPCom服务接口（使用JSON）
        self.service_request_sub = self.create_subscription(
            String, '/create_ppcom_topic_request', self.service_request_callback, 10)
        self.service_response_pub = self.create_publisher(
            String, '/create_ppcom_topic_response', 10)
        
        self.get_logger().info("ROS2 PPCom Router启动完成!")
    
    def topology_callback(self, msg):
        """处理初始拓扑"""
        try:
            topo_data = PPComTopologyData.from_json_string(msg.data)
            
            if self.ppcom_topo is None:
                self.ppcom_topo = PPComAccess(topo_data)
                self.get_logger().info(f"初始拓扑: {self.ppcom_topo.node_id}")
                
        except Exception as e:
            self.get_logger().error(f"拓扑处理错误: {e}")
    
    def topology_doa_callback(self, msg):
        """处理拓扑更新"""
        try:
            topo_data = PPComTopologyData.from_json_string(msg.data)
            
            if self.ppcom_topo is not None:
                self.ppcom_topo.update(topo_data)
                self.get_logger().debug("拓扑已更新")
                
        except Exception as e:
            self.get_logger().error(f"拓扑更新错误: {e}")
    
    def create_data_callback(self, topic_name, source_node):
        """创建数据回调函数"""
        def data_callback(msg):
            self.relay_message(msg, topic_name, source_node)
        return data_callback
    
    def relay_message(self, msg, topic_name, source_node):
        """根据PPCom规则转发消息"""
        if self.ppcom_topo is None:
            return

        adjacency = self.ppcom_topo.get_adj()
        
        with self.dialogue_mutex:
            if topic_name not in self.topic_to_dialogue:
                return
                
            dialogue = self.topic_to_dialogue[topic_name]
            relayed_count = 0
            
            for target_node in self.ppcom_topo.node_id:
                if target_node == source_node:
                    continue

                try:
                    i = self.ppcom_topo.node_id.index(source_node)
                    j = self.ppcom_topo.node_id.index(target_node)
                except ValueError:
                    continue

                # 检查节点状态和LOS连接
                if (i >= len(self.ppcom_topo.node_alive) or 
                    j >= len(self.ppcom_topo.node_alive)):
                    continue

                if (not self.ppcom_topo.node_alive[i] or 
                    not self.ppcom_topo.node_alive[j]):
                    continue

                if adjacency[i, j] <= 0.0:
                    continue
                
                # 检查权限
                if (source_node, target_node) not in dialogue.permitted_edges:
                    continue
                
                # 转发消息
                if target_node in dialogue.target_to_pub:
                    dialogue.target_to_pub[target_node].publish(msg)
                    relayed_count += 1
            
            if relayed_count > 0:
                self.get_logger().debug(f"forward: {source_node} -> {relayed_count}targets")
    
    def service_request_callback(self, msg):
        """处理PPCom服务请求"""
        try:
            request_data = json.loads(msg.data)
            service_data = PPComServiceData.from_dict(request_data)
            
            self.get_logger().info(f"PPCom service: {service_data.source} -> {service_data.targets}")
            
            # 处理'all'目标
            targets = service_data.targets.copy()
            if 'all' in targets:
                if self.ppcom_topo:
                    targets = [node_id for node_id in self.ppcom_topo.node_id 
                              if node_id != service_data.source]
                else:
                    targets = ['raffles', 'jurong', 'sentosa', 'changi', 'nanyang']
                    if service_data.source in targets:
                        targets.remove(service_data.source)
            
            with self.dialogue_mutex:
                topic_name = service_data.topic_name
                source = service_data.source
                
                # 创建或更新对话
                if topic_name not in self.topic_to_dialogue:
                    dialogue = Dialogue(topic_name, source)
                    self.topic_to_dialogue[topic_name] = dialogue
                    
                    # 创建订阅器
                    dialogue.subscription = self.create_subscription(
                        String, topic_name, 
                        self.create_data_callback(topic_name, source), 10)
                else:
                    dialogue = self.topic_to_dialogue[topic_name]
                    dialogue.add_source(source)

                # 为每个目标设置发布器和权限
                for target in targets:
                    if target == source:
                        continue
                    
                    dialogue.add_permitted_edge((source, target))
                    
                    if target not in dialogue.target_to_pub:
                        target_topic = f"{topic_name}/{target}"
                        pub = self.create_publisher(String, target_topic, 10)
                        dialogue.add_target_publisher(target, pub)
                        self.get_logger().info(f"create publisher: {target_topic}")

            # 发送响应
            response = {"result": "success!"}
            response_msg = String()
            response_msg.data = json.dumps(response)
            self.service_response_pub.publish(response_msg)
            
        except Exception as e:
            self.get_logger().error(f"Service processing error: {e}")
            response = {"result": f"fail! Error: {str(e)}"}
            response_msg = String()
            response_msg.data = json.dumps(response)
            self.service_response_pub.publish(response_msg)

def main(args=None):
    rclpy.init(args=args)
    router = PPComRouterROS2()
    
    try:
        rclpy.spin(router)
    except KeyboardInterrupt:
        router.get_logger().info("receive interrupt signal")
    finally:
        router.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()