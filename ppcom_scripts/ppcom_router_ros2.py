#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rotors_comm_msgs.msg import PPComTopology
import threading
import numpy as np
import time

class CompletePPComRouter(Node):
    def __init__(self):
        super().__init__('complete_ppcom_router')
        
        # UAV列表和域映射
        self.uav_list = ['jurong', 'raffles', 'changi', 'sentosa', 'nanyang']
        self.domain_mapping = {
            'jurong': 1,
            'raffles': 2, 
            'changi': 3,
            'sentosa': 4,
            'nanyang': 5
        }
        
        # 拓扑管理
        self.ppcom_access = None
        self.topology_lock = threading.Lock()
        
        # 通信管理
        self.setup_uav_communication()
        
        # 拓扑订阅
        self.topology_sub = self.create_subscription(
            PPComTopology,
            '/ppcom_topology_doa',
            self.topology_callback,
            10)
        
        # 统计信息
        self.message_count = {uav: 0 for uav in self.uav_list}
        self.last_message_time = {uav: 0 for uav in self.uav_list}
        
        # 状态监控定时器
        self.status_timer = self.create_timer(5.0, self.print_status)
        
        self.get_logger().info('Complete PPCom Router started on Domain 10')
        self.get_logger().info(f'Managing UAVs: {self.uav_list}')
    
    def setup_uav_communication(self):
        """为每个UAV设置通信通道"""
        self.uav_subscribers = {}
        self.uav_publishers = {}
        
        for uav_name in self.uav_list:
            # 订阅来自UAV的消息 (UAV域 → Router域)
            from_topic = f'/ppcom_from_{uav_name}'
            self.uav_subscribers[uav_name] = self.create_subscription(
                String,
                from_topic,
                lambda msg, uav=uav_name: self.handle_uav_message(msg, uav),
                10
            )
            
            # 发布到UAV的消息 (Router域 → UAV域)
            to_topic = f'/ppcom_to_{uav_name}'
            self.uav_publishers[uav_name] = self.create_publisher(
                String,
                to_topic,
                10
            )
            
            self.get_logger().info(f'Setup communication for {uav_name}: {from_topic} → {to_topic}')
    
    def handle_uav_message(self, msg, source_uav):
        """处理来自UAV的消息"""
        # 更新统计信息
        self.message_count[source_uav] += 1
        self.last_message_time[source_uav] = time.time()
        
        # 记录接收到的消息
        self.get_logger().info(f'📨 Received from {source_uav}: {msg.data[:60]}...')
        
        # 路由消息到其他UAV
        routed_count = self.route_message_to_uavs(msg, source_uav)
        
        if routed_count > 0:
            self.get_logger().info(f'📤 Routed message from {source_uav} to {routed_count} UAVs')
    
    def route_message_to_uavs(self, msg, source_uav):
        """将消息路由到其他UAV"""
        routed_count = 0
        
        for target_uav in self.uav_list:
            if target_uav == source_uav:
                continue  # 不发给自己
            
            # 检查连接性（视距检查）
            if self.check_connectivity(source_uav, target_uav):
                try:
                    # 发送消息到目标UAV
                    self.uav_publishers[target_uav].publish(msg)
                    routed_count += 1
                    self.get_logger().debug(f'  ↳ Sent to {target_uav}')
                except Exception as e:
                    self.get_logger().warn(f'Failed to send to {target_uav}: {e}')
            else:
                self.get_logger().debug(f'  ✗ No line of sight: {source_uav} → {target_uav}')
        
        return routed_count
    
    def check_connectivity(self, source_uav, target_uav):
        """检查两个UAV之间的连接性（视距）"""
        with self.topology_lock:
            # 如果没有拓扑信息，默认允许通信
            if not self.ppcom_access:
                return True
            
            try:
                # 查找UAV在拓扑中的索引
                source_idx = self.ppcom_access.node_id.index(source_uav)
                target_idx = self.ppcom_access.node_id.index(target_uav)
                
                # 检查节点是否存活
                if (source_idx >= len(self.ppcom_access.node_alive) or 
                    target_idx >= len(self.ppcom_access.node_alive)):
                    return False
                
                if (not self.ppcom_access.node_alive[source_idx] or 
                    not self.ppcom_access.node_alive[target_idx]):
                    return False
                
                # 检查视距连接
                adjacency = self.ppcom_access.get_adj()
                los_value = adjacency[source_idx, target_idx]
                
                return los_value > 0.0
                
            except (ValueError, IndexError, AttributeError) as e:
                self.get_logger().debug(f'Connectivity check error: {e}')
                return True  # 出错时默认允许通信
    
    def topology_callback(self, msg):
        """更新拓扑信息"""
        with self.topology_lock:
            if self.ppcom_access is None:
                self.ppcom_access = PPComAccess(msg)
                self.get_logger().info('🗺️  PPCom topology initialized')
            else:
                self.ppcom_access.update(msg)
                self.get_logger().debug('🗺️  PPCom topology updated')
        
        # 打印拓扑状态
        if self.ppcom_access:
            alive_nodes = [node for i, node in enumerate(self.ppcom_access.node_id) 
                          if i < len(self.ppcom_access.node_alive) and self.ppcom_access.node_alive[i]]
            self.get_logger().info(f'🟢 Active nodes: {alive_nodes}')
    
    def print_status(self):
        """打印系统状态"""
        current_time = time.time()
        self.get_logger().info('📊 === PPCom Router Status ===')
        
        for uav in self.uav_list:
            msg_count = self.message_count[uav]
            last_time = self.last_message_time[uav]
            
            if last_time > 0:
                time_since_last = current_time - last_time
                status = "🟢 Active" if time_since_last < 5.0 else "🟡 Quiet"
                self.get_logger().info(f'  {uav}: {msg_count} msgs, {status} ({time_since_last:.1f}s ago)')
            else:
                self.get_logger().info(f'  {uav}: No messages received')
        
        # 打印拓扑状态
        if self.ppcom_access:
            self.get_logger().info('🗺️  Topology matrix (line of sight):')
            adj_matrix = self.ppcom_access.get_simple_adj()
            node_names = self.ppcom_access.node_id
            
            # 打印邻接矩阵
            for i, source in enumerate(node_names):
                if i < len(adj_matrix):
                    connections = []
                    for j, target in enumerate(node_names):
                        if i != j and j < len(adj_matrix[i]) and adj_matrix[i][j] > 0:
                            connections.append(target)
                    
                    if connections:
                        self.get_logger().info(f'    {source} → {connections}')
        
        self.get_logger().info('📊 ========================')


class PPComAccess:
    """PPCom拓扑访问类"""
    
    def __init__(self, topo):
        self.lock = threading.Lock()
        self.node_id = list(topo.node_id)
        self.node_alive = list(topo.node_alive)
        self.n_nodes = len(topo.node_id)
        self.adjacency = self.range_to_link_mat(topo.range, self.n_nodes)
    
    def range_to_link_mat(self, distances, n):
        """将距离数组转换为邻接矩阵"""
        link_mat = np.zeros([n, n])
        range_idx = 0
        
        for i in range(n):
            for j in range(i+1, n):
                if range_idx < len(distances):
                    # 距离 > 0 表示有连接
                    link_mat[i, j] = distances[range_idx]
                    link_mat[j, i] = link_mat[i, j]
                    range_idx += 1
        
        return link_mat
    
    def update(self, topo):
        """更新拓扑信息"""
        with self.lock:
            self.node_alive = list(topo.node_alive)
            self.adjacency = self.range_to_link_mat(topo.range, self.n_nodes)
    
    def get_adj(self):
        """获取邻接矩阵副本"""
        with self.lock:
            return self.adjacency.copy()
    
    def get_simple_adj(self):
        """获取简化的邻接矩阵（0/1值）"""
        adj = self.get_adj()
        
        # 将距离值转换为0/1连接状态
        for i in range(adj.shape[0]):
            for j in range(adj.shape[1]):
                adj[i][j] = 1.0 if adj[i][j] > 0.0 else 0.0
        
        return adj


def main(args=None):
    rclpy.init(args=args)
    
    # 设置Domain 10
    import os
    os.environ['ROS_DOMAIN_ID'] = '10'
    
    router = CompletePPComRouter()
    
    try:
        rclpy.spin(router)
    except KeyboardInterrupt:
        router.get_logger().info('PPCom Router shutting down...')
    finally:
        router.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()