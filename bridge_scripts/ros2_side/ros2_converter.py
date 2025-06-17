import rclpy
from rclpy.node import Node
import json
import sys
import os

# 添加共享模块路径
sys.path.insert(0, '/root/bridge_scripts')

from std_msgs.msg import String
from shared.ppcom_msgs import PPComTopologyData, PPComMessageTypes

class PPComROS2Converter(Node):
    """ROS2 PPCom转换器 - 将JSON消息转换回ROS2标准格式"""
    
    def __init__(self):
        super().__init__('ppcom_ros2_converter')
        
        self.get_logger().info("PPCom ROS2 converter start...")
        
        # 订阅从bridge传来的JSON消息
        self.topo_json_sub = self.create_subscription(
            String, '/ppcom_topology_json', self.topo_json_callback, 10)
        self.topo_doa_json_sub = self.create_subscription(
            String, '/ppcom_topology_doa_json', self.topo_doa_json_callback, 10)
        
        # 发布转换后的消息供PPCom Router使用
        self.topo_converted_pub = self.create_publisher(
            String, '/ppcom_topology_converted', 10)
        self.topo_doa_converted_pub = self.create_publisher(
            String, '/ppcom_topology_doa_converted', 10)
        
        # 存储最新的拓扑数据
        self.latest_topology = None
        
        self.get_logger().info("PPCom ROS2 converter start completely!")
    
    def topo_json_callback(self, msg):
        """处理拓扑JSON消息"""
        try:
            topology_data = PPComTopologyData.from_json_string(msg.data)
            
            if topology_data.message_type == PPComMessageTypes.TOPOLOGY:
                self.latest_topology = topology_data
                
                # 转发给PPCom Router
                self.topo_converted_pub.publish(msg)
                self.get_logger().debug(f"convert topology: {len(topology_data.node_id)}node")
            
        except Exception as e:
            self.get_logger().error(f"topology JSON convert failed: {e}")
    
    def topo_doa_json_callback(self, msg):
        """处理拓扑DOA JSON消息"""
        try:
            topology_data = PPComTopologyData.from_json_string(msg.data)
            
            if topology_data.message_type == PPComMessageTypes.TOPOLOGY_DOA:
                if self.latest_topology:
                    # 更新存储的拓扑数据
                    self.latest_topology.node_alive = topology_data.node_alive
                    self.latest_topology.range = topology_data.range
                    self.latest_topology.timestamp = topology_data.timestamp
                
                # 转发给PPCom Router
                self.topo_doa_converted_pub.publish(msg)
                self.get_logger().debug(f"conv topology DOA: {len(topology_data.node_id)}node")
            
        except Exception as e:
            self.get_logger().error(f" topology DOA JSON convert fail: {e}")
    
    def get_current_topology(self):
        """获取当前拓扑数据"""
        return self.latest_topology

def main(args=None):
    rclpy.init(args=args)
    converter = PPComROS2Converter()
    
    try:
        rclpy.spin(converter)
    except KeyboardInterrupt:
        converter.get_logger().info("receive interrupt signal")
    finally:
        converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()