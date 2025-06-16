import rospy
import json
import sys
import os
from std_msgs.msg import String

# 导入ROS1包（只有在工作空间内才能成功导入）
from rotors_comm.msg import PPComTopology
from caric_mission.srv import CreatePPComTopic, CreatePPComTopicResponse

class PPComROS1Converter:
    """ROS1 PPCom转换器 - 运行在工作空间内"""
    
    def __init__(self):
        rospy.init_node('ppcom_ros1_converter', anonymous=False)
        rospy.loginfo("PPCom ROS1转换器启动中...")
        
        # 验证环境
        self.verify_environment()
        
        # 设置发布器 - 发布JSON格式用于桥接
        self.topo_json_pub = rospy.Publisher('/ppcom_topology_json', String, queue_size=1)
        self.topo_doa_json_pub = rospy.Publisher('/ppcom_topology_doa_json', String, queue_size=1)
        
        # 等待原始PPCom话题
        self.wait_for_ppcom_topics()
        
        # 设置订阅器 - 订阅原始PPCom话题
        self.topo_sub = rospy.Subscriber('/ppcom_topology', PPComTopology, self.topo_callback)
        self.topo_doa_sub = rospy.Subscriber('/ppcom_topology_doa', PPComTopology, self.topo_doa_callback)
        
        # 设置服务代理
        self.setup_service_proxy()
        
        rospy.loginfo("PPCom ROS1转换器启动完成!")
        
    def verify_environment(self):
        """验证ROS环境是否正确"""
        try:
            # 测试包导入
            test_msg = PPComTopology()
            rospy.loginfo("✓ rotors_comm包导入成功")
            
            # 检查工作空间
            package_path = os.environ.get('ROS_PACKAGE_PATH', '')
            if '/root/ws_caric' not in package_path:
                rospy.logwarn("工作空间路径可能不正确")
            else:
                rospy.loginfo("✓ 工作空间环境正确")
                
        except Exception as e:
            rospy.logfatal(f"环境验证失败: {e}")
            sys.exit(1)
    
    def wait_for_ppcom_topics(self):
        """等待PPCom话题出现"""
        rospy.loginfo("等待PPCom话题...")
        timeout = 30
        count = 0
        
        while count < timeout:
            topics = rospy.get_published_topics()
            topic_names = [topic for topic, _ in topics]
            ppcom_topics = [t for t in topic_names if 'ppcom' in t.lower()]
            
            if ppcom_topics:
                rospy.loginfo(f"发现PPCom话题: {ppcom_topics}")
                return
            
            count += 1
            rospy.sleep(1.0)
            if count % 5 == 0:
                rospy.loginfo(f"等待PPCom话题... ({count}/{timeout})")
        
        rospy.logwarn("未发现PPCom话题，请确保CARIC仿真已启动")
    
    def setup_service_proxy(self):
        """设置服务代理"""
        try:
            rospy.wait_for_service('create_ppcom_topic', timeout=5.0)
            self.ppcom_service_proxy = rospy.ServiceProxy('create_ppcom_topic', CreatePPComTopic)
            
            # 提供JSON版本的服务
            self.ppcom_service_json = rospy.Service(
                'create_ppcom_topic_json', 
                CreatePPComTopic, 
                self.create_ppcom_topic_json_callback
            )
            rospy.loginfo("✓ PPCom服务代理设置完成")
        except rospy.ROSException:
            rospy.logwarn("原始PPCom服务不可用")
    
    def topo_callback(self, msg):
        """转换PPComTopology消息为JSON"""
        try:
            json_data = {
                'message_type': 'ppcom_topology',
                'node_id': list(msg.node_id),
                'node_alive': list(msg.node_alive),
                'range': list(msg.range),
                'timestamp': rospy.Time.now().to_sec()
            }
            
            json_msg = String()
            json_msg.data = json.dumps(json_data)
            self.topo_json_pub.publish(json_msg)
            
            rospy.loginfo_throttle(10, f"转换拓扑: {len(msg.node_id)}节点")
            
        except Exception as e:
            rospy.logerr(f"拓扑转换错误: {e}")
    
    def topo_doa_callback(self, msg):
        """转换PPComTopology DOA消息为JSON"""
        try:
            json_data = {
                'message_type': 'ppcom_topology_doa',
                'node_id': list(msg.node_id),
                'node_alive': list(msg.node_alive),
                'range': list(msg.range),
                'timestamp': rospy.Time.now().to_sec()
            }
            
            json_msg = String()
            json_msg.data = json.dumps(json_data)
            self.topo_doa_json_pub.publish(json_msg)
            
            rospy.loginfo_throttle(10, f"转换拓扑DOA: {len(msg.node_id)}节点")
            
        except Exception as e:
            rospy.logerr(f"拓扑DOA转换错误: {e}")
    
    def create_ppcom_topic_json_callback(self, req):
        """处理JSON版本的PPCom服务请求"""
        try:
            rospy.loginfo(f"PPCom服务请求: {req.source} -> {req.targets}")
            response = self.ppcom_service_proxy(req)
            rospy.loginfo(f"PPCom服务响应: {response.result}")
            return response
        except Exception as e:
            rospy.logerr(f"PPCom服务错误: {e}")
            response = CreatePPComTopicResponse()
            response.result = f"fail! Error: {str(e)}"
            return response

def main():
    try:
        converter = PPComROS1Converter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("PPCom ROS1转换器退出")

if __name__ == '__main__':
    main()