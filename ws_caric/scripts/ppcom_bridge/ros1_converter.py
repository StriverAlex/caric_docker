import rospy
import json
import sys
import os
from std_msgs.msg import String

# Import ROS1 packages (only successful within workspace)
from rotors_comm.msg import PPComTopology
from caric_mission.srv import CreatePPComTopic, CreatePPComTopicResponse

class PPComROS1Converter:
    """ROS1 PPCom Converter - Runs within the workspace"""
    
    def __init__(self):
        rospy.init_node('ppcom_ros1_converter', anonymous=False)
        rospy.loginfo("Starting PPCom ROS1 converter...")
        
        # Verify environment
        self.verify_environment()
        
        # Set up publishers - publish JSON format for bridging
        self.topo_json_pub = rospy.Publisher('/ppcom_topology_json', String, queue_size=1)
        self.topo_doa_json_pub = rospy.Publisher('/ppcom_topology_doa_json', String, queue_size=1)
        
        # Wait for original PPCom topics
        self.wait_for_ppcom_topics()
        
        # Set up subscribers - subscribe to original PPCom topics
        self.topo_sub = rospy.Subscriber('/ppcom_topology', PPComTopology, self.topo_callback)
        self.topo_doa_sub = rospy.Subscriber('/ppcom_topology_doa', PPComTopology, self.topo_doa_callback)
        
        # Set up service proxy
        self.setup_service_proxy()
        
        rospy.loginfo("PPCom ROS1 converter startup completed!")
        
    def verify_environment(self):
        """Verify ROS environment is correct"""
        try:
            # Test package import
            test_msg = PPComTopology()
            rospy.loginfo("✓ rotors_comm package imported successfully")
            
            # Check workspace
            package_path = os.environ.get('ROS_PACKAGE_PATH', '')
            if '/root/ws_caric' not in package_path:
                rospy.logwarn("Workspace path may be incorrect")
            else:
                rospy.loginfo("✓ Workspace environment correct")
                
        except Exception as e:
            rospy.logfatal(f"Environment verification failed: {e}")
            sys.exit(1)
    
    def wait_for_ppcom_topics(self):
        """Wait for PPCom topics to appear"""
        rospy.loginfo("Waiting for PPCom topics...")
        timeout = 30
        count = 0
        
        while count < timeout:
            topics = rospy.get_published_topics()
            topic_names = [topic for topic, _ in topics]
            ppcom_topics = [t for t in topic_names if 'ppcom' in t.lower()]
            
            if ppcom_topics:
                rospy.loginfo(f"Discovered PPCom topics: {ppcom_topics}")
                return
            
            count += 1
            rospy.sleep(1.0)
            if count % 5 == 0:
                rospy.loginfo(f"Waiting for PPCom topics... ({count}/{timeout})")
        
        rospy.logwarn("No PPCom topics found, ensure CARIC simulation is running")
    
    def setup_service_proxy(self):
        """Set up service proxy"""
        try:
            rospy.wait_for_service('create_ppcom_topic', timeout=5.0)
            self.ppcom_service_proxy = rospy.ServiceProxy('create_ppcom_topic', CreatePPComTopic)
            
            # Provide JSON version service
            self.ppcom_service_json = rospy.Service(
                'create_ppcom_topic_json', 
                CreatePPComTopic, 
                self.create_ppcom_topic_json_callback
            )
            rospy.loginfo("✓ PPCom service proxy setup completed")
        except rospy.ROSException:
            rospy.logwarn("Original PPCom service unavailable")
    
    def topo_callback(self, msg):
        """Convert PPComTopology message to JSON"""
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
            
            rospy.loginfo_throttle(10, f"Converted topology: {len(msg.node_id)} nodes")
            
        except Exception as e:
            rospy.logerr(f"Topology conversion error: {e}")
    
    def topo_doa_callback(self, msg):
        """Convert PPComTopology DOA message to JSON"""
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
            
            rospy.loginfo_throttle(10, f"Converted topology DOA: {len(msg.node_id)} nodes")
            
        except Exception as e:
            rospy.logerr(f"Topology DOA conversion error: {e}")
    
    def create_ppcom_topic_json_callback(self, req):
        """Handle JSON version PPCom service request"""
        try:
            rospy.loginfo(f"PPCom service request: {req.source} -> {req.targets}")
            response = self.ppcom_service_proxy(req)
            rospy.loginfo(f"PPCom service response: {response.result}")
            return response
        except Exception as e:
            rospy.logerr(f"PPCom service error: {e}")
            response = CreatePPComTopicResponse()
            response.result = f"fail! Error: {str(e)}"
            return response

def main():
    try:
        converter = PPComROS1Converter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("PPCom ROS1 converter exited")

if __name__ == '__main__':
    main()