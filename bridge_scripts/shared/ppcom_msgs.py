from dataclasses import dataclass
from typing import List
import json

@dataclass
class PPComTopologyData:
    """PPCom拓扑数据类 - 用于JSON序列化/反序列化"""
    message_type: str
    node_id: List[str]
    node_alive: List[bool]
    range: List[float]
    timestamp: float = 0.0
    
    def to_dict(self):
        """转换为字典"""
        return {
            'message_type': self.message_type,
            'node_id': self.node_id,
            'node_alive': self.node_alive,
            'range': self.range,
            'timestamp': self.timestamp
        }
    
    @classmethod
    def from_dict(cls, data):
        """从字典创建对象"""
        return cls(
            message_type=data.get('message_type', ''),
            node_id=data.get('node_id', []),
            node_alive=data.get('node_alive', []),
            range=data.get('range', []),
            timestamp=data.get('timestamp', 0.0)
        )
    
    @classmethod
    def from_json_string(cls, json_str):
        """从JSON字符串创建对象"""
        try:
            data = json.loads(json_str)
            return cls.from_dict(data)
        except Exception as e:
            raise ValueError(f"JSON解析失败: {e}")
    
    def to_json_string(self):
        """转换为JSON字符串"""
        return json.dumps(self.to_dict())

class PPComMessageTypes:
    """PPCom消息类型常量"""
    TOPOLOGY = 'ppcom_topology'
    TOPOLOGY_DOA = 'ppcom_topology_doa'
    SERVICE_REQUEST = 'ppcom_service_request'
    SERVICE_RESPONSE = 'ppcom_service_response'

@dataclass
class PPComServiceData:
    """PPCom服务数据类"""
    source: str
    targets: List[str]
    topic_name: str
    package_name: str
    message_type: str
    
    def to_dict(self):
        return {
            'source': self.source,
            'targets': self.targets,
            'topic_name': self.topic_name,
            'package_name': self.package_name,
            'message_type': self.message_type
        }
    
    @classmethod
    def from_dict(cls, data):
        return cls(
            source=data.get('source', ''),
            targets=data.get('targets', []),
            topic_name=data.get('topic_name', ''),
            package_name=data.get('package_name', ''),
            message_type=data.get('message_type', '')
        )