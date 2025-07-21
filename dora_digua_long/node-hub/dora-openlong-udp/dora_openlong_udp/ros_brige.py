import socket
import json
from dora import Node
from collections import deque
# from loong_msgs.msg import CmdImitate

node=Node()

class UdpCmdReceiver:
    def __init__(self, port=8888, buffer_size=100):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', port))
        self.sock.settimeout(1.0)
        self.data_queue = deque(maxlen=buffer_size)
        ros2_context = dora.Ros2Context()
        ros2_node = ros2_context.new_node(
            "dora_custom_publisher", "/ros2_demo", dora.Ros2NodeOptions(rosout=True)
        )
        topic_qos = dora.Ros2QosPolicies(reliable=True, max_blocking_time=0.1)
        custom_topic = ros2_node.create_topic("/cmd_imitate", "loong_msgs/CmdImitate", topic_qos)
        self.cmd_publisher = ros2_node.create_publisher(custom_topic)
        self.dora_node = Node()  

    def _parse_packet(self, data):
        try:
            cmd_dict = json.loads(data.decode('utf-8'))
            cmd_msgs = CmdImitate()
            cmd_msgs.in_charge = cmd_dict['in_charge']
            cmd_msgs.filt_level = cmd_dict['filt_level']
            cmd_msgs.arm_mode = cmd_dict['arm_mode']
            cmd_msgs.finger_mode = cmd_dict['finger_mode']
            cmd_msgs.neck_mode = cmd_dict['neck_mode']
            cmd_msgs.arx_pos_left = cmd_dict['arx_pos_left']
            cmd_msgs.arx_pos_right = cmd_dict['arx_pos_right']
            cmd_msgs.hand_q_left = cmd_dict['hand_q_left']
            cmd_msgs.hand_q_right = cmd_dict['hand_q_right']
            cmd_msgs.timestamp = cmd_dict['timestamp']
            return cmd_msgs
        except Exception as e:
            print(f"解析错误: {type(e).__name__} - {str(e)}")
            return None

    def start(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
                parsed = self._parse_packet(data)
                if parsed:
                    self.cmd_publisher.publish(parsed)
                    print(f"收到指令 seq:{parsed['sequence']} 时间戳:{parsed['timestamp']}")
            except socket.timeout:
                continue

def main():
    node=Node()
    receiver = UdpCmdReceiver()
    receiver.start()