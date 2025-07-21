# filepath: /root/dora_digua_long/node-hub/dora-openlong/dora_openlong/contrl.py
import dora
from dora import Node
import json  # 导入 json 模块

class CmdDataReceiver:
    def __init__(self):
        self.node = Node()
        
    def on_event(self):
        for event in self.node:
            if event["type"] == "INPUT" and event["id"] == "arm_state":
                try:
                    # 将 pyarrow 数组转换为字节对象
                    raw_bytes = bytes(event["value"].to_pylist())
                    data = json.loads(raw_bytes.decode("utf-8"))
                    print("✅ 接收到新的控制指令：")
                    # print(f"  序列号: {data.get('sequence')}")
                    # print(f"  时间戳: {data.get('timestamp')}")
                    print(f"  是否在充电: {data.get('in_charge')}")
                    print(f"  滤波等级: {data.get('filt_level')}")
                    print(f"  臂部模式: {data.get('arm_mode')}")
                    print(f"  手指模式: {data.get('finger_mode')}")
                    print(f"  脖子模式: {data.get('neck_mode')}")
                    print(f"  左臂位置: {data.get('arx_pos_left')}")
                    print(f"  右臂位置: {data.get('arx_pos_right')}")
                    print(f"  左臂力矩: {data.get('arm_fm_left')}")
                    print(f"  右臂力矩: {data.get('arm_fm_right')}")
                    print(f"  左手姿态: {data.get('hand_q_left')}")
                    print(f"  右手姿态: {data.get('hand_q_right')}")
                    print(f"  脖子角度: {data.get('neck_q')}")
                    print(f"  腰部角度: {data.get('waist_q')}")
                    print("-" * 40)
                except Exception as e:
                    print(f"处理输入数据时出错: {e}")

def main():
    print("🚀 Dora 控制节点已启动，等待接收控制指令...")
    Cmd = CmdDataReceiver()
    Cmd.on_event()