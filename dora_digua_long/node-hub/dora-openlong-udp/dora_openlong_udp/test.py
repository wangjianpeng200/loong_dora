import time
import dora
from dora import Node
# from base_interfaces_demo.msg import Student

class Student:
    name: string
    age: int
    height: int

def main():
    # 初始化 ROS2 上下文和节点（仅一次）
    ros2_context = dora.Ros2Context()
    ros2_node = ros2_context.new_node(
        "dora_custom_publisher", "/py01_topic", dora.Ros2NodeOptions(rosout=True)
    )
    topic_qos = dora.Ros2QosPolicies(reliable=True, max_blocking_time=0.1)
    # custom_topic = ros2_node.create_topic("topic_stu", "base_interfaces_demo/Student", topic_qos)
    custom_topic = ros2_node.create_topic("topic_stu", "base_interfaces_demo/Student", topic_qos)
    cmd_publisher = ros2_node.create_publisher(custom_topic)

    # 初始化 Dora 节点
    dora_node = Node()

    # 发布消息循环
    while True:
        msg = Student()
        msg.name = "dora"
        msg.age = 18
        msg.height = 180.0  # 修正为浮点数
        cmd_publisher.publish(msg)
        print("Published student message")
        time.sleep(1)

if __name__ == "__main__":
    main()