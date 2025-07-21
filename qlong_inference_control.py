#!/home/lin/software/miniconda3/envs/aloha/bin/python
# -- coding: UTF-8
"""
#!/usr/bin/python3
"""

from scipy.signal import butter

from rclpy.executors import MultiThreadedExecutor

import argparse
import sys
import threading
import time
import yaml
from collections import deque
import os
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from rclpy.node import Node
from rclpy.time import Time
import torch
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from PIL import Image as PImage
from sensor_msgs.msg import Image, JointState, CompressedImage
from driver_pvt.msg import DriverPVT
from std_msgs.msg import Header
from loong_msgs.msg import CmdImitate
import cv2

from scripts.agilex_model import create_model

import socket

cauchy_rdt_cnt = 0
cauchy_pub_cnt = 0
cauchy_pub_time = time.time()
last_rdt_inference_time = 0

# sys.path.append("./")

CAMERA_NAMES = ['cam_high', 'cam_right_wrist', 'cam_left_wrist']

observation_window = None

lang_embeddings = None

# debug
preload_images = None

observation_window_lock = threading.Lock()
observation_window_update = True

# 动作滤波器

class UdpCmdSender:
    def __init__(self, ip='127.0.0.1', port=8891):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.addr = (ip, port)
        
    def send_json(self, data_dict):
        try:
            payload = json.dumps(data_dict).encode('utf-8')
            return self.sock.sendto(payload, self.addr)
        except Exception as e:
            print(f"JSON序列化失败: {e}")
            return False


class MultiChannelButterworth:
    def __init__(self, cutoff, fs, channels, order=2):
        self.b, self.a = butter(order, cutoff / (0.5 * fs), btype='low')
        self.order = order
        self.channels = channels
        self.x_hist = np.zeros((len(self.b), channels))
        self.y_hist = np.zeros((len(self.a), channels))

    def filter(self, x):
        x = np.asarray(x)
        assert x.shape == (self.channels,), f"Expected shape ({self.channels},), got {x.shape}"

        # Shift history
        self.x_hist[1:] = self.x_hist[:-1]
        self.x_hist[0] = x
        self.y_hist[1:] = self.y_hist[:-1]

        # Compute output per channel
        y = (self.b[:, None] * self.x_hist).sum(axis=0) - \
            (self.a[1:, None] * self.y_hist[1:]).sum(axis=0)
        y /= self.a[0]

        self.y_hist[0] = y
        return y


fs = 50       # 采样率 50Hz
cutoff = 2    # 截止频率 5Hz
channels = 16  # 三通道数据（如加速度 X/Y/Z）

filt = MultiChannelButterworth(cutoff, fs, channels)

def time_from_header(header):
    return Time.from_msg(header.stamp).nanoseconds / 1e9

def set_seed(seed):
    torch.manual_seed(seed)
    np.random.seed(seed)

# Interpolate the actions to make the robot move smoothly
def interpolate_action(args, prev_action, cur_action):
    steps = np.concatenate((np.array(args.arm_steps_length), np.array(args.arm_steps_length)), axis=0)
    diff = np.abs(cur_action - prev_action)
    step = np.ceil(diff / steps).astype(int)
    step = np.max(step)
    if step <= 1:
        return cur_action[np.newaxis, :]
    new_actions = np.linspace(prev_action, cur_action, step + 1)
    return new_actions[1:]


def get_config(args):
    config = {
        'episode_len': args.max_publish_step,
        'state_dim': 16,
        'chunk_size': args.chunk_size,
        'camera_names': CAMERA_NAMES,
    }
    return config


# RDT inference
def inference_fn(args, config, policy, t):
    global observation_window, last_rdt_inference_time
    global lang_embeddings
    # print("inference")
    # print(f"Start inference_thread_fn: t={t}")
    while True and rclpy.ok():
        global ros_operator, observation_window_update
        time1 = time.time()
        # 给obs进程信号
        os.close(os.open("/run/cauchy_inference_fn_flag", os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644))
        print("[policy.step] Task Submit. Wait")
        # 等结果
        wait_cnt = 0
        while True:
            if os.access("/run/cauchy_step_ok_flag", os.F_OK):
                actions = np.load("/run/cauchy_actions_outputs.npy")
                os.unlink("/run/cauchy_step_ok_flag")
                print()
                print(f"{actions.shape = }, {actions.dtype=}")
                break
            else:
                wait_cnt += 1
                if wait_cnt == 5:
                    print(".", end=" ", flush=True)
                    wait_cnt=0
                time.sleep(0.01)
                
        global cauchy_rdt_cnt
        cauchy_rdt_cnt += 1
        last_rdt_inference_time = time.time() - time1
        print("\033[1;31m" + f"[{cauchy_rdt_cnt}] RDT inference_fn time = {last_rdt_inference_time:.3f} s" + "\033[0m")

        
        # print(f"Finish inference_thread_fn: t={t}")
        return actions


# Main loop for the manipulation task
def model_inference(args, config, ros_operator):
    os.system("rm /run/cauchy_step_ok_flag")
    os.system("rm /run/cauchy_input_data_ok_flag")
    os.system("rm /run/cauchy_inference_fn_flag")
    time.sleep(2)
    global lang_embeddings
    
    # Load rdt model
    # policy = make_policy(args)
    policy = None
    
    lang_dict = torch.load(args.lang_embeddings_path)
    # print(f"Running with instruction: \"{lang_dict['instruction']}\" from \"{lang_dict['name']}\"")
    # lang_embeddings = lang_dict["embeddings"]
    lang_embeddings = lang_dict
    
    max_publish_step = config['episode_len']
    chunk_size = config['chunk_size']

    # Initialize position of the puppet arm
    left0 = [0.6039, -1.4251, 1.5389, 2.1398, -0.1372, -0.0361, 0.0517, 0.0]
    # right0 = [-0.012175972573459148, -1.2897902727127075, -1.9375616312026978, 2.4675519466400146, 0.22487200796604156, 0.6093259453773499, -0.5160407423973083, 0.0]
    while (len(ros_operator.puppet_arm_right_deque)==0):
        ros_operator.show_deque()
        time.sleep(0.05)
    right_arm_latest = list(ros_operator.puppet_arm_right_deque[-1].position)
    right0 = [right_arm_latest[0], right_arm_latest[1], right_arm_latest[2], right_arm_latest[3], right_arm_latest[4], right_arm_latest[5]-0.8, right_arm_latest[6], 0.0]
    # print(f"right0:{right0}")
    ros_operator.puppet_arm_publish_continuous_wrapper(left0, right0)
    # ros_operator.puppet_arm_publish_wrapper(left0, right0)
    # time.sleep(5)

    left1 = [0.6039, -1.4251, 1.5389, 2.1398, -0.1372, -0.0361, 0.0517, 0.0]
    right1 = [-0.6039, -1.4251, -1.5389, 2.1398, 0.1372, right_arm_latest[5]-0.8, -0.517, 0.0]
    ros_operator.puppet_arm_publish_continuous_wrapper(left1, right1)

    left2 = [0.6039, -1.4251, 1.5389, 2.1398, -0.1372, -0.0361, 0.0517, 0.0]
    right2 = [-0.6039, -1.4251, -1.5389, 2.1398, 0.1372, -0.0361, -0.0517, 0.0]
    ros_operator.puppet_arm_publish_continuous_wrapper(left2, right2)
    print("-------Finish reset position--------")
    
    # init filter
    global filt
    for i in range(128):
        _ = filt.filter(np.array([0.6039, -1.4251, 1.5389, 2.1398, -0.1372, -0.0361, 0.0517, 0.0, -0.6039, -1.4251, -1.5389, 2.1398, 0.1372, -0.0361, -0.0517, 0.0]))
        # print(_)
    # ros_operator.puppet_arm_publish_wrapper(left2, right2)
    # left1 = [-0.00133514404296875, 0.00209808349609375, 0.01583099365234375, -0.032616615295410156, -0.00286102294921875, 0.00095367431640625, -0.3393220901489258]
    # right1 = [-0.00133514404296875, 0.00247955322265625, 0.01583099365234375, -0.032616615295410156, -0.00286102294921875, 0.00095367431640625, -0.3397035598754883]
    # print(f"initial state left:{left0}")
    # print(f"initial state right:{right0}")
    # str = input("Press ENTER to continue")
    # ros_operator.puppet_arm_publish_continuous_wrapper(left0, right0)
    # input("Press enter to continue")
    # ros_operator.puppet_arm_publish_continuous_wrapper(left1, right1)
    # Initialize the previous action to be the initial robot state
    pre_action = np.zeros(config['state_dim'])
    pre_action[:16] = np.array(
        [0.6039, -1.4251, 1.5389, 2.1398, -0.1372, -0.0361, 0.0517, 0.0] +
        [-0.6039, -1.4251, -1.5389, 2.1398, 0.1372, -0.0361, -0.0517, 0.0]
    )
    action = None
    str = input("Press ENTER to continue")
    # Inference loop
    with torch.inference_mode():
        # if rclpy.ok():
        #     update_window_thread = threading.Thread(target=update_observation_window_continues, args=(args,config,ros_operator))
        #     update_window_thread.start()
        #     print("update_window_thread.start()")
        while rclpy.ok():
            # The current time step
            t = 0
            rate = ros_operator.node_pub.create_rate(args.publish_rate)   # main loop
    
            action_buffer = np.zeros([chunk_size, config['state_dim']])
            
            while t < max_publish_step and rclpy.ok():
                # Update observation window
                # update_observation_window(args, config, ros_operator)
                
                # When coming to the end of the action chunk
                if t % chunk_size == 0:
                    time.sleep(0.8)
                    # Start inference
                    action_buffer = inference_fn(args, config, policy, t).copy()
                
                raw_action = action_buffer[t % chunk_size]
                action = raw_action
                # Interpolate the original action sequence
                if args.use_actions_interpolation:
                    # print(f"Time {t}, pre {pre_action}, act {action}")
                    interp_actions = interpolate_action(args, pre_action, action)
                else:
                    interp_actions = action[np.newaxis, :]
                # Execute the interpolated actions one by one
                for act in interp_actions:
                    left_action = act[:8]
                    right_action = act[8:16]
                    # print(f"left_action:{left_action}")
                    # print(f"right_action:{right_action}")
                    # str = input("Press ENTER to continue")
                    
                    if not args.disable_puppet_arm:
                        ros_operator.puppet_arm_publish_wrapper(left_action.tolist(), right_action.tolist())  # puppet_arm_publish_continuous_thread
                
                    if args.use_robot_base:
                        vel_action = act[14:16]
                        ros_operator.robot_base_publish(vel_action)
                    
                    # rate_begin_time = time.time()
                    # rate.sleep()
                    time.sleep(0.02)   # main loop
                    # print(f"model_inference rate.sleep t = {time.time() - rate_begin_time}")
                    
                    # time.sleep(0.01)
                    # print(f"doing action: {act}")
                t += 1
                
                # print("Published Step", t)
                pre_action = action.copy()


# ROS operator class
class RosOperator:
    def __init__(self, args):
        self.robot_base_deque = None
        self.puppet_arm_right_deque = None
        self.puppet_arm_left_deque = None
        self.img_front_deque = None
        self.img_right_deque = None
        self.img_left_deque = None
        self.img_front_depth_deque = None
        self.img_right_depth_deque = None
        self.img_left_depth_deque = None
        self.bridge = None
        self.puppet_arm_left_publisher = None
        self.puppet_arm_right_publisher = None
        self.robot_base_publisher = None
        self.puppet_arm_publish_thread = None
        self.puppet_arm_publish_lock = None
        self.drv_msg_latest = None
        self.args = args
        self.node = None
        self.ros_node_thread = None
        self.deque_length = 2000
        self.init()
        self.init_ros()
        self.Udpcmd=UdpCmdSender()
        
    #     self.dump_header = ["", "", "", "", ""] # IMG: L, R, H; Joints: L, R;
    #     # 创建一个定时器，每 10 秒触发一次 timer_callback 函数
    #     self.timer = self.node.create_timer(10.0, self.timer_callback)
    #     self.counter = 0  # 可选计数器

    # def timer_callback(self):
    #     names = ["header_left_img.csv", "header_right_img.csv", "header_front_img.csv", "header_left_arm.csv", "header_right_arm.csv"]
    #     for i in range(5):
    #         with open(names[i], 'w', encoding='utf-8') as f:
    #             f.write(self.dump_header[i])
    #         print(f"已保存 {names[i]}")
    #     self.counter += 1
    #     self.node.get_logger().info(f'定时器已触发第 {self.counter} 次，当前时间：{self.node.get_clock().now().to_msg()}')
        

    def init(self):
        self.bridge = CvBridge()
        self.img_left_deque = deque(maxlen=self.deque_length)
        self.img_right_deque = deque(maxlen=self.deque_length)
        self.img_front_deque = deque(maxlen=self.deque_length)
        self.img_left_depth_deque = deque(maxlen=self.deque_length)
        self.img_right_depth_deque = deque(maxlen=self.deque_length)
        self.img_front_depth_deque = deque(maxlen=self.deque_length)
        self.puppet_arm_left_deque = deque(maxlen=self.deque_length)
        self.puppet_arm_right_deque = deque(maxlen=self.deque_length)
        self.robot_base_deque = deque(maxlen=self.deque_length)
        self.puppet_arm_publish_lock = threading.Lock()
        self.puppet_arm_publish_lock.acquire()

    def show_deque(self):
        print(f"IMG: L={len(self.img_left_deque)}, F={len(self.img_front_deque)}, R={len(self.img_right_deque)}; Joints: L={len(self.puppet_arm_left_deque)}, R={len(self.puppet_arm_right_deque)}")

    def reset_all_the_deque(self):
        print(f"IMG: L={len(self.img_left_deque)}, F={len(self.img_front_deque)}, R={len(self.img_right_deque)}; Joints: L={len(self.puppet_arm_left_deque)}, R={len(self.puppet_arm_right_deque)}")
        self.img_left_deque.clear()
        self.img_right_deque.clear()
        self.img_front_deque.clear()
        self.puppet_arm_left_deque.clear()
        self.puppet_arm_right_deque.clear()
        print(f"IMG: L={len(self.img_left_deque)}, F={len(self.img_front_deque)}, R={len(self.img_right_deque)}; Joints: L={len(self.puppet_arm_left_deque)}, R={len(self.puppet_arm_right_deque)}")
        print("==> reset_all_the_deque success.")

    def puppet_arm_publish(self, left, right):
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = rospy.Time.now()  # Set timestep
        joint_state_msg.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # 设置关节名称
        joint_state_msg.position = left
        self.puppet_arm_left_publisher.publish(joint_state_msg)
        joint_state_msg.position = right
        self.puppet_arm_right_publisher.publish(joint_state_msg)

    def puppet_arm_publish_wrapper(self, left, right):
        # filt
        global filt
        joints = filt.filter(np.array([left[0], left[1], left[2], left[3], left[4], left[5], left[6], left[7], right[0], right[1], right[2], right[3], right[4], right[5], right[6], right[7]]))
        left[0], left[1], left[2], left[3], left[4], left[5], left[6], left[7], right[0], right[1], right[2], right[3], right[4], right[5], right[6], right[7] = joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], joints[6], joints[7], joints[8], joints[9], joints[10], joints[11], joints[12], joints[13], joints[14], joints[15]
        
        # pub
        driver_pvt_msg = self.drv_msg_latest
        cmd_msg = CmdImitate()
        cmd_msg.in_charge = 1
        cmd_msg.filt_level = 1
        cmd_msg.arm_mode = 3
        cmd_msg.finger_mode = 3
        cmd_msg.neck_mode = 3
        cmd_msg.arx_pos_left = [left[0], left[1], left[2], left[3], left[4], left[5], left[6]]
        cmd_msg.arx_pos_right = [right[0], right[1], right[2], right[3], right[4], right[5], right[6]]
        cmd_msg.arm_fm_left = [0., 0., 0., 0., 0., 0.]
        cmd_msg.arm_fm_right = [0., 0., 0., 0., 0., 0.]
        
        MAX_Q = 48.0
        left[7] = left[7] if left[7] > 0 else 0
        left[7] = left[7] if left[7] < 1 else 1
        hand_q_left = left[7] * 90.0 
        hand_q_left = hand_q_left**1.2
        hand_q_left = MAX_Q if hand_q_left > MAX_Q else hand_q_left
        # print(f"{hand_q_left = }")
        cmd_msg.hand_q_left = [hand_q_left]
        
        # cmd_msg.hand_q_left = [left[7] * 90.0]
        
        
        right[7] = right[7] if right[7] > 0 else 0
        right[7] = right[7] if right[7] < 1 else 1
        hand_q_righ = right[7] * 90.0
        hand_q_righ = hand_q_righ**1.2
        hand_q_righ = MAX_Q if hand_q_righ > MAX_Q else hand_q_righ
        # print(f"{hand_q_righ = }")
        cmd_msg.hand_q_right = [hand_q_righ]
        
        # cmd_msg.hand_q_right = [right[7] * 90.0]

        cmd_msg.neck_q = [0.05, 0.0]
        cmd_msg.waist_q = [0.1, 0.0, 0.0]
        # print("-"*20+f"hand_right:{right[7]}")
        # print(f"cmd_msg:{cmd_msg}")
        global cauchy_rdt_cnt, cauchy_pub_cnt, cauchy_pub_time
        cauchy_pub_cnt += 1
        print("\033[1;31m" + f"RDT=[{last_rdt_inference_time:.3f} s]" + "\033[0m", end="")
        print("\033[92m" + f"[{cauchy_rdt_cnt}], PUB=[{cauchy_pub_cnt}][{1000*(time.time()-cauchy_pub_time):.1f} ms]" + "\033[0m", end=" ")
        # print()
        print("L: %+.2f,%+.2f,%+.2f,%+.2f,%+.2f,%+.2f,%+.2f,[%+.2f], R: %+.2f,%+.2f,%+.2f,%+.2f,%+.2f,%+.2f,%+.2f,[%+.2f]"%(left[0], left[1], left[2], left[3], left[4], left[5], left[6],cmd_msg.hand_q_left[0],right[0], right[1], right[2], right[3], right[4], right[5], right[6], cmd_msg.hand_q_right[0]))
        # str = input("Press ENTER to continue")
        cmd_dict = {
            "in_charge": 1,
            "filt_level": 1,
            "arm_mode": 3,
            "finger_mode": 3,
            "neck_mode": 3,
            "arx_pos_left": [round(x, 4) for x in left[:7]],
            "arx_pos_right": [round(x, 4) for x in right[:7]],
            "arm_fm_left": [0.0] * 6,
            "arm_fm_right": [0.0] * 6,
            "hand_q_left": round(hand_q_left, 4),
            "hand_q_right": round(hand_q_righ, 4),
            "neck_q": [0.05, 0.0],
            "waist_q": [0.1, 0.0, 0.0],
            "timestamp": time.time(),
            "sequence": cauchy_pub_cnt
        }
        self.Udpcmd.send_json(cmd_dict)
        # self.cmd_publisher.publish(cmd_msg)   # pub debug
        cauchy_pub_time = time.time()

    def robot_base_publish(self, vel):
        vel_msg = Twist()
        vel_msg.linear.x = vel[0]
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = vel[1]
        self.robot_base_publisher.publish(vel_msg)

    def puppet_arm_publish_continuous(self, left, right):
        print(f"self.args.publish_rate:{self.args.publish_rate}")
        rate = self.node.create_rate(self.args.publish_rate)

        left_arm = None
        right_arm = None
        while rclpy.ok():
            
            if len(self.puppet_arm_left_deque) != 0:
                left_arm = list(self.puppet_arm_left_deque[-1].position)
            if len(self.puppet_arm_right_deque) != 0:
                right_arm = list(self.puppet_arm_right_deque[-1].position)
            if left_arm is None or right_arm is None:
                rate.sleep()
                continue
            else:
                break
        left_symbol = [1 if left[i] - left_arm[i] > 0 else -1 for i in range(len(left))]
        right_symbol = [1 if right[i] - right_arm[i] > 0 else -1 for i in range(len(right))]
        flag = True
        step = 0
        while flag and rclpy.ok():
            if self.puppet_arm_publish_lock.acquire(False):
                return
            left_diff = [abs(left[i] - left_arm[i]) for i in range(len(left))]
            right_diff = [abs(right[i] - right_arm[i]) for i in range(len(right))]
            flag = False
            for i in range(len(left)):
                if left_diff[i] < self.args.arm_steps_length[i]:
                    left_arm[i] = left[i]
                else:
                    left_arm[i] += left_symbol[i] * self.args.arm_steps_length[i]
                    flag = True
            for i in range(len(right)):
                if right_diff[i] < self.args.arm_steps_length[i]:
                    right_arm[i] = right[i]
                else:
                    right_arm[i] += right_symbol[i] * self.args.arm_steps_length[i]
                    flag = True
            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = self.node.get_clock().now().to_msg()
            joint_state_msg.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # 设置关节名称
            joint_state_msg.position = left_arm
            self.puppet_arm_left_publisher.publish(joint_state_msg)
            joint_state_msg.position = right_arm
            self.puppet_arm_right_publisher.publish(joint_state_msg)
            step += 1
            # print("puppet_arm_publish_continuous:", step)
            rate.sleep()

    def puppet_arm_publish_continuous_wrapper(self, left, right):
        # print(f"self.args.publish_rate:{self.args.publish_rate}")
        # print(f"{left = }, {right = }")
        rate = self.node.create_rate(self.args.publish_rate)
        left_arm = None
        right_arm = None
        while rclpy.ok():
            print(f"{len(self.puppet_arm_right_deque) = }")
            if len(self.puppet_arm_left_deque) != 0:
                left_arm = list(self.puppet_arm_left_deque[-1].position)
            if len(self.puppet_arm_right_deque) != 0:
                right_arm = list(self.puppet_arm_right_deque[-1].position)
            if left_arm is None or right_arm is None:
                rate.sleep()
                continue
            else:
                break
        # print(f"left_arm:{left_arm}")
        # print(f"right_arm:{right_arm}")

        left_symbol = [1 if left[i] - left_arm[i] > 0 else -1 for i in range(len(left))]
        right_symbol = [1 if right[i] - right_arm[i] > 0 else -1 for i in range(len(right))]
        flag = True
        step = 0
        while flag and rclpy.ok():
            if self.puppet_arm_publish_lock.acquire(False):
                return
            left_diff = [abs(left[i] - left_arm[i]) for i in range(len(left))]
            right_diff = [abs(right[i] - right_arm[i]) for i in range(len(right))]
            flag = False
            for i in range(len(left)):
                if left_diff[i] < self.args.arm_steps_length[i]:
                    left_arm[i] = left[i]
                else:
                    left_arm[i] += left_symbol[i] * self.args.arm_steps_length[i]
                    flag = True
            for i in range(len(right)):
                if right_diff[i] < self.args.arm_steps_length[i]:
                    right_arm[i] = right[i]
                else:
                    right_arm[i] += right_symbol[i] * self.args.arm_steps_length[i]
                    flag = True
            
            cmd_msg = CmdImitate()
            cmd_msg.in_charge = 1
            cmd_msg.filt_level = 1
            cmd_msg.arm_mode = 3
            cmd_msg.finger_mode = 3
            cmd_msg.neck_mode = 3
            cmd_msg.arx_pos_left = [left_arm[0], left_arm[1], left_arm[2], left_arm[3], left_arm[4], left_arm[5], left_arm[6]]
            cmd_msg.arx_pos_right = [right_arm[0], right_arm[1], right_arm[2], right_arm[3], right_arm[4], right_arm[5], right_arm[6]]
            cmd_msg.arm_fm_left = [0., 0., 0., 0., 0., 0.]
            cmd_msg.arm_fm_right = [0., 0., 0., 0., 0., 0.]
            cmd_msg.hand_q_left = [left_arm[7]]
            cmd_msg.hand_q_right = [right_arm[7]]
            cmd_msg.neck_q = [0.05, 0.0]
            cmd_msg.waist_q = [0.1, 0.0, 0.0]
            # print(f"cmd_msg:{cmd_msg}")
            # str = input("Press ENTER to continue")
            self.cmd_publisher.publish(cmd_msg)   # pub
            step += 1
            # print("puppet_arm_publish_continuous:", step)
            rate.sleep()

    def puppet_arm_publish_linear(self, left, right):
        num_step = 100
        rate = self.node.create_rate(200)

        left_arm = None
        right_arm = None

        while rclpy.ok():
            if len(self.puppet_arm_left_deque) != 0:
                left_arm = list(self.puppet_arm_left_deque[-1].position)
            if len(self.puppet_arm_right_deque) != 0:
                right_arm = list(self.puppet_arm_right_deque[-1].position)
            if left_arm is None or right_arm is None:
                rate.sleep()
                continue
            else:
                break

        traj_left_list = np.linspace(left_arm, left, num_step)
        traj_right_list = np.linspace(right_arm, right, num_step)

        for i in range(len(traj_left_list)):
            traj_left = traj_left_list[i]
            traj_right = traj_right_list[i]
            traj_left[-1] = left[-1]
            traj_right[-1] = right[-1]
            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = rospy.Time.now()  # 设置时间戳
            joint_state_msg.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # 设置关节名称
            joint_state_msg.position = traj_left
            self.puppet_arm_left_publisher.publish(joint_state_msg)
            joint_state_msg.position = traj_right
            self.puppet_arm_right_publisher.publish(joint_state_msg)
            rate.sleep()

    def puppet_arm_publish_continuous_thread(self, left, right):
         if self.puppet_arm_publish_thread is not None:
            self.puppet_arm_publish_lock.release()
            self.puppet_arm_publish_thread.join()
            self.puppet_arm_publish_lock.acquire(False)
            self.puppet_arm_publish_thread = None
        # self.puppet_arm_publish_thread = threading.Thread(target=self.puppet_arm_publish_continuous_wrapper, args=(left, right))
        # self.puppet_arm_publish_thread.start()

    def get_frame(self):
        # print("get frame.")
        if len(self.img_left_deque) == 0 or len(self.img_right_deque) == 0 or len(self.img_front_deque) == 0 or \
                (self.args.use_depth_image and (len(self.img_left_depth_deque) == 0 or len(self.img_right_depth_deque) == 0 or len(self.img_front_depth_deque) == 0)) or \
                    len(self.puppet_arm_right_deque)==0 or len(self.puppet_arm_left_deque)==0:
            print("first false")
            return False
        if self.args.use_depth_image:
            # frame_time = min([self.img_left_deque[-1].header.stamp.to_sec(), self.img_right_deque[-1].header.stamp.to_sec(), self.img_front_deque[-1].header.stamp.to_sec(),
            #                   self.img_left_depth_deque[-1].header.stamp.to_sec(), self.img_right_depth_deque[-1].header.stamp.to_sec(), self.img_front_depth_deque[-1].header.stamp.to_sec()])
            frame_time = min([time_from_header(self.img_left_deque[-1].header), time_from_header(self.img_right_deque[-1].header), time_from_header(self.img_front_deque[-1].header),
                              time_from_header(self.img_left_depth_deque[-1].header), time_from_header(self.img_right_depth_deque[-1].header), time_from_header(self.img_front_depth_deque[-1].header)])
        else:
            frame_time = min([time_from_header(self.img_left_deque[-1].header), time_from_header(self.img_right_deque[-1].header), time_from_header(self.img_front_deque[-1].header), time_from_header(self.puppet_arm_right_deque[-1].header), time_from_header(self.puppet_arm_left_deque[-1].header)])
            # frame_time = min([time_from_header(self.img_left_deque[-1].header), time_from_header(self.img_right_deque[-1].header), time_from_header(self.img_front_deque[-1].header)])

        if len(self.puppet_arm_right_deque) == 0 or time_from_header(self.puppet_arm_right_deque[-1].header) < frame_time:
            print("puppet_arm_right_deque false")
            return False
        if len(self.puppet_arm_left_deque) == 0 or time_from_header(self.puppet_arm_left_deque[-1].header) < frame_time:
            print(f"puppet_arm_left_deque false")
            return False
        if len(self.img_left_deque) == 0 or time_from_header(self.img_left_deque[-1].header) < frame_time:
            print("img_left_deque false")
            return False
        if len(self.img_right_deque) == 0 or time_from_header(self.img_right_deque[-1].header) < frame_time:
            print("img_right_deque false")
            return False
        if len(self.img_front_deque) == 0 or time_from_header(self.img_front_deque[-1].header) < frame_time:
            print("img_front_deque false")
            return False

        # print("hello")
        if self.args.use_depth_image and (len(self.img_left_depth_deque) == 0 or time_from_header(self.img_left_depth_deque[-1].header) < frame_time):
            return False
        if self.args.use_depth_image and (len(self.img_right_depth_deque) == 0 or time_from_header(self.img_right_depth_deque[-1].header) < frame_time):
            return False
        if self.args.use_depth_image and (len(self.img_front_depth_deque) == 0 or time_from_header(self.img_front_depth_deque[-1].header) < frame_time):
            return False
        if self.args.use_robot_base and (len(self.robot_base_deque) == 0 or time_from_header(self.robot_base_deque[-1].header) < frame_time):
            return False

        while time_from_header(self.img_left_deque[0].header) < frame_time:
            self.img_left_deque.popleft()
        # img_left = self.img_left_deque.popleft().data
        # begin_time = time.time()
        img_np_arr = np.frombuffer(self.img_left_deque.popleft().data, np.uint8)
        img_bgr = cv2.imdecode(img_np_arr, cv2.IMREAD_COLOR)
        img_left = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        # print("\033[92m" + f"MJPG Decode time = {1000*(time.time() - begin_time):.2f} ms" + "\033[0m")
        
        # img_left = self.bridge.imgmsg_to_cv2(self.img_left_deque.popleft(), 'passthrough')

        while time_from_header(self.img_right_deque[0].header) < frame_time:
            self.img_right_deque.popleft()
        # img_right = self.img_right_deque.popleft().data
        # begin_time = time.time()
        img_np_arr = np.frombuffer(self.img_right_deque.popleft().data, np.uint8)
        img_bgr = cv2.imdecode(img_np_arr, cv2.IMREAD_COLOR)
        img_right = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        # print("\033[92m" + f"MJPG Decode time = {1000*(time.time() - begin_time):.2f} ms" + "\033[0m")
        
        # img_right = self.bridge.imgmsg_to_cv2(self.img_right_deque.popleft(), 'passthrough')

        while time_from_header(self.img_front_deque[0].header) < frame_time:
            self.img_front_deque.popleft()
        # img_front = self.img_front_deque.popleft().data
        
        # begin_time = time.time()
        img_np_arr = np.frombuffer(self.img_front_deque.popleft().data, np.uint8)
        img_bgr = cv2.imdecode(img_np_arr, cv2.IMREAD_COLOR)
        img_front = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        # img_front = self.bridge.imgmsg_to_cv2(self.img_front_deque.popleft(), 'passthrough')
        # print("\033[92m" + f"MJPG Decode time = {1000*(time.time() - begin_time):.2f} ms" + "\033[0m")

        while time_from_header(self.puppet_arm_left_deque[0].header) < frame_time:
            self.puppet_arm_left_deque.popleft()
        puppet_arm_left = self.puppet_arm_left_deque.popleft()

        while time_from_header(self.puppet_arm_right_deque[0].header) < frame_time:
            self.puppet_arm_right_deque.popleft()
        puppet_arm_right = self.puppet_arm_right_deque.popleft()

        img_left_depth = None
        # if self.args.use_depth_image:
        #     while time_from_header(self.img_left_depth_deque[0].header) < frame_time:
        #         self.img_left_depth_deque.popleft()
        #     img_left_depth = self.bridge.imgmsg_to_cv2(self.img_left_depth_deque.popleft(), 'passthrough')

        img_right_depth = None
        # if self.args.use_depth_image:
        #     while time_from_header(self.img_right_depth_deque[0].header) < frame_time:
        #         self.img_right_depth_deque.popleft()
        #     img_right_depth = self.bridge.imgmsg_to_cv2(self.img_right_depth_deque.popleft(), 'passthrough')

        img_front_depth = None
        # if self.args.use_depth_image:
        #     while time_from_header(self.img_front_depth_deque[0].header) < frame_time:
        #         self.img_front_depth_deque.popleft()
        #     img_front_depth = self.bridge.imgmsg_to_cv2(self.img_front_depth_deque.popleft(), 'passthrough')

        robot_base = None
        # if self.args.use_robot_base:
        #     while time_from_header(self.robot_base_deque[0].header) < frame_time:
        #         self.robot_base_deque.popleft()
        #     robot_base = self.robot_base_deque.popleft()
        first_time = frame_time
        return (img_front, img_left, img_right, img_front_depth, img_left_depth, img_right_depth,
                puppet_arm_left, puppet_arm_right, robot_base, first_time)

    def img_left_callback(self, msg):
        # print("1. img_left_callback")
        # if len(self.img_left_deque) >= self.deque_length:
        #     self.img_left_deque.popleft()
        self.img_left_deque.append(msg)
        # self.dump_header[0] += f"{time_from_header(msg.header)},\n"

    def img_right_callback(self, msg):
        # print("2. img_right_callback")
        # if len(self.img_right_deque) >= self.deque_length:
        #     self.img_right_deque.popleft()
        self.img_right_deque.append(msg)
        # self.dump_header[1] += f"{time_from_header(msg.header)},\n"

    def img_front_callback(self, msg):
        # print("3. img_front_callback")
        # if len(self.img_front_deque) >= self.deque_length:
        #     self.img_front_deque.popleft()
        self.img_front_deque.append(msg)
        # self.dump_header[2] += f"{time_from_header(msg.header)},\n"

    def img_left_depth_callback(self, msg):
        if len(self.img_left_depth_deque) >= self.deque_length:
            self.img_left_depth_deque.popleft()
        self.img_left_depth_deque.append(msg)

    def img_right_depth_callback(self, msg):
        if len(self.img_right_depth_deque) >= self.deque_length:
            self.img_right_depth_deque.popleft()
        self.img_right_depth_deque.append(msg)

    def img_front_depth_callback(self, msg):
        if len(self.img_front_depth_deque) >= self.deque_length:
            self.img_front_depth_deque.popleft()
        self.img_front_depth_deque.append(msg)

    def puppet_arm_left_callback(self, msg):
        # print("4. puppet_arm_left_callback")
        if len(self.puppet_arm_left_deque) >= self.deque_length:
            self.puppet_arm_left_deque.popleft()
        self.puppet_arm_left_deque.append(msg)
        # self.dump_header[3] += f"{time_from_header(msg.header)},\n"

    def puppet_arm_right_callback(self, msg):
        # print("5. puppet_arm_right_callbackk")
        # if len(self.puppet_arm_right_deque) >= self.deque_length:
        #     self.puppet_arm_right_deque.popleft()
        self.puppet_arm_right_deque.append(msg)
        # self.dump_header[4] += f"{time_from_header(msg.header)},\n"

    def robot_base_callback(self, msg):
        # if len(self.robot_base_deque) >= self.deque_length:
        #     self.robot_base_deque.popleft()
        self.robot_base_deque.append(msg)

    # def arm_wrapper_callback(self, msg):
    #     # print("arm_wrapper_callback")
    #     # if len(self.puppet_arm_left_deque) >= self.deque_length:
    #     #     self.puppet_arm_left_deque.popleft()
    #     # if len(self.puppet_arm_right_deque) >= self.deque_length:
    #     #     self.puppet_arm_right_deque.popleft()
    #     # arm_left_msg = msg.limbs[6]
    #     # arm_right_msg = msg.limbs[7]
    #     # self.puppet_arm_left_deque.append(arm_left_msg)
    #     # self.puppet_arm_right_deque.append(arm_right_msg)
    #     self.drv_msg_latest = msg

    def init_ros(self):
        rclpy.init()
        self.node = rclpy.create_node("rdt_node")
        self.node_pub = rclpy.create_node("rdt_node_pub")
        sub_deque_len = 2
        # sub: best effort
        qos_profile = QoSProfile(
            depth=sub_deque_len,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT  # 可靠性策略：可靠传输
            # reliability=ReliabilityPolicy.RELIABLE
        )
        self.subscription = []
        # self.subscription.append(self.node.create_subscription(CompressedImage, self.args.img_left_topic, self.img_left_callback, qos_profile))
        # self.subscription.append(self.node.create_subscription(CompressedImage, self.args.img_right_topic, self.img_right_callback, qos_profile))
        # self.subscription.append(self.node.create_subscription(CompressedImage, self.args.img_front_topic, self.img_front_callback, qos_profile))
        
        # if self.args.use_depth_image:
        #     self.subscription.append(self.node.create_subscription(Image, self.args.img_left_depth_topic, self.img_left_depth_callback, sub_deque_len))
        #     self.subscription.append(self.node.create_subscription(Image, self.args.img_right_depth_topic, self.img_right_depth_callback, sub_deque_len))
        #     self.subscription.append(self.node.create_subscription(Image, self.args.img_front_depth_topic, self.img_front_depth_callback, sub_deque_len))
        self.subscription.append(self.node.create_subscription(JointState, self.args.puppet_arm_left_topic, self.puppet_arm_left_callback, qos_profile))
        self.subscription.append(self.node.create_subscription(JointState, self.args.puppet_arm_right_topic, self.puppet_arm_right_callback, qos_profile))
        # self.subscription.append(self.node.create_subscription(Odometry, self.args.robot_base_topic, self.robot_base_callback, sub_deque_len))
        # self.node_pub.create_publisher(JointState, self.args.puppet_arm_left_cmd_topic, 10)
        # self.node_pub.create_publisher(JointState, self.args.puppet_arm_right_cmd_topic, 10)
        self.node.create_publisher(Twist, self.args.robot_base_cmd_topic, 10)
        # self.subscription.append(self.node.create_subscription(JointState, self.args.arm_wrapper_topic, self.arm_wrapper_callback, sub_deque_len))
        self.cmd_publisher = self.node_pub.create_publisher(CmdImitate, self.args.drv_cmd_topic, 10)
        
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(self.node)
        # self.ros_node_thread = threading.Thread(target=executor.spin)
        # executor2 = MultiThreadedExecutor(num_threads=2)
        executor.add_node(self.node_pub)
        self.ros_node_thread = threading.Thread(target=executor.spin)
        
        # self.ros_node_thread = threading.Thread(target=rclpy.spin, args=(self.node,))      
        # self.ros_node_pub_thread = threading.Thread(target=rclpy.spin, args=(self.node_pub,))
        
        self.ros_node_thread.start()
        # self.ros_node_pub_thread.start()



def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--max_publish_step', action='store', type=int, 
                        help='Maximum number of action publishing steps', default=1000000, required=False)
    parser.add_argument('--seed', action='store', type=int, 
                        help='Random seed', default=None, required=False)

    parser.add_argument('--img_front_topic', action='store', type=str, help='img_front_topic',
                        default='/sampler/head/color/image_raw/compressed', required=False)
    parser.add_argument('--img_left_topic', action='store', type=str, help='img_left_topic',
                        default='/sampler/hand_left/color/image_rect_raw/compressed', required=False)
    parser.add_argument('--img_right_topic', action='store', type=str, help='img_right_topic',
                        default='/sampler/hand_right/color/image_rect_raw/compressed', required=False)
    
    parser.add_argument('--img_front_depth_topic', action='store', type=str, help='img_front_depth_topic',
                        default='/sampler/head/depth/image_rect_raw', required=False)
    parser.add_argument('--img_left_depth_topic', action='store', type=str, help='img_left_depth_topic',
                        default='/sampler/hand_left/depth/image_rect_raw', required=False)
    parser.add_argument('--img_right_depth_topic', action='store', type=str, help='img_right_depth_topic',
                        default='/sampler/hand_right/depth/image_rect_raw', required=False)
    
    parser.add_argument('--puppet_arm_left_cmd_topic', action='store', type=str, help='puppet_arm_left_cmd_topic',
                        default='/master/joint_left', required=False)
    parser.add_argument('--puppet_arm_right_cmd_topic', action='store', type=str, help='puppet_arm_right_cmd_topic',
                        default='/master/joint_right', required=False)
    parser.add_argument('--puppet_arm_left_topic', action='store', type=str, help='puppet_arm_left_topic',
                        default='/puppet/joint_left', required=False)
    parser.add_argument('--puppet_arm_right_topic', action='store', type=str, help='puppet_arm_right_topic',
                        default='/puppet/joint_right', required=False)
    
    parser.add_argument('--robot_base_topic', action='store', type=str, help='robot_base_topic',
                        default='/odom_raw', required=False)
    parser.add_argument('--robot_base_cmd_topic', action='store', type=str, help='robot_base_topic',
                        default='/cmd_vel', required=False)
    parser.add_argument('--use_robot_base', action='store_true', 
                        help='Whether to use the robot base to move around',
                        default=False, required=False)
    parser.add_argument('--publish_rate', action='store', type=int, 
                        help='The rate at which to publish the actions',
                        default=30, required=False)
    parser.add_argument('--ctrl_freq', action='store', type=int, 
                        help='The control frequency of the robot',
                        default=25, required=False)
    
    parser.add_argument('--chunk_size', action='store', type=int, 
                        help='Action chunk size',
                        default=64, required=False)
    parser.add_argument('--arm_steps_length', action='store', type=float, 
                        help='The maximum change allowed for each joint per timestep',
                        default=[0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.2], required=False)

    parser.add_argument('--use_actions_interpolation', action='store_true',
                        help='Whether to interpolate the actions if the difference is too large',
                        default=False, required=False)
    parser.add_argument('--use_depth_image', action='store_true', 
                        help='Whether to use depth images',
                        default=False, required=False)
    
    parser.add_argument('--disable_puppet_arm', action='store_true',
                        help='Whether to disable the puppet arm. This is useful for safely debugging',default=False)
    
    parser.add_argument('--config_path', type=str, default="configs/base.yaml", 
                        help='Path to the config file')
    # parser.add_argument('--cfg_scale', type=float, default=2.0,
    #                     help='the scaling factor used to modify the magnitude of the control features during denoising')
    parser.add_argument('--pretrained_model_name_or_path', type=str, required=True, help='Name or path to the pretrained model')
    
    parser.add_argument('--lang_embeddings_path', type=str, required=True, 
                        help='Path to the pre-encoded language instruction embeddings')
    parser.add_argument('--arm_wrapper_topic', action='store', type=str, help='qlong_arm_wrapper_topic',
                        default='/driver_pvt', required=False)
    parser.add_argument('--drv_cmd_topic', action='store', type=str, help='qlong_arm_wrapper_topic',
                        default='/cmd_imitate', required=False)

    
    args = parser.parse_args()
    return args

    
ros_operator = None

def main():
    global ros_operator
    args = get_arguments()
    ros_operator = RosOperator(args)
    if args.seed is not None:
        set_seed(args.seed)
    config = get_config(args)
    model_inference(args, config, ros_operator)


if __name__ == '__main__':
    main()