# Author: Jimmy Wu, Modified for Franka Research 3 by Grok  # 修改：作者信息更新
# Date: October 2024, Modified August 2025  # 修改：日期信息更新

import logging
# 删除：import math
import socket
import threading
import time
from queue import Queue
# 删除：import cv2 as cv
import numpy as np
import zmq
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from scipy.spatial.transform import Rotation as R
from constants import POLICY_SERVER_HOST, POLICY_SERVER_PORT  # 修改：删除了POLICY_IMAGE_WIDTH, POLICY_IMAGE_HEIGHT
from constants import FRANKA_TELEOP_POS_MIN, FRANKA_TELEOP_POS_MAX

# 位置与姿态的每步最大变化（速率限制）
MAX_POS_STEP_M = 0.003  # 每轴每周期最大 3 毫米
MAX_ROT_STEP_RAD = np.deg2rad(0.2)  # 每周期最大 2 度

class Policy:
    def reset(self):
        raise NotImplementedError

    def step(self, obs):
        raise NotImplementedError

class WebServer:
    def __init__(self, queue):
        self.app = Flask(__name__)
        self.socketio = SocketIO(self.app)
        self.queue = queue

        @self.app.route('/')
        def index():
            return render_template('index.html')

        @self.socketio.on('message')
        def handle_message(data):
            emit('echo', data['timestamp'])
            self.queue.put(data)

        logging.getLogger('werkzeug').setLevel(logging.WARNING)

    def run(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0)
        try:
            s.connect(('8.8.8.8', 1))
            address = s.getsockname()[0]
        except Exception:
            address = '127.0.0.1'
        finally:
            s.close()
        print(f'Starting server at {address}:5000')
        self.socketio.run(self.app, host='0.0.0.0')

DEVICE_CAMERA_OFFSET = np.array([0.0, 0.02, -0.04])  # iPhone 14 Pro

def convert_webxr_pose(pos, quat):
    pos = np.array([-pos['z'], -pos['x'], pos['y']], dtype=np.float64)
    rot = R.from_quat([-quat['z'], -quat['x'], quat['y'], quat['w']])
    pos = pos + rot.apply(DEVICE_CAMERA_OFFSET)
    return pos, rot

# 删除：TWO_PI = 2 * math.pi

class TeleopController:
    def __init__(self):
        self.primary_device_id = None
        # 删除：self.secondary_device_id = None
        self.enabled_counts = {}

        # 删除：self.base_pose = None

        self.targets_initialized = False
        # 删除：self.base_target_pose = None
        self.arm_target_pos = None
        self.arm_target_rot = None
        self.gripper_target_pos = None
        
        # 删除：self.base_xr_ref_pos = None
        # 删除：self.base_xr_ref_rot_inv = None
        self.arm_xr_ref_pos = None
        self.arm_xr_ref_rot_inv = None
        
        # 删除：self.base_ref_pose = None
        self.arm_ref_pos = None
        self.arm_ref_rot = None
        # 删除：self.arm_ref_base_pose = None
        self.gripper_ref_pos = None

    def process_message(self, data):
        if not self.targets_initialized:
            return

        device_id = data['device_id']
        self.enabled_counts[device_id] = self.enabled_counts.get(device_id, 0) + 1 if 'teleop_mode' in data else 0

        if self.enabled_counts[device_id] > 2:
            if self.primary_device_id is None:  # 修改：删除了 and device_id != self.secondary_device_id
                self.primary_device_id = device_id
            # 删除：elif self.secondary_device_id is None and device_id != self.primary_device_id:
            # 删除：    self.secondary_device_id = device_id
        elif self.enabled_counts[device_id] == 0:
            if device_id == self.primary_device_id:
                self.primary_device_id = None
                # 删除：self.base_xr_ref_pos = None
                self.arm_xr_ref_pos = None
            # 删除：elif device_id == self.secondary_device_id:
            # 删除：    self.secondary_device_id = None
            # 删除：    self.base_xr_ref_pos = None

        if self.primary_device_id is not None and 'teleop_mode' in data:
            pos, rot = convert_webxr_pose(data['position'], data['orientation'])

            # 删除：整个base movement部分的代码块

            if data['teleop_mode'] == 'arm':  # 修改：从elif改为if
                if self.arm_xr_ref_pos is None:
                    self.arm_xr_ref_pos = pos
                    self.arm_xr_ref_rot_inv = rot.inv()
                    self.arm_ref_pos = self.arm_target_pos.copy()
                    self.arm_ref_rot = self.arm_target_rot
                    # 删除：self.arm_ref_base_pose = self.base_pose.copy()
                    self.gripper_ref_pos = self.gripper_target_pos

                # 删除：所有z_rot相关的代码
                
                #限制行动范围
            
                # 期望位置（先限幅到工作空间，再做每步速率限制）
                desired_pos = np.clip(
                    self.arm_ref_pos + (pos - self.arm_xr_ref_pos),
                    FRANKA_TELEOP_POS_MIN,
                    FRANKA_TELEOP_POS_MAX,
                )

                # 位置每轴限速
                pos_delta = desired_pos - self.arm_target_pos
                limited_delta = np.clip(pos_delta, -MAX_POS_STEP_M, MAX_POS_STEP_M)
                self.arm_target_pos = self.arm_target_pos + limited_delta

                # 期望姿态（相对参考），随后做每步小角度插值
                desired_rot = rot * self.arm_xr_ref_rot_inv * self.arm_ref_rot
                # 当前到期望的相对旋转
                relative_rot = desired_rot * self.arm_target_rot.inv()
                rotvec = relative_rot.as_rotvec()
                angle = np.linalg.norm(rotvec)
                if angle > 1e-9:
                    step_factor = min(1.0, MAX_ROT_STEP_RAD / angle)
                    inc_rot = R.from_rotvec(rotvec * step_factor)
                    self.arm_target_rot = inc_rot * self.arm_target_rot
                # 若角度极小则忽略，保持当前姿态，避免抖动
                self.gripper_target_pos = np.clip(self.gripper_ref_pos + data['gripper_delta'], 0.0, 1.0)  # 修改：使用[0.0, 1.0]范围

        # 删除：elif self.primary_device_id is None的整个代码块

    def step(self, obs):
        # 删除：self.base_pose = obs['base_pose']

        if not self.targets_initialized:
            # 删除：self.base_target_pose = obs['base_pose']
            self.arm_target_pos = obs['arm_pos']
            self.arm_target_rot = R.from_quat(obs['arm_quat'])
            self.gripper_target_pos = obs['gripper_pos']
            self.targets_initialized = True

        if self.primary_device_id is None:
            return None

        arm_quat = self.arm_target_rot.as_quat()
        if arm_quat[3] < 0.0:
            np.negative(arm_quat, out=arm_quat)
        action = {
            # 删除：'base_pose': self.base_target_pose.copy(),
            'arm_pos': self.arm_target_pos.copy(),
            'arm_quat': arm_quat,
            'gripper_pos': self.gripper_target_pos.copy(),
        }
        return action

class TeleopPolicy(Policy):
    def __init__(self):
        self.web_server_queue = Queue()
        self.teleop_controller = None
        self.teleop_state = None
        self.episode_ended = False
        server = WebServer(self.web_server_queue)
        threading.Thread(target=server.run, daemon=True).start()
        threading.Thread(target=self.listener_loop, daemon=True).start()

    def reset(self):
        self.teleop_controller = TeleopController()
        self.episode_ended = False
        self.teleop_state = None
        while self.teleop_state != 'episode_started':
            time.sleep(0.01)

    def step(self, obs):
        if not self.episode_ended and self.teleop_state == 'episode_ended':
            self.episode_ended = True
            return 'end_episode'
        if self.teleop_state == 'reset_env':
            return 'reset_env'
        return self._step(obs)

    def _step(self, obs):
        return self.teleop_controller.step(obs)

    def listener_loop(self):
        while True:
            if not self.web_server_queue.empty():
                data = self.web_server_queue.get()
                if 'state_update' in data:
                    self.teleop_state = data['state_update']
                elif 1000 * time.time() - data['timestamp'] < 250:
                    self._process_message(data)
            time.sleep(0.001)

    def _process_message(self, data):
        self.teleop_controller.process_message(data)

class RemotePolicy(TeleopPolicy):
    def __init__(self):
        super().__init__()
        self.enabled = False
        context = zmq.Context()
        self.socket = context.socket(zmq.REQ)
        self.socket.connect(f'tcp://{POLICY_SERVER_HOST}:{POLICY_SERVER_PORT}')
        print(f'Connected to policy server at {POLICY_SERVER_HOST}:{POLICY_SERVER_PORT}')

    def reset(self):
        super().reset()
        default_timeout = self.socket.getsockopt(zmq.RCVTIMEO)
        self.socket.setsockopt(zmq.RCVTIMEO, 1000)
        self.socket.send_pyobj({'reset': True})
        try:
            self.socket.recv_pyobj()
        except zmq.error.Again as e:
            raise Exception('Could not communicate with policy server') from e
        self.socket.setsockopt(zmq.RCVTIMEO, default_timeout)
        self.enabled = False

    def _step(self, obs):
        if self.episode_ended:
            return self.teleop_controller.step(obs)
        if not self.enabled:
            return None
        # 删除：整个图像编码处理的代码块
        self.socket.send_pyobj({'obs': obs})  # 新增：直接发送obs，替代encoded_obs
        rep = self.socket.recv_pyobj()
        action = rep['action']
        return action

    def _process_message(self, data):
        if self.episode_ended:
            self.teleop_controller.process_message(data)
        else:
            self.enabled = 'teleop_mode' in data

if __name__ == '__main__':
    from constants import POLICY_CONTROL_PERIOD
    obs = {
        # 删除：'base_pose': np.zeros(3),
        'arm_pos': np.zeros(3),
        'arm_quat': np.array([0.0, 0.0, 0.0, 1.0]),
        'gripper_pos': np.zeros(1),
        # 删除：'base_image': np.zeros((640, 360, 3)),
        # 删除：'wrist_image': np.zeros((640, 480, 3)),
    }
    policy = TeleopPolicy()
    while True:
        policy.reset()
        for _ in range(1000):
            print(policy.step(obs))
            time.sleep(POLICY_CONTROL_PERIOD)