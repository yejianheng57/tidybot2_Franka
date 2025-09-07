import numpy as np

################################################################################
# Mobile base

# Vehicle center to steer axis (m)
h_x, h_y = 0.190150 * np.array([1.0, 1.0, -1.0, -1.0]), 0.170150 * np.array([-1.0, 1.0, 1.0, -1.0])  # Kinova / Franka
# h_x, h_y = 0.140150 * np.array([1.0, 1.0, -1.0, -1.0]), 0.120150 * np.array([-1.0, 1.0, 1.0, -1.0])  # ARX5

# Encoder magnet offsets
ENCODER_MAGNET_OFFSETS = [0.0 / 4096, 0.0 / 4096, 0.0 / 4096, 0.0 / 4096]  # TODO

################################################################################
# Teleop and imitation learning

# Base and arm RPC servers
BASE_RPC_HOST = 'localhost'
BASE_RPC_PORT = 50000
ARM_RPC_HOST = 'localhost'
ARM_RPC_PORT = 50001
RPC_AUTHKEY = b'secret password'

# 新增：Franka机器人相关常量
FRANKA_ROBOT_IP = "172.16.0.2"  # 替换为实际的Franka机器人IP
FRANKA_DEFAULT_DYNAMICS_FACTOR = 0.6  # 默认速度因子(没用，速度要去franka_controller.py里改)
FRANKA_HOME_POSITION = [0.307, 0.0, 0.487]  # 默认home位置
FRANKA_HOME_ORIENTATION = [1.0, 0.0, 0.0, 0.0]  # 默认home姿态

# 遥操作安全工作空间（基座坐标系，单位：m）
FRANKA_TELEOP_POS_MIN = np.array([-0.7, -0.7, -0.1])
FRANKA_TELEOP_POS_MAX = np.array([ 0.7,  0.7,  1.00])

# Cameras
BASE_CAMERA_SERIAL = 'TODO'
# WRIST_CAMERA_SERIAL = 'TODO'  # Not used by Kinova wrist camera

# Policy
POLICY_SERVER_HOST = 'localhost'
POLICY_SERVER_PORT = 5555
POLICY_CONTROL_FREQ = 100
POLICY_CONTROL_PERIOD = 1.0 / POLICY_CONTROL_FREQ
POLICY_IMAGE_WIDTH = 84
POLICY_IMAGE_HEIGHT = 84

#阻抗刚度
constants_K = 1
