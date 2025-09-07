#!/usr/bin/env python3
"""
调试回放脚本 - 用于测试和改进真机回放效果
"""

import time
import numpy as np
from arm_server import ArmManager
from constants import ARM_RPC_HOST, ARM_RPC_PORT, RPC_AUTHKEY

def test_replay_settings():
    """测试不同的回放设置"""
    manager = ArmManager(address=(ARM_RPC_HOST, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
    manager.connect()
    arm = manager.Arm()
    
    try:
        print("重置机械臂...")
        arm.reset()
        
        # 设置较慢的速度和较平滑的滤波
        print("设置回放参数...")
        arm.set_dynamics_factor(0.03)  # 3% 速度
        arm.set_filter_alpha(0.2)      # 较平滑的滤波
        
        print("回放参数设置完成！")
        print("现在可以运行回放命令：")
        print("python replay_episodes.py --input-dir data/demos/你的序列文件夹")
        
    except Exception as e:
        print(f"设置失败: {e}")
    finally:
        arm.close()

if __name__ == '__main__':
    test_replay_settings()


