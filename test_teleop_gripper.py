#!/usr/bin/env python3
"""
测试遥操作夹爪控制
"""

import time
import numpy as np
from real_env import RealEnv

def test_teleop_gripper():
    """测试遥操作夹爪控制"""
    print("=== 测试遥操作夹爪控制 ===")
    
    try:
        # 创建真实环境
        env = RealEnv()
        print("真实环境创建成功")
        
        # 重置环境
        env.reset()
        print("环境重置完成")
        
        # 获取初始状态
        obs = env.get_obs()
        print(f"初始夹爪位置: {obs['gripper_pos']}")
        
        # 测试夹爪控制
        print("\n开始测试夹爪控制...")
        
        # 测试打开夹爪
        print("1. 测试打开夹爪...")
        action = {
            'arm_pos': obs['arm_pos'].copy(),  # 保持当前位置
            'arm_quat': obs['arm_quat'].copy(),  # 保持当前姿态
            'gripper_pos': np.array([0.0])  # 完全打开
        }
        env.step(action)
        time.sleep(3)
        
        # 获取状态
        obs = env.get_obs()
        print(f"   打开后夹爪位置: {obs['gripper_pos']}")
        
        # 测试关闭夹爪
        print("2. 测试关闭夹爪...")
        action['gripper_pos'] = np.array([1.0])  # 完全关闭
        env.step(action)
        time.sleep(3)
        
        # 获取状态
        obs = env.get_obs()
        print(f"   关闭后夹爪位置: {obs['gripper_pos']}")
        
        # 测试中间位置
        print("3. 测试中间位置...")
        action['gripper_pos'] = np.array([0.5])  # 中间位置
        env.step(action)
        time.sleep(3)
        
        # 获取状态
        obs = env.get_obs()
        print(f"   中间位置夹爪位置: {obs['gripper_pos']}")
        
        # 测试连续变化
        print("4. 测试连续变化...")
        for i in range(5):
            pos = i / 4.0  # 0.0, 0.25, 0.5, 0.75, 1.0
            action['gripper_pos'] = np.array([pos])
            env.step(action)
            time.sleep(1)
            
            obs = env.get_obs()
            print(f"   位置 {pos:.2f}: 夹爪位置 {obs['gripper_pos']}")
        
        print("遥操作夹爪测试完成！")
        
    except Exception as e:
        print(f"测试失败: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            env.close()
        except:
            pass

if __name__ == '__main__':
    test_teleop_gripper()
