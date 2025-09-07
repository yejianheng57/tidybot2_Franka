#!/usr/bin/env python3
"""
简单的夹爪测试脚本
用于独立测试Franka夹爪功能
"""

import time
import numpy as np
from franka_controller import FrankaController
from constants import FRANKA_ROBOT_IP

def test_gripper_basic():
    """基本夹爪测试"""
    print("=== 基本夹爪测试 ===")
    
    try:
        # 创建控制器
        controller = FrankaController(FRANKA_ROBOT_IP)
        print("控制器创建成功")
        
        # 测试夹爪功能
        controller.test_gripper()
        
        # 测试通过execute_action控制夹爪
        print("\n=== 通过execute_action测试夹爪 ===")
        
        # 测试打开夹爪
        print("测试打开夹爪...")
        action = {
            'arm_pos': np.array([0.307, 0.0, 0.487]),  # 保持当前位置
            'arm_quat': np.array([1.0, 0.0, 0.0, 0.0]),
            'gripper_pos': 0.0  # 完全打开
        }
        controller.execute_action(action)
        time.sleep(3)
        
        # 测试关闭夹爪
        print("测试关闭夹爪...")
        action['gripper_pos'] = 1.0  # 完全关闭
        controller.execute_action(action)
        time.sleep(3)
        
        # 测试中间位置
        print("测试中间位置...")
        action['gripper_pos'] = 0.5  # 中间位置
        controller.execute_action(action)
        time.sleep(3)
        
        print("夹爪测试完成！")
        
    except Exception as e:
        print(f"测试失败: {e}")
    finally:
        try:
            controller.close()
        except:
            pass

def test_gripper_direct():
    """直接夹爪API测试"""
    print("=== 直接夹爪API测试 ===")
    
    try:
        # 创建控制器
        controller = FrankaController(FRANKA_ROBOT_IP)
        print("控制器创建成功")
        
        # 直接测试夹爪API
        gripper = controller.gripper
        
        print("1. 测试获取夹爪宽度...")
        width = gripper.width
        print(f"   当前夹爪宽度: {width}")
        
        print("2. 测试打开夹爪...")
        gripper.open(speed=0.05)
        time.sleep(2)
        width = gripper.width
        print(f"   打开后夹爪宽度: {width}")
        
        print("3. 测试移动到特定宽度...")
        gripper.move(0.04, speed=0.05)
        time.sleep(2)
        width = gripper.width
        print(f"   移动到0.04后夹爪宽度: {width}")
        
        print("4. 测试夹取...")
        gripper.grasp(0.0, speed=0.05, force=20.0)
        time.sleep(2)
        width = gripper.width
        print(f"   夹取后夹爪宽度: {width}")
        
        print("直接API测试完成！")
        
    except Exception as e:
        print(f"直接API测试失败: {e}")
    finally:
        try:
            controller.close()
        except:
            pass

if __name__ == '__main__':
    print("Franka夹爪测试脚本")
    print("=" * 50)
    
    # 运行基本测试
    test_gripper_basic()
    
    print("\n" + "=" * 50)
    
    # 运行直接API测试
    test_gripper_direct()
    
    print("\n测试完成！")
