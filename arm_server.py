# arm_server.py
# 简化版Franka机械臂RPC服务器

import time
from multiprocessing.managers import BaseManager as MPBaseManager
import numpy as np
from franka_controller import FrankaController
from constants import ARM_RPC_HOST, ARM_RPC_PORT, RPC_AUTHKEY, FRANKA_ROBOT_IP

class Arm:
    def __init__(self, robot_ip=FRANKA_ROBOT_IP):
        """
        Franka机械臂管理类
        
        Args:
            robot_ip: Franka机器人IP地址
        """
        self.robot_ip = robot_ip
        self.controller = None
        print(f"Arm manager initialized for Franka robot at {robot_ip}")

    def reset(self):
        """重置机械臂"""
        try:
            # 关闭现有连接
            if self.controller:
                self.controller.close()
            
            # 创建新的控制器
            self.controller = FrankaController(self.robot_ip)
            
            # 重置到安全位置
            self.controller.reset()
            
            print("Arm reset completed successfully")
            
        except Exception as e:
            print(f"Error during arm reset: {e}")
            raise

    def execute_action(self, action):
        """
        执行动作指令
        
        Args:
            action: dict containing:
                - 'arm_pos': np.array([x, y, z]) 目标位置
                - 'arm_quat': np.array([x, y, z, w]) 目标四元数  
                - 'gripper_pos': np.array([pos]) or float 夹爪位置
        """
        if self.controller is None:
            raise RuntimeError("Controller not initialized. Call reset() first.")
        
        try:
            self.controller.execute_action(action)
        except Exception as e:
            print(f"Error executing action: {e}")
            raise

    def get_state(self):
        """获取当前机械臂状态"""
        if self.controller is None:
            raise RuntimeError("Controller not initialized. Call reset() first.")
        
        try:
            state = self.controller.get_state()
            if state is None:
                raise RuntimeError("Failed to get robot state")
            return state
        except Exception as e:
            print(f"Error getting state: {e}")
            raise

    def set_dynamics_factor(self, factor):
        """设置运动速度因子"""
        if self.controller is None:
            raise RuntimeError("Controller not initialized. Call reset() first.")
        
        try:
            self.controller.set_dynamics_factor(factor)
        except Exception as e:
            print(f"Error setting dynamics factor: {e}")
            raise

    def set_filter_alpha(self, alpha):
        """设置低通滤波系数"""
        if self.controller is None:
            raise RuntimeError("Controller not initialized. Call reset() first.")
        
        try:
            self.controller.set_filter_alpha(alpha)
        except Exception as e:
            print(f"Error setting filter alpha: {e}")
            raise

    def emergency_stop(self):
        """紧急停止"""
        if self.controller:
            try:
                self.controller.emergency_stop()
            except Exception as e:
                print(f"Error during emergency stop: {e}")

    def close(self):
        """关闭连接"""
        try:
            if self.controller:
                self.controller.close()
                self.controller = None
            print("Arm closed successfully")
        except Exception as e:
            print(f"Error closing arm: {e}")


class ArmManager(MPBaseManager):
    """多进程管理器"""
    pass


# 注册类到管理器
ArmManager.register('Arm', Arm)


def run_server():
    """运行RPC服务器"""
    try:
        manager = ArmManager(address=(ARM_RPC_HOST, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
        server = manager.get_server()
        print(f'Franka arm server started at {ARM_RPC_HOST}:{ARM_RPC_PORT}')
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nServer interrupted by user")
    except Exception as e:
        print(f"Server error: {e}")
    finally:
        print("Server shutdown")


def test_client():
    """测试客户端"""
    manager = ArmManager(address=(ARM_RPC_HOST, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
    manager.connect()
    arm = manager.Arm()
    
    try:
        print("Resetting arm...")
        arm.reset()
        
        # 设置较慢的速度
        arm.set_dynamics_factor(0.1)
        
        # 测试动作
        test_action = {
            'arm_pos': np.array([0.4, 0.2, 0.5]),
            'arm_quat': np.array([0.707, 0.707, 0.0, 0.0]),
            'gripper_pos': np.array([0.8])
        }
       

        test_action2 = {
            'arm_pos': np.array([0.4, 0.3, 0.4]),
            'arm_quat': np.array([0.707, 0.707, 0.0, 0.0]),
            'gripper_pos': np.array([0.8])
        }

        test_action3 = {
            'arm_pos': np.array([0.4, 0.35, 0.35]),
            'arm_quat': np.array([0.707, 0.707, 0.0, 0.0]),
            'gripper_pos': np.array([0.8])
        }


        test_action4 = {
            'arm_pos': np.array([0.4, 0.4, 0.3]),
            'arm_quat': np.array([0.707, 0.707, 0.0, 0.0]),
            'gripper_pos': np.array([0.8])
        }
        

        arm.execute_action(test_action)
        time.sleep(5)
        print("Executing test action...")
        arm.execute_action(test_action2)
        time.sleep(5)
        print("Executing test action2...")
        arm.execute_action(test_action3)
        time.sleep(5)
        print("Executing test action3...")
        arm.execute_action(test_action4)
        print("Executing test action4...")
        time.sleep(10)
        # 获取状态
        state = arm.get_state()
        print(f"Current state: {state}")
        
        print("Test completed successfully!")
        
    except Exception as e:
        print(f"Test error: {e}")
        arm.emergency_stop()
    finally:
        arm.close()


if __name__ == '__main__':
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == 'test':
        test_client()
    else:
        run_server()