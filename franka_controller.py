# franka_controller.py
# 简化版Franka控制器，专注于遥操作功能

import time
import numpy as np
import franky
from franky import *
from constants import FRANKA_ROBOT_IP, constants_K, FRANKA_HOME_POSITION, FRANKA_HOME_ORIENTATION # 假设IP地址在constants.py中定义

class FrankaController:
    def __init__(self, robot_ip=FRANKA_ROBOT_IP):
        """
        初始化Franka控制器
        
        Args:
            robot_ip: Franka机器人的IP地址
        """
        # 连接机器人和夹爪
        self.robot_ip = robot_ip
        self.robot = Robot(robot_ip)
        self.gripper = Gripper(robot_ip)
        
        # 测试夹爪连接
        try:
            test_width = self.gripper.width
            print(f"Gripper connected successfully. Current width: {test_width}")
        except Exception as e:
            print(f"Warning: Gripper connection test failed: {e}")
            print("This might indicate a hardware or network issue")
        
        # 设置动态因子（速度、加速度比例）- 可根据需要调整
        self.robot.relative_dynamics_factor = 0.10  # 5%的最大速度/加速度
                
        # 设置阻抗，使运动更平缓（根据需要调整数值）
        try:
            self.robot.set_cartesian_impedance([100*constants_K, 100*constants_K, 100*constants_K, 30*constants_K, 30*constants_K, 40*constants_K])
            self.robot.set_joint_impedance([50*constants_K, 50*constants_K, 50*constants_K, 50*constants_K, 30*constants_K, 20*constants_K, 10*constants_K])
            print("Impedance set: cartesian=[100,100,100,20,20,20], joint=[50,50,50,50,30,20,10]")
        except Exception as e:
            print(f"Warning: Failed to set impedance: {e}")
        

        # 状态跟踪
        self.last_gripper_pos = None
        self._gripper_fail_count = 0  # 连续失败计数，用于触发重连
        
        print(f"Franka controller connected to robot at {robot_ip}")

    def execute_action(self, action):
        """
        执行遥操作指令（异步模式）
        
        Args:
            action: dict containing:
                - 'arm_pos': np.array([x, y, z]) 目标位置 (meters)
                - 'arm_quat': np.array([x, y, z, w]) 目标四元数姿态  
                - 'gripper_pos': float or np.array 夹爪开合度 [0, 1]
        """
        try:
            # 1. 处理机械臂运动（异步执行）
            target_pose = Affine(action['arm_pos'], action['arm_quat'])
            motion = CartesianMotion(target_pose)
            


            # 异步执行 - 不等待运动完成，允许被新指令打断
            self.robot.move(motion, asynchronous=True)
            
            # 2. 处理夹爪运动
            gripper_pos = action['gripper_pos']
            if hasattr(gripper_pos, 'item'):  # 如果是numpy数组
                gripper_pos = gripper_pos.item()
            
            # 只有夹爪位置变化较大时才执行夹爪动作
            if self.last_gripper_pos is None or abs(gripper_pos - self.last_gripper_pos) > 0.05:
                self._control_gripper(gripper_pos)
                self.last_gripper_pos = gripper_pos
                
        except Exception as e:
            print(f"Error executing action: {e}")
            # 尝试恢复
            self.robot.recover_from_errors()
            raise

    def _control_gripper(self, gripper_pos):
        """内部方法：控制夹爪"""
        try:
            print(f"Controlling gripper to position: {gripper_pos}")
            
            if gripper_pos < 0.1:
                # 完全打开
                print("Opening gripper completely")
                self.gripper.open(speed=0.05)
            elif gripper_pos > 0.9:
                # 夹取物体（可能阻塞且易失败，控制速度/力度，并做失败计数）
                print("Grasping object")
                try:
                    self.gripper.grasp(0.0, speed=0.03, force=20.0)
                    self._gripper_fail_count = 0
                except Exception as grasp_err:
                    print(f"Gripper grasp command error: {grasp_err}")
                    self._gripper_fail_count += 1
                    # 不立刻重连，累计到阈值再重连，避免频繁卡顿
                    if self._gripper_fail_count >= 3:
                        raise
            else:
                # 移动到特定宽度
                # 将[0,1]映射到实际夹爪宽度[0.08m, 0m]
                target_width = (1.0 - gripper_pos) * 0.08
                print(f"Moving gripper to width: {target_width}")
                self.gripper.move(target_width, speed=0.05)
                
        except Exception as e:
            print(f"Gripper control error: {e}")
            # 连续失败达到阈值时再尝试重连，避免频繁阻塞
            if self._gripper_fail_count >= 3:
                try:
                    print("Attempting to reinitialize gripper connection...")
                    self.gripper = Gripper(self.robot_ip)
                    self._gripper_fail_count = 0
                    print("Gripper reinitialized successfully")
                except Exception as e2:
                    print(f"Failed to reinitialize gripper: {e2}")
                

    def get_state(self):
        """
        获取当前机器人状态
        
        Returns:
            dict: 包含当前位姿和夹爪状态
        """
        try:
            # 获取笛卡尔状态
            cartesian_state = self.robot.current_cartesian_state
            end_effector_pose = cartesian_state.pose.end_effector_pose
            
            # 提取位置（适配 numpy 数组或对象）
            if isinstance(end_effector_pose.translation, np.ndarray):
                arm_pos = end_effector_pose.translation
            else:
                arm_pos = np.array([
                    end_effector_pose.translation.x,
                    end_effector_pose.translation.y,
                    end_effector_pose.translation.z
                ])
            
            # 提取姿态（适配 numpy 数组或对象）
            if isinstance(end_effector_pose.quaternion, np.ndarray):
                arm_quat = end_effector_pose.quaternion
            else:
                arm_quat = np.array([
                    end_effector_pose.quaternion.x,
                    end_effector_pose.quaternion.y, 
                    end_effector_pose.quaternion.z,
                    end_effector_pose.quaternion.w
                ])
            
            # 获取夹爪状态
            try:
                gripper_width = self.gripper.width
                print(f"Current gripper width: {gripper_width}")
                # 将夹爪宽度转换为[0,1]范围
                gripper_pos = np.array([1.0 - gripper_width / 0.08])
                print(f"Converted gripper position: {gripper_pos}")
            except Exception as e:
                print(f"Error getting gripper width: {e}")
                gripper_pos = np.array([0.0])
            
            return {
                'arm_pos': arm_pos,
                'arm_quat': arm_quat,
                'gripper_pos': gripper_pos
            }
            
        except Exception as e:
            print(f"Error getting state: {e}")
            return None


    def reset(self):
        """重置机器人到安全位置"""
        try:
            # 恢复错误状态
            self.robot.recover_from_errors()
            
            # 打开夹爪
            self.gripper.open(speed=0.05)
            
            # 移动到安全的home位置（使用常量配置）
            home_pos = np.array(FRANKA_HOME_POSITION)
            home_quat = np.array(FRANKA_HOME_ORIENTATION)
            
            home_action = {
                'arm_pos': home_pos,
                'arm_quat': home_quat, 
                'gripper_pos': 0.0
            }
            
            # 同步执行reset（确保完全到位）
            target_pose = Affine(home_pos, home_quat)
            motion = CartesianMotion(RobotPose(target_pose,elbow_state=ElbowState(0.0)))
            self.robot.move(motion)  # 同步执行


            self.last_gripper_pos = 0.0
            print("Robot reset to home position")
            
        except Exception as e:
            print(f"Error during reset: {e}")
            raise

    def set_dynamics_factor(self, factor):
        """
        设置运动动态因子
        
        Args:
            factor: float (0.0-1.0) 或 RelativeDynamicsFactor对象
        """
        try:
            self.robot.relative_dynamics_factor = factor
            print(f"Dynamics factor set to {factor}")
        except Exception as e:
            print(f"Error setting dynamics factor: {e}")

    def emergency_stop(self):
        """紧急停止"""
        try:
            self.robot.recover_from_errors()
            print("Emergency stop executed")
        except Exception as e:
            print(f"Error during emergency stop: {e}")


    def close(self):
        try:
            ##还非得用hasattr,不然会raise EOFError和已中止 (核心已转储)
            # 等待所有异步运动完成
            if hasattr(self.robot, 'join_motion'):
                self.robot.join_motion()
            
            # 关闭机器人连接
            if hasattr(self.robot, 'close'):
                self.robot.close()
            
            # 关闭夹爪连接
            if hasattr(self.gripper, 'close'):
                self.gripper.close()
                
            print("Franka controller closed")
        except Exception as e:
            print(f"Error closing Franka controller: {e}")
    

    def test_gripper(self):
        """测试夹爪功能"""
        try:
            print("Testing gripper functionality...")
            
            # 测试打开
            print("1. Testing open...")
            self.gripper.open(speed=0.05)
            time.sleep(2)
            print(f"   Current width: {self.gripper.width}")
            
            # 测试移动到中间位置
            print("2. Testing move to middle...")
            self.gripper.move(0.04, speed=0.05)
            time.sleep(2)
            print(f"   Current width: {self.gripper.width}")
            
            # 测试夹取
            print("3. Testing grasp...")
            self.gripper.grasp(0.0, speed=0.05, force=20.0)
            time.sleep(2)
            print(f"   Current width: {self.gripper.width}")
            
            print("Gripper test completed successfully!")
            
        except Exception as e:
            print(f"Gripper test failed: {e}")

# 测试代码
if __name__ == '__main__':
    controller = FrankaController(FRANKA_ROBOT_IP)  # 替换为实际IP
    
    try:
        # 重置
        controller.reset()
        
        # 测试夹爪功能
        print("Testing gripper functionality...")
        controller.test_gripper()
        
        # 测试基本运动
        test_actions = [
            {
                'arm_pos': np.array([0.2, 0.0, 0.4]),
                'arm_quat': np.array([1.0, 0.0, 0.0, 0.0]),
                'gripper_pos': 0.0
            },
            {
                'arm_pos': np.array([0.4, -0.2, 0.2]),
                'arm_quat': np.array([0.707, 0.707 ,0.0, 0.0]), 
                'gripper_pos': 1.0
            }
        ]
        
        for i, action in enumerate(test_actions):
            print(f"Executing action {i+1}")
            controller.execute_action(action)
            time.sleep(5.0)  # 等待运动
            
            # 获取状态
            state = controller.get_state()
            if state:
                print(f"Current pos: {state['arm_pos']}")
                print(f"Current quat: {state['arm_quat']}")
                print(f"Gripper: {state['gripper_pos']}")
        
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        controller.close()