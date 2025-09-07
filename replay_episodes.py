# Author: Jimmy Wu
# Date: October 2024

import argparse
import time
from itertools import count
from pathlib import Path
import cv2 as cv
from constants import POLICY_CONTROL_PERIOD
from episode_storage import EpisodeReader
from mujoco_env import MujocoEnv

def replay_episode(env, episode_dir, show_images=False, execute_obs=False, speed_factor=1.0, is_sim=False):
    # Reset env
    env.reset()

    # Load episode data
    reader = EpisodeReader(episode_dir)
    print(f'Loaded episode from {episode_dir}')
    print(f'Mode: {"Simulation" if is_sim else "Real robot"}')
    print(f'Replay speed factor: {speed_factor}x')

    start_time = time.time()
    for step_idx, (obs, action) in enumerate(zip(reader.observations, reader.actions)):
        # Enforce desired control freq with speed factor
        step_end_time = start_time + step_idx * POLICY_CONTROL_PERIOD / speed_factor
        while time.time() < step_end_time:
            time.sleep(0.0001)

        # Show image observations
        if show_images:
            window_idx = 0
            for k, v in obs.items():
                if v.ndim == 3:
                    cv.imshow(k, cv.cvtColor(v, cv.COLOR_RGB2BGR))
                    cv.moveWindow(k, 640 * window_idx, 0)
                    window_idx += 1
            cv.waitKey(1)

        # Execute in action in env
        if execute_obs:
            env.step(obs)
        else:
            env.step(action)

def main(args):
    # Create env
    if args.sim:
        env = MujocoEnv(render_images=False)
        # 仿真默认速度：可以较快，便于观察
        default_speed = 1.0
    else:
        from real_env import RealEnv
        env = RealEnv()
        # 真机默认速度：较慢，确保安全
        default_speed = 0.3

    # 确定使用的速度因子
    if args.sim:
        # 仿真模式：优先使用sim_speed_factor，否则使用speed_factor，最后使用默认值
        speed_factor = args.sim_speed_factor if args.sim_speed_factor is not None else (args.speed_factor if args.speed_factor != 1.0 else default_speed)
    else:
        # 真机模式：优先使用real_speed_factor，否则使用speed_factor，最后使用默认值
        speed_factor = args.real_speed_factor if args.real_speed_factor is not None else (args.speed_factor if args.speed_factor != 1.0 else default_speed)

    try:
        input_path = Path(args.input_dir)
        
        # 检查是否包含 data.pkl（单个序列目录）
        if (input_path / 'data.pkl').exists():
            # 单个序列目录
            episode_dirs = [input_path]
        else:
            # 父目录，查找所有子目录
            episode_dirs = sorted([child for child in input_path.iterdir() if child.is_dir()])
        
        if not episode_dirs:
            print(f"No episodes found in {args.input_dir}")
            return
            
        for episode_dir in episode_dirs:
            replay_episode(env, episode_dir, show_images=args.show_images, execute_obs=args.execute_obs, speed_factor=speed_factor, is_sim=args.sim)
            # input('Press <Enter> to continue...')
    finally:
        env.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-dir', default='data/demos')
    parser.add_argument('--sim', action='store_true')
    parser.add_argument('--show-images', action='store_true')
    parser.add_argument('--execute-obs', action='store_true')
    parser.add_argument('--speed-factor', type=float, default=1.0, help='General replay speed factor (e.g., 0.5 for half speed, 2.0 for double speed)')
    parser.add_argument('--sim-speed-factor', type=float, default=None, help='Simulation replay speed factor (default: 1.0)')
    parser.add_argument('--real-speed-factor', type=float, default=None, help='Real robot replay speed factor (default: 0.3)')
    main(parser.parse_args())
