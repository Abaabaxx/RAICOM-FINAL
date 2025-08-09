#! /usr/bin/env python3

import subprocess
import os

# 定义 launch 文件的绝对路径
launch_file_path = '/home/lby/tianbot_ws/src/f1/launch/ftg.launch'

# 确保文件路径存在
if not os.path.exists(launch_file_path):
    print(f"错误: launch 文件不存在于路径 {launch_file_path}")
else:
    # 使用 subprocess.call 来执行命令
    # 这会阻塞 Python 脚本，直到 roslaunch 命令结束 (例如你手动 Ctrl+C)
    print(f"正在启动 launch 文件: {launch_file_path}")
    try:
        # Popen allows for more flexibility, e.g., non-blocking execution
        # but for simplicity, `call` is often sufficient.
        process = subprocess.Popen(['roslaunch', launch_file_path])
        process.wait()  # 等待 roslaunch 进程结束
    except KeyboardInterrupt:
        print("\n检测到 Ctrl+C，正在关闭 roslaunch...")
        process.terminate()
    except Exception as e:
        print(f"执行 roslaunch 时发生错误: {e}")