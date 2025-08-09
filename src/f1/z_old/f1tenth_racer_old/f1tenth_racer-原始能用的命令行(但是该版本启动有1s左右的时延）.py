#!/usr/bin/env python
# coding=utf-8

# 原始的脚本启动
"""
本脚本使用 os.system 直接在终端中发布话题，使得小车启动。
"""

import os

# 1. 定义您想在终端中执行的完整命令字符串
command = 'rostopic pub -1 /ftg/enable std_msgs/Bool "data: true"'

# 2. 打印提示信息，告知用户将要执行什么
# print("即将执行以下终端命令:")
# print(command)

# 3. 使用 os.system() 执行命令
os.system(command)

# 4. 打印完成信息
print("\n命令已执行完毕。")