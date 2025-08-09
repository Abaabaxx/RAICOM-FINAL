#!/usr/bin/env python
# coding=utf-8

"""
雷达寻隙法(Follow The Gap)算法实现
vscode
基本原理：搜索激光雷达回传数据，找到最佳无障碍方向作为运动目标
并根据环境变化动态调整阿克曼转向角与速度
"""

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry

# ---------------- 全局参数配置 ----------------
# 算法参数
BUBBLE_RADIUS = 200           # 零值气泡半径，将障碍物周围设为零以避免碰撞
PREPROCESS_CONV_SIZE = 80     # 预处理激光数据的卷积窗口大小
BEST_POINT_CONV_SIZE = 100    # 寻找最佳点的卷积窗口大小
MAX_LIDAR_DIST = 7.0          # 雷达有效距离阈值

# 运动控制参数
STRAIGHTS_SPEED = 2.2         # 直线行驶速度
CORNERS_SPEED = 1.0           # 转弯速度
STRAIGHTS_STEERING_ANGLE = 0.25  # 转弯阈值角度

# 全局变量
radians_per_elem = 0.00436    # 每个激光点的弧度，初始值将在处理时更新

# ---------------- 激光数据处理函数 ----------------
def preprocess_lidar(ranges):
    """
    对激光雷达数据进行预处理，包括降噪和范围限制
    
    参数:
        ranges: 激光雷达原始距离数据
        
    返回:
        处理后的距离数据数组
    """
    global radians_per_elem
    # 更新每个激光点对应的弧度值
    radians_per_elem = (2 * np.pi) / len(ranges)
    
    # 调试输出
    print(ranges[360], ranges[-360], ranges[720])
    
    # 提取有效数据范围(180度视野)
    proc_ranges = np.array(ranges[360:-360])
    
    # 使用卷积进行平滑过滤，减少噪声影响
    proc_ranges = np.convolve(proc_ranges, 
                             np.ones(PREPROCESS_CONV_SIZE), 
                             'same') / PREPROCESS_CONV_SIZE
    
    # 限制数据范围在有效区间内
    proc_ranges = np.clip(proc_ranges, 0, MAX_LIDAR_DIST)
    
    return proc_ranges

def find_max_gap(free_space_ranges):
    """
    在处理后的激光数据中寻找最大连续空白区域
    
    参数:
        free_space_ranges: 处理后的距离数据
        
    返回:
        最大空白区域的起始和结束索引
    """
    # 创建掩码数组，将零值区域掩盖
    masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
    
    # 找出所有连续的非零序列
    slicearr = np.ma.notmasked_contiguous(masked)
    
    if isinstance(slicearr, list):
        # 初始化为第一个连续区域
        max_len = slicearr[0].stop - slicearr[0].start
        chosen_slice = slicearr[0]
        
        # 遍历所有连续区域找出最大的
        for sl in slicearr[1:]:
            sl_len = sl.stop - sl.start
            if sl_len > max_len:
                max_len = sl_len
                chosen_slice = sl
    else:
        chosen_slice = slicearr
        
    return chosen_slice.start, chosen_slice.stop

def find_best_point(start_i, end_i, ranges):
    """
    在最大间隙中找出最佳行驶点(距离最远点)
    
    参数:
        start_i: 间隙起始索引
        end_i: 间隙结束索引
        ranges: 处理后的距离数据
        
    返回:
        最佳点的索引
    """
    # 对间隙内的点进行平均，找出平均距离最远的区域
    averaged_max_gap = np.convolve(
        ranges[start_i:end_i], 
        np.ones(BEST_POINT_CONV_SIZE),
        'same'
    ) / BEST_POINT_CONV_SIZE
    
    # 返回平均值最大位置的索引(相对于原始数组)
    return averaged_max_gap.argmax() + start_i

def get_angle(range_index, range_len):
    """
    将激光雷达索引转换为转向角
    
    参数:
        range_index: 激光数据中的索引
        range_len: 激光数据长度
        
    返回:
        对应的阿克曼转向角
    """
    # 计算相对于中心的偏移角度
    lidar_angle = (range_index - (range_len / 2)) * radians_per_elem
    
    # 对目标角度进行缓冲，减小转向幅度
    steering_angle = lidar_angle / 2
    
    return steering_angle

def get_range(data, angle):
    """
    获取指定角度的激光雷达距离数据
    
    参数:
        data: 激光雷达消息
        angle: 角度(度)
        
    返回:
        指定角度的距离值
    """
    # 角度转换为弧度
    angle = np.deg2rad(angle)
    
    # 计算对应的数组索引并获取距离值
    index = int((angle - data.angle_min) / data.angle_increment)
    dis = data.ranges[index]
    
    # 检查数据是否在有效范围内
    if dis < data.range_min or dis > data.range_max:
        dis = MAX_LIDAR_DIST
        
    return dis

# ---------------- 核心算法实现 ----------------
def process_lidar(ranges):
    """
    处理激光雷达数据并生成控制指令
    
    参数:
        ranges: 激光雷达原始距离数据
    """
    # 预处理激光数据
    proc_ranges = preprocess_lidar(ranges)
    
    # 找出最近障碍物位置
    closest = proc_ranges.argmin()
    
    # 创建安全气泡区域(将障碍物周围设为零)
    min_index = max(0, closest - BUBBLE_RADIUS)
    max_index = min(len(proc_ranges) - 1, closest + BUBBLE_RADIUS)
    proc_ranges[int(min_index):int(max_index)] = 0
    
    # 寻找最大间隙
    gap_start, gap_end = find_max_gap(proc_ranges)
    
    # 在间隙中找出最佳行驶点
    best = find_best_point(gap_start, gap_end, proc_ranges)
    
    # 计算转向角
    steering_angle = get_angle(best, len(proc_ranges))
    
    # 根据转向角大小调整速度
    if abs(steering_angle) > STRAIGHTS_STEERING_ANGLE:
        speed = CORNERS_SPEED  # 转弯速度
    else:
        speed = STRAIGHTS_SPEED  # 直线速度
    
    # 创建并发布控制消息
    drive_stamped_msg = AckermannDriveStamped()
    drive_stamped_msg.header.stamp = rospy.Time.now()
    drive_stamped_msg.header.frame_id = "base_link"
    drive_stamped_msg.drive.steering_angle = steering_angle
    drive_stamped_msg.drive.speed = speed
    
    # 输出转向角度并发布消息
    print(steering_angle)
    drive_pub.publish(drive_stamped_msg)

# ---------------- 特殊转弯操作 ----------------
def turn_right():
    """
    执行右转弯特殊操作序列
    """
    rospy.logwarn("执行特殊右转弯操作")
    
    drive_msg = AckermannDriveStamped()
    drive_msg.drive.steering_angle = -100
    drive_msg.drive.speed = CORNERS_SPEED
    
    # 以100Hz的频率发布20次转向命令
    rate = rospy.Rate(100)
    for m in range(20):
        drive_pub.publish(drive_msg)
        rate.sleep()
    
    # 恢复直行
    drive_msg.drive.steering_angle = 0
    drive_msg.drive.speed = CORNERS_SPEED
    for n in range(10):
        drive_pub.publish(drive_msg)
    
    return

def turn_right2():
    """
    执行第二种右转弯特殊操作序列
    """
    rospy.logwarn("执行特殊右转弯操作2")
    
    drive_msg = AckermannDriveStamped()
    drive_msg.drive.steering_angle = 100
    drive_msg.drive.speed = CORNERS_SPEED
    
    # 以100Hz的频率发布15次转向命令
    rate = rospy.Rate(100)
    for m in range(15):
        drive_pub.publish(drive_msg)
        rate.sleep()
    
    # 恢复直行
    drive_msg.drive.steering_angle = 0
    drive_msg.drive.speed = CORNERS_SPEED
    for n in range(10):
        drive_pub.publish(drive_msg)
    
    return

# ---------------- ROS回调函数 ----------------
def xwcpos_callback(odommsg):
    """
    里程计数据回调函数，用于特定位置触发特殊动作
    
    参数:
        odommsg: 里程计消息
    """
    # 根据位置执行特殊转弯
    if abs(odommsg.pose.pose.position.x - 5.0) <= 0.2 and abs(odommsg.pose.pose.position.y + 0.5) <= 0.2:
        turn_right()
    elif abs(odommsg.pose.pose.position.x - 4.0) <= 0.2 and abs(odommsg.pose.pose.position.y + 4.0) <= 0.5:
        turn_right2()

def xwcgap_callback(data):
    """
    激光雷达数据回调函数
    
    参数:
        data: 激光雷达消息
    """
    # 直接使用FG算法处理激光数据并控制车辆
    process_lidar(data.ranges)

# ---------------- 主函数 ----------------
if __name__ == '__main__': 
    try:
        # 订阅必要的ROS话题
        odom_sub = rospy.Subscriber('/tianracer/odom', Odometry, xwcpos_callback)
        scan_sub = rospy.Subscriber('/tianracer/scan', LaserScan, xwcgap_callback)
        
        # 发布控制命令的话题
        drive_pub = rospy.Publisher('/tianracer/ackermann_cmd_stamped', AckermannDriveStamped, queue_size=1)
        
        # 保持节点运行
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass