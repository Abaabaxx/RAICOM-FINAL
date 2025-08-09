#!/usr/bin/env python
# coding=utf-8

"""
雷达寻隙算法(Follow The Gap)实现

本代码实现了Follow The Gap (FTG)算法。其核心执行流程如下：

1.  **数据预处理**: 对原始雷达数据进行切片、平滑和限距，使其更易于分析。
2.  **创建安全气泡**: 找到最近的障碍物点，并将其周围一定范围的区域标记为“禁区”（距离设为0），以保证安全距离。
3.  **寻找最大间隙**: 在处理后的数据中，搜索所有可通行的路径（非0区域），并找出其中最宽的一条。
4.  **确定最佳目标点**: 在最宽的间隙内，找到距离最远（最开阔）的点作为前进的目标方向。
5.  **生成控制指令**: 根据目标方向计算出阿克曼转向角，并根据转向角的缓急动态调整车速，最后发布控制指令。

算法核心逻辑分析:
这个算法可以分为两个独立但又相互影响的部分:
1.主要的反应式避障(FTG):通过xwcgap_callback函数,实时处理雷达数据,实现基本的沿着开阔空间行驶的逻辑.
2.辅助的定位纠正:通过xwcpos_callback函数,使用里程计(Odometry)信息,在赛道的特定位置执行预设的,写死的转向动作,用于纠正纯FTG算法可能出错的特定弯道.

本算法基本稳定在41.5s/3r左右.
"""

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry

# 初始化ROS节点
rospy.init_node("xwcgap")

# 算法参数配置
BUBBLE_RADIUS = 200        # 零值气泡半径,以单个雷达点作为气泡圆心,气泡内设置为0拒绝小车车身进入
PREPROCESS_CONV_SIZE = 80  # 预处理卷积窗口大小
BEST_POINT_CONV_SIZE = 100 # 最佳点卷积窗口大小
MAX_LIDAR_DIST = 7.0       # 雷达回传数据中可以作为有价值参考的最远阈值
STRAIGHTS_SPEED = 2.2      # 直行速度
CORNERS_SPEED = 1.0        # 拐角速度
STRAIGHTS_STEERING_ANGLE = 0.25  # 判断拐角状态与否的角度阈值,如果阿克曼方向角大于阈值则判断进入拐角状态

# 每个雷达点的弧度
radians_per_elem = 0.00436  # 初始值,将在预处理函数中根据实际数据重新计算


def preprocess_lidar(ranges):
    """
    雷达数据预处理:
    1. 计算每个雷达点对应的弧度值（radians_per_elem）
    2. 截取需要处理的前180度数据（限制视野为前180度）
    3. 对数据进行卷积平滑处理（雷达数据平滑，动态窗口卷积法）
    4. 限制数据在有效范围内（强制近视，大于MAX_LIDAR_DIST的数据按照MAX_LIDAR_DIST算）
    
    参数:
        ranges: 雷达原始数据
        
    返回:
        proc_ranges: 处理后的雷达数据
    """
    global radians_per_elem
    radians_per_elem = (2 * np.pi) / len(ranges)  # 计算每个雷达点对应的弧度值
    print(ranges[360], ranges[-360], ranges[720])
    
    # 截取前180度的数据
    proc_ranges = np.array(ranges[360:-360])
    
    # 使用卷积平滑过滤,窗口大小为PREPROCESS_CONV_SIZE,计算每个元素在窗口内的平均值
    proc_ranges = np.convolve(proc_ranges, np.ones(PREPROCESS_CONV_SIZE), 'same') / PREPROCESS_CONV_SIZE
    
    # 限制数据在0到MAX_LIDAR_DIST之间,确保没有值超过MAX_LIDAR_DIST
    proc_ranges = np.clip(proc_ranges, 0, MAX_LIDAR_DIST)
    
    return proc_ranges


def find_max_gap(free_space_ranges):
    """
    在非0区域内寻找最大间隙
    
    参数:
        free_space_ranges: 已处理的雷达数据,其中0表示障碍物或无效区域
        
    返回:
        start, stop: 最大间隙的起始和结束索引
    """
    # 创建一个掩码数组,将所有等于0的值掩盖起来
    masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
    
    # 找到所有未被掩盖的连续元素序列(非零值序列)
    slicearr = np.ma.notmasked_contiguous(masked)
    
    if isinstance(slicearr, list):
        # 初始化最大长度为第一个非零值序列的长度
        max_len = slicearr[0].stop - slicearr[0].start
        chosen_slice = slicearr[0]
        
        # 遍历所有其他非零值序列,找出最长的一个
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
    在最大间隙范围内选择一个平均值最大的点
    这有助于避免直接撞到角落,因为距离平均值高的地方代表更加平坦开阔的区域
    
    参数:
        start_i: 最大间隙的起始索引
        end_i: 最大间隙的结束索引
        ranges: 处理后的雷达数据
        
    返回:
        最佳点在原始数组中的索引
    """
    # 对间隙范围内的数据进行平滑处理
    averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(BEST_POINT_CONV_SIZE), 'same') / BEST_POINT_CONV_SIZE
    
    # 找到平均值最大的位置,并加上起始索引得到原始数组中的索引
    return averaged_max_gap.argmax() + start_i


def get_angle(range_index, range_len):
    """
    从激光雷达数据索引中计算角度,并将其转换为阿克曼转向角
    
    参数:
        range_index: 雷达数据中的索引位置
        range_len: 雷达数据的总长度
        
    返回:
        steering_angle: 计算出的阿克曼转向角
    """
    # 计算相对于雷达数据中心的偏移量,转换为弧度
    lidar_angle = (range_index - (range_len / 2)) * radians_per_elem
    
    # 对目标角度进行缓冲,防止过度转向
    steering_angle = lidar_angle / 2
    
    return steering_angle


def process_lidar(ranges):
    """
    雷达寻隙算法(Follow The Gap)的主体实现
    
    参数:
        ranges: 雷达原始数据
    """
    # 预处理雷达数据
    proc_ranges = preprocess_lidar(ranges)
    
    # 找到最近的障碍物的索引
    closest = proc_ranges.argmin()
    
    # 定义气泡区域,将最近障碍物周围的点设置为0
    min_index = closest - BUBBLE_RADIUS
    max_index = closest + BUBBLE_RADIUS
    if min_index < 0:
        min_index = 0
    if max_index >= len(proc_ranges):
        max_index = len(proc_ranges) - 1
    proc_ranges[int(min_index):int(max_index)] = 0
    
    # 找到最大间隙
    gap_start, gap_end = find_max_gap(proc_ranges)
    
    # 在最大间隙中找到最佳目标点
    best = find_best_point(gap_start, gap_end, proc_ranges)
    
    # 计算转向角
    steering_angle = get_angle(best, len(proc_ranges))
    
    # 根据转向角判断当前是直行还是拐角状态,调整速度
    if abs(steering_angle) > STRAIGHTS_STEERING_ANGLE:
        speed = CORNERS_SPEED
    else:
        speed = STRAIGHTS_SPEED
    
    # 创建并发布阿克曼驱动消息
    drive_stamped_msg = AckermannDriveStamped()
    drive_stamped_msg.header.stamp = rospy.Time.now()
    drive_stamped_msg.header.frame_id = "base_link"  # 或者 "tianracer/base_link"
    drive_stamped_msg.drive.steering_angle = steering_angle
    drive_stamped_msg.drive.speed = speed
    
    print(steering_angle)
    drive_pub.publish(drive_stamped_msg)


def get_range(data, angle):
    """
    获取指定角度的雷达距离值
    
    参数:
        data: 雷达数据消息
        angle: 需要获取距离的角度(度)
        
    返回:
        distance: 指定角度的距离值
    """
    # 将角度从度转换为弧度
    angle = np.deg2rad(angle)
    
    # 计算该角度在雷达数据数组中的索引位置
    # ranges[]数组是雷达回传数据,表示从angle_min到angle_max角度范围内的距离数据
    # 数组大小与激光雷达转速,方向角分辨率有关,由angle_increment决定
    index = int((angle - data.angle_min) / data.angle_increment)
    dis = data.ranges[index]
    
    # 过滤不可信的数据
    if dis < data.range_min or dis > data.range_max:
        dis = MAX_LIDAR_DIST  # 不可信数据默认为最长距离
        
    return dis


def xwcpos_callback(odommsg):
    """
    位置回调函数,用于处理里程计消息
    在特定位置触发转向动作
    
    参数:
        odommsg: 里程计消息
    """
    # 检测特定位置点并执行相应的转向动作
    if abs(odommsg.pose.pose.position.x - 5.0) <= 0.2 and abs(odommsg.pose.pose.position.y + 0.5) <= 0.2:
        turn_right()
    elif abs(odommsg.pose.pose.position.x - 4.0) <= 0.2 and abs(odommsg.pose.pose.position.y + 4.0) <= 0.5:
        turn_right2()


def turn_right():
    """
    执行向右转向的固定动作序列
    """
    rospy.logwarn(11111111111111111111111111111111111111111111111111111111111)
    drive_msg2 = AckermannDriveStamped()
    drive_msg2.drive.steering_angle = -100
    drive_msg2.drive.speed = CORNERS_SPEED
    rate = rospy.Rate(100)
    
    # 发送右转命令20次
    for m in range(20):
        drive_pub.publish(drive_msg2)
        rate.sleep()
    
    # 恢复直行10次
    drive_msg2.drive.steering_angle = 0
    drive_msg2.drive.speed = CORNERS_SPEED
    for n in range(10):
        drive_pub.publish(drive_msg2)
    
    return


def turn_right2():
    """
    执行另一种向右转向的固定动作序列
    """
    rospy.logwarn(11111111111111111111111111111111111111111111111111111111111)
    drive_msg2 = AckermannDriveStamped()
    drive_msg2.drive.steering_angle = 100
    drive_msg2.drive.speed = CORNERS_SPEED
    rate = rospy.Rate(100)
    
    # 发送转向命令15次
    for m in range(15):
        drive_pub.publish(drive_msg2)
        rate.sleep()
    
    # 恢复直行10次
    drive_msg2.drive.steering_angle = 0
    drive_msg2.drive.speed = CORNERS_SPEED
    for n in range(10):
        drive_pub.publish(drive_msg2)
    
    return


def xwcgap_callback(data):
    """
    雷达回调函数,处理激光雷达数据
    
    参数:
        data: 雷达数据消息
    """
    # 直接调用雷达处理函数,进行避障控制
    process_lidar(data.ranges)


if __name__ == '__main__':
    try:
        # 订阅话题,带有tianracer命名空间前缀
        odom_sub = rospy.Subscriber('/tianracer/odom', Odometry, xwcpos_callback)
        scan_sub = rospy.Subscriber('/tianracer/scan', LaserScan, xwcgap_callback)
        
        # 发布话题
        drive_pub = rospy.Publisher('/tianracer/ackermann_cmd_stamped', AckermannDriveStamped, queue_size=1)
        
        # 保持节点运行
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass