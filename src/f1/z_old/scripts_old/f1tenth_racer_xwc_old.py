#!/usr/bin/env python
#coding=utf-8

#code & annotation @xwc3171367903
# 本代码主要算法是follow_the_gap（fg），即雷达寻隙法。这个反应式算法的基本原理是搜索雷达回传数据，更新无障碍的最远方向设置为运动目的地
# 并不断进行阿克曼切角与速度的调整以适应直奔目的地途中的环境变化。本版算法基本稳定41.5s/3r左右。

import rospy#rospy必需的包
import numpy as np#numpy重命名np。numpy用于处理多维数组和矩阵，并提供大量的数学函数来操作数组
from sensor_msgs.msg import LaserScan#雷达传感器相关
from ackermann_msgs.msg import AckermannDriveStamped,AckermannDrive#阿克曼消息包，用于处理阿克曼运动学相关参数
from nav_msgs.msg import Odometry 

rospy.init_node("xwcgap")#rospy必需步骤，初始化节点

BUBBLE_RADIUS = 200#零值气泡半径。以单个雷达点作为气泡圆心，气泡内设置为0拒绝小车车身进入
PREPROCESS_CONV_SIZE = 80
BEST_POINT_CONV_SIZE = 100
MAX_LIDAR_DIST = 7.0#雷达回传数据中可以作为有价值参考的最远阈值
STRAIGHTS_SPEED = 2.2#直行速度
CORNERS_SPEED =1.0#拐角速度
STRAIGHTS_STEERING_ANGLE = 0.25#判断拐角状态与否的角度阈值。如果阿克曼方向角大于阈值则判断进入拐角状态
# AHEAD_THREHOLD = 3.0#判断无障碍高速直行的前向阈值距离
# TRANSVERSE_THREHOLD = 0.4#判断无障碍高速直行的横向阈值距离

radians_per_elem = 0.00436#一雷达点的弧度。这里0.1是python浮点数定义赋值，无实义

#预处理雷达降噪函数
def preprocess_lidar(ranges):
    global radians_per_elem
    radians_per_elem = (2 * np.pi) / len(ranges)#计算每个雷达点对应的弧度值
    print(ranges[360],ranges[-360],ranges[720])
    proc_ranges = np.array(ranges[360:-360])#从ranges[]中截取前180度的数据
    proc_ranges = np.convolve(proc_ranges, np.ones(PREPROCESS_CONV_SIZE), 'same') / PREPROCESS_CONV_SIZE#使用np.convolve函数对proc_ranges进行卷积平滑过滤。
    #这里使用的是均匀权重的窗口（所有元素都是1），窗口的大小由PREPROCESS_CONV_SIZE定义。'same' 参数表示输出数组的长度与输入数组相同。卷积的结果除以窗口大小，得到每个元素在窗口内的平均值。
    proc_ranges = np.clip(proc_ranges, 0, MAX_LIDAR_DIST)#使用np.clip函数限制proc_ranges的值在0到MAX_LIDAR_DIST之间。
    #对高值进行拒绝，确保没有值超过MAX_LIDAR_DIST
    return proc_ranges

#在非0区内寻找最大间隙。
def find_max_gap(free_space_ranges):
    masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)#使用masked array函数创建一个新的masked[]。这个新的数组将free_space_ranges中所有等于0的值掩盖起来。
    slicearr = np.ma.notmasked_contiguous(masked)#使用notmasked_contiguous函数找到masked数组中所有未被掩盖的、连续的元素序列（即非零值序列），并返回这些序列的切片对象数组。
    if isinstance(slicearr, list):
        max_len = slicearr[0].stop - slicearr[0].start#初始化max_len为第一个切片对象的长度（即第一个非零值序列的长度）
        chosen_slice = slicearr[0]#将chosen_slice初始化为第一个切片对象
        for sl in slicearr[1:]:#遍历slicearr数组中除第一个切片对象之外的所有其他切片对象
            sl_len = sl.stop - sl.start#计算当前切片对象的长度
            if sl_len > max_len:#如果当前切片对象的长度大于max_len，则更新max_len为当前切片对象的长度
                max_len = sl_len
                chosen_slice = sl
    else:
        chosen_slice = slicearr
    return chosen_slice.start, chosen_slice.stop

#在最大间隙范围内选择一个平均值最大的点。帮助避免直接撞到角落，距离平均值高的地方代表更加平坦开阔
#start_i表示最大间隙范围的起始索引，end_i表示最大间隙范围的结束索引
def find_best_point( start_i, end_i, ranges):
    averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(BEST_POINT_CONV_SIZE),'same') / BEST_POINT_CONV_SIZE#使用convolve函数对ranges[]的start_i到end_i范围内的数据进行平滑过滤。
    #np.ones(BEST_POINT_CONV_SIZE)创建了一个长度为BEST_POINT_CONV_SIZE的全1数组，'same' 表示输出数组的长度与输入数组ranges[start_i:end_i]相同
    #最后，将卷积的结果除以BEST_POINT_CONV_SIZE得到平均值
    return averaged_max_gap.argmax() + start_i#使用argmax函数找到averaged_max_gap数组中的最大值的索引
    #这个索引对应的是滑动窗口平均值最大的位置。由于我们在ranges[]的一个子集上进行平滑过滤，所以需要将argmax的结果加上start_i来得到原始ranges数组中的索引

#从激光雷达数据中获取角度，并将其转换为阿克曼转向角
def get_angle(range_index, range_len):
    lidar_angle = (range_index - (range_len / 2)) * radians_per_elem#计算传入参数相对于激光雷达数据中心的偏移量，乘以radians_per_elem来得到该元素对应的弧度值
    steering_angle = lidar_angle/2 #对目标角度进行缓冲
    return steering_angle

#fg算法主体
def process_lidar(ranges):
    proc_ranges = preprocess_lidar(ranges)#预处理雷达，进行平滑降噪处理
    closest = proc_ranges.argmin()#使用argmin函数找到proc_ranges数组中的最小值索引，这个索引代表离激光雷达最近的物体

    min_index = closest - BUBBLE_RADIUS
    max_index = closest + BUBBLE_RADIUS
    if min_index < 0: min_index = 0#定义了气泡区域，即在离最近物体一定距离（由BUBBLE_RADIUS定义）内的所有点都被设置为0
    if max_index >= len(proc_ranges): max_index = len(proc_ranges) - 1
    proc_ranges[int(min_index):int(max_index)] = 0

    gap_start, gap_end = find_max_gap(proc_ranges)#找到proc_ranges中的最大间隙的起始和结束位置

    best = find_best_point(gap_start, gap_end, proc_ranges)#在找到的最大间隙中找到一个最佳目标点

    steering_angle = get_angle(best, len(proc_ranges))#调用get_angle函数计算基于最佳点的转向角

    if abs(steering_angle) > STRAIGHTS_STEERING_ANGLE:#判断拐角状态调整速度发布
        speed = CORNERS_SPEED
    else:
        speed = STRAIGHTS_SPEED
    # speed = STRAIGHTS_SPEED - abs(steering_angle)*2
    # print(len(ranges))
    #发布阿克曼运动速度与转向角指令（fg算法结算）
    drive_msg = AckermannDrive()
    drive_msg.steering_angle=steering_angle
    print(steering_angle)
    drive_msg.speed=speed
    drive_pub.publish(drive_msg)

#雷达数据处理，输出距离
def get_range(data, angle):
    angle = np.deg2rad(angle)#deg->rad
    dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]#ranges[]数组是雷达回传数据的一部分
    #ranges[]数组表示雷达旋转时，记录从angle_min到angle_max 角度范围内的距离数据，
    #数组的大小并不是固定的360个，与激光雷达转速、方向角分辨有关，即以多少角度为间隔采集数据，
    #也就是消息里面的angle_increment，(angle_max-angle_min) /angle_increment 即是数组的大小，序号从angle_min开始，
    #即angle_min对应的序号为0，intensities[]与ranges对应，记录每个数据点的强度或者是激光雷达的反射率
    if dis < data.range_min or dis > data.range_max:
        dis = MAX_LIDAR_DIST#不可信的数据默认为最长
    return dis

def xwcpos_callback(odommsg):
    # global odom_switch
    #print (odommsg.pose.pose.position.x,odommsg.pose.pose.position.y)
    if abs(odommsg.pose.pose.position.x - 5.0)<= 0.2 and abs(odommsg.pose.pose.position.y + 0.5)<= 0.2:
        turn_right()
    elif abs(odommsg.pose.pose.position.x - 4.0)<= 0.2 and abs(odommsg.pose.pose.position.y + 4.0)<= 0.5:
        turn_right2()

# def turn_right():
#     rospy.logwarn(11111111111111111111111111111111111111111111111111111111111)
#     drive_msg2 = AckermannDriveStamped()
#     drive_msg2.drive.steering_angle=100
#     drive_msg2.drive.speed=CORNERS_SPEED
#     rate = rospy.Rate(100)
#     for m in range(2000):
#         drive_pub.publish(drive_msg2)
#         rate.sleep()
#     drive_msg2.drive.steering_angle=0
#     drive_msg2.drive.speed=CORNERS_SPEED
#     for n in range(10):
#         drive_pub.publish(drive_msg2)
#     return

def turn_right():
    rospy.logwarn(11111111111111111111111111111111111111111111111111111111111)
    drive_msg2 = AckermannDriveStamped()
    drive_msg2.drive.steering_angle=-100
    drive_msg2.drive.speed=CORNERS_SPEED
    rate = rospy.Rate(100)
    for m in range(20):
        drive_pub.publish(drive_msg2)
        rate.sleep()
    drive_msg2.drive.steering_angle=0
    drive_msg2.drive.speed=CORNERS_SPEED
    for n in range(10):
        drive_pub.publish(drive_msg2)
    return

def turn_right2():
    rospy.logwarn(11111111111111111111111111111111111111111111111111111111111)
    drive_msg2 = AckermannDriveStamped()
    drive_msg2.drive.steering_angle=100
    drive_msg2.drive.speed=CORNERS_SPEED
    rate = rospy.Rate(100)
    for m in range(15):
        drive_pub.publish(drive_msg2)
        rate.sleep()
    drive_msg2.drive.steering_angle=0
    drive_msg2.drive.speed=CORNERS_SPEED
    for n in range(10):
        drive_pub.publish(drive_msg2)
    return

#雷达回调函数
def xwcgap_callback(data):
    # # closest = min(data.ranges[360:-360])
    # # # global odom_switch
    # # # print (get_range(data,90),get_range(data,-90))
    # # #如果正前方和横向阈值距离内无障碍则高速直行。这里是为了弥补fg算法在岔路处路线抉择困难的短板
    # if data.ranges[720]>=AHEAD_THREHOLD and data.ranges[360]>=TRANSVERSE_THREHOLD and data.ranges[-360]>=TRANSVERSE_THREHOLD:
    # # if min(data.ranges[700:740])>=AHEAD_THREHOLD and get_range(data,90)>=TRANSVERSE_THREHOLD and get_range(data,-90)>=TRANSVERSE_THREHOLD:
    # # print(min(data.ranges[710:730]))
    # # if min(filter(lambda x: x != 0.0, data.ranges))>=0.5:
    #     #发布阿克曼运动速度与转向角指令（无障碍直行）
    #     drive_msg1 = AckermannDrive()
    #     drive_msg1.steering_angle=0
    #     drive_msg1.speed=STRAIGHTS_SPEED
    #     drive_pub.publish(drive_msg1)
    # # elif get_range(data,-90)>=1.6 and get_range(data,90)>=0.7:
    # #     turn_right()
    # # elif odom_switch:
    # #     turn_right()
    # else :
        process_lidar(data.ranges)#如果正前方阈值距离内出现障碍，则进入fg算法控制小车避障
    
if __name__ == '__main__': 
    try:
        # odom_sub = rospy.Subscriber('/tianracer/odom', Odometry, xwcpos_callback)
        scan_sub = rospy.Subscriber('/scan', LaserScan, xwcgap_callback)#订阅雷达发布话题并回调
        drive_pub = rospy.Publisher('/tianracer/ackermann_cmd', AckermannDrive, queue_size=1)#发布阿克曼控制消息话题
        rospy.spin()

    except rospy.ROSInterruptException:
        pass