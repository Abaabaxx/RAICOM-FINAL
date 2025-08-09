#!/usr/bin/env python
# coding=utf-8

# 完全利用.cfg文件中定义的所有参数，使用了雷达和转向硬编码数值
# 使用RQT动态调参的Follow The Gap算法实现

"""
雷达寻隙算法(Follow The Gap)实现 - 动态调参版本

本代码实现了Follow The Gap (FTG)算法，并支持动态参数调整。其核心执行流程如下：

1.  **数据预处理**: 对原始雷达数据进行切片、平滑和限距，使其更易于分析。
2.  **创建安全气泡**: 找到最近的障碍物点，并将其周围一定范围的区域标记为"禁区"（距离设为0），以保证安全距离。
3.  **寻找最大间隙**: 在处理后的数据中，搜索所有可通行的路径（非0区域），并找出其中最宽的一条。
4.  **确定最佳目标点**: 在最宽的间隙内，找到距离最远（最开阔）的点作为前进的目标方向。
5.  **生成控制指令**: 根据目标方向计算出阿克曼转向角，并根据转向角的缓急动态调整车速，最后发布控制指令。

算法核心逻辑分析:
这个算法可以分为两个独立但又相互影响的部分:
1.主要的反应式避障(FTG):通过radar_callback函数,实时处理雷达数据,实现基本的沿着开阔空间行驶的逻辑.
2.辅助的定位纠正:通过pos_callback函数,使用里程计(Odometry)信息,在赛道的特定位置执行预设的,写死的转向动作,用于纠正纯FTG算法可能出错的特定弯道.

使用dynamic_reconfigure支持实时参数调整。
"""

"""
roslaunch /home/lby/tianbot_ws/src/f1/launch/7-1-test0.launch
python /home/lby/tianbot_ws/src/f1/scripts/7-1-test1.py
rosrun rqt_reconfigure rqt_reconfigure
"""

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry

# 导入 dynamic_reconfigure 相关的模块
from dynamic_reconfigure.server import Server
from f1.cfg import F1TuningConfig  # 'f1' 是你的包名，'F1TuningConfig' 是根据你的.cfg文件生成的


class FollowTheGapNode:
    def __init__(self):
        """
        构造函数，用于初始化节点、参数、发布者和订阅者
        """
        # 初始化 ROS 节点
        rospy.init_node("ftg_node")
        
        # --- 将所有.cfg中的参数定义为类的成员变量 ---
        
        # A_FTG_Core_Algorithm 组
        self.BUBBLE_RADIUS = 100
        self.PREPROCESS_CONV_SIZE = 50
        self.BEST_POINT_CONV_SIZE = 100
        
        # B_Vehicle_Control 组
        self.MAX_LIDAR_DIST = 7.0
        self.STEERING_GAIN = 0.5
        self.STRAIGHTS_SPEED = 3.0
        self.CORNERS_SPEED = 1.5
        self.STRAIGHTS_STEERING_ANGLE = 0.2
        self.DESIRED_FOV = 180.0
        
        # C_Maneuver_1 组
        self.M1_TRIGGER_X = 5.0
        self.M1_TRIGGER_Y = -0.5
        self.M1_TRIGGER_TOLERANCE = 0.3
        self.M1_ANGLE = -1.0
        self.M1_SPEED = 1.2
        self.M1_DURATION = 0.3
        
        # D_Maneuver_2 组
        self.M2_TRIGGER_X = 4.0
        self.M2_TRIGGER_Y = 4.0
        self.M2_TRIGGER_TOLERANCE = 0.5
        self.M2_ANGLE = 1.0
        self.M2_SPEED = 1.2
        self.M2_DURATION = 0.2

        self.radians_per_elem = 0.00436

        # 创建 dynamic_reconfigure 服务器实例
        # 当参数改变时，它会自动调用 self.reconfigure_callback
        self.dyn_reconfig_server = Server(F1TuningConfig, self.reconfigure_callback)

        # 创建发布者和订阅者
        self.drive_pub = rospy.Publisher('/tianracer/ackermann_cmd_stamped', AckermannDriveStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber('/tianracer/odom', Odometry, self.pos_callback)
        self.scan_sub = rospy.Subscriber('/tianracer/scan', LaserScan, self.radar_callback)

        rospy.loginfo("Follow The Gap Node with dynamic reconfigure has been initialized.")

    def reconfigure_callback(self, config, level):
        """
        dynamic_reconfigure 的回调函数。每当参数在 rqt_reconfigure 中被修改时，此函数被调用。
        
        参数:
            config: 包含所有参数值的配置对象
            level: 参数级别（在cfg文件中定义）
            
        返回:
            config: 返回配置对象
        """
        rospy.loginfo("Received dynamic reconfigure request. Updating parameters.")
        
        # --- 从 config 对象更新类的成员变量 ---
        
        # A_FTG_Core_Algorithm 组
        self.BUBBLE_RADIUS = config.bubble_radius
        self.PREPROCESS_CONV_SIZE = config.preprocess_conv_size
        self.BEST_POINT_CONV_SIZE = config.best_point_conv_size
        
        # B_Vehicle_Control 组
        self.MAX_LIDAR_DIST = config.max_lidar_dist
        self.STEERING_GAIN = config.steering_gain
        self.STRAIGHTS_SPEED = config.straights_speed
        self.CORNERS_SPEED = config.corners_speed
        self.STRAIGHTS_STEERING_ANGLE = config.straights_steering_angle
        self.DESIRED_FOV = config.desired_fov

        # C_Maneuver_1 组
        self.M1_TRIGGER_X = config.m1_trigger_x
        self.M1_TRIGGER_Y = config.m1_trigger_y
        self.M1_TRIGGER_TOLERANCE = config.m1_trigger_tolerance
        self.M1_ANGLE = config.m1_angle
        self.M1_SPEED = config.m1_speed
        self.M1_DURATION = config.m1_duration
        
        # D_Maneuver_2 组
        self.M2_TRIGGER_X = config.m2_trigger_x
        self.M2_TRIGGER_Y = config.m2_trigger_y
        self.M2_TRIGGER_TOLERANCE = config.m2_trigger_tolerance
        self.M2_ANGLE = config.m2_angle
        self.M2_SPEED = config.m2_speed
        self.M2_DURATION = config.m2_duration
        
        rospy.loginfo("Parameters updated: bubble_radius={}, max_lidar_dist={:.2f}, straights_speed={:.2f}, corners_speed={:.2f}".format(
            self.BUBBLE_RADIUS, self.MAX_LIDAR_DIST, self.STRAIGHTS_SPEED, self.CORNERS_SPEED))
        
        return config

    def preprocess_lidar(self, ranges):
        """
        一、雷达数据预处理:
        1. 计算每个雷达点对应的弧度值（radians_per_elem）
        2. 截取需要处理的前方数据（使用DESIRED_FOV参数）
        3. 对数据进行卷积平滑处理（雷达数据平滑，动态窗口卷积法）
        4. 限制数据在有效范围内（强制近视，大于MAX_LIDAR_DIST的数据按照MAX_LIDAR_DIST算）
        
        参数:
            ranges: 雷达原始数据
            
        返回:
            proc_ranges: 处理后的雷达数据
        """
        self.radians_per_elem = (2 * np.pi) / len(ranges)
        
        # 使用DESIRED_FOV参数进行动态截取
        total_points = len(ranges)
        center_index = total_points // 2
        
        # 计算期望视野角度对应需要截取的点数
        fov_rad = np.deg2rad(self.DESIRED_FOV)
        points_per_radian = total_points / (2 * np.pi)
        half_fov_points = int((fov_rad / 2) * points_per_radian)

        # 计算截取的起始和结束索引
        start_index = max(0, center_index - half_fov_points)
        end_index = min(total_points, center_index + half_fov_points)
        
        proc_ranges = np.array(ranges[start_index:end_index])
        
        # 使用卷积平滑过滤,窗口大小为PREPROCESS_CONV_SIZE,计算每个元素在窗口内的平均值
        if self.PREPROCESS_CONV_SIZE > 0:
            proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        
        # 限制数据在0到MAX_LIDAR_DIST之间,确保没有值超过MAX_LIDAR_DIST
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        
        return proc_ranges

    def create_safety_bubble(self, proc_ranges, bubble_radius):
        """
        二、制造"安全气泡"（画出"绝对禁区"）
        1.找到最大威胁：在经过数据预处理后的的数据中，算法会找到距离最近的那个点
        2. "膨胀"威胁:在最近的障碍物周围创建一个"安全气泡",将最近点及其周围半径内的雷达距离值设为0，表示禁区
        
        参数:
            proc_ranges: 经过预处理的雷达数据
            bubble_radius: 安全气泡的半径（向左/向右扩展的点数）
            
        返回:
            带有安全气泡的雷达数据
        """
        # 找到最近的障碍物的索引
        closest_index = proc_ranges.argmin()
        
        # 计算气泡的左右边界
        min_index = max(0, closest_index - bubble_radius)
        max_index = min(len(proc_ranges) - 1, closest_index + bubble_radius)
        
        # 创建气泡（将范围内的值设为0）
        proc_ranges[int(min_index):int(max_index)] = 0
        
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """
        三、寻找最大间隙（"找条最宽的路"）(在非0区域内寻找最大间隙)
        1.识别所有通道
        2.选择最宽的通道作为前进方向
        
        参数:
            free_space_ranges: 已处理的雷达数据,其中0表示障碍物或无效区域
            
        返回:
            start, stop: 最大间隙的起始和结束索引
        """
        # 创建一个掩码数组,将所有等于0的值掩盖起来
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        
        # 找到所有未被掩盖的连续元素序列(非零值序列)
        slice_arr = np.ma.notmasked_contiguous(masked)
        
        # 如果没有找到任何gap，返回中间位置的小gap
        if not slice_arr:
            mid_point = len(free_space_ranges) // 2
            return mid_point, mid_point + 1
        
        if isinstance(slice_arr, list):
            # 初始化最大长度为第一个非零值序列的长度
            max_len = slice_arr[0].stop - slice_arr[0].start
            chosen_slice = slice_arr[0]
            
            # 遍历所有其他非零值序列,找出最长的一个
            for sl in slice_arr[1:]:
                sl_len = sl.stop - sl.start
                if sl_len > max_len:
                    max_len = sl_len
                    chosen_slice = sl
        else:
            chosen_slice = slice_arr
            
        return chosen_slice.start, chosen_slice.stop

    def find_best_point(self, start_i, end_i, ranges):
        """
        四、在间隙中寻找最佳目标点（"瞄准最开阔的远方"）
        该最佳目标点是经过BEST_POINT_CONV_SIZE最佳点卷积窗口平滑处理后,在最大间隙范围内的平均值最大的点
        
        参数:
            start_i: 最大间隙的起始索引
            end_i: 最大间隙的结束索引
            ranges: 处理后的雷达数据
            
        返回:
            最佳点在原始数组中的索引
        """
        # Gap长度为0或1的情况
        if start_i >= end_i:
            return start_i
        
        gap_slice = ranges[start_i:end_i]
        
        # 对间隙范围内的数据进行平滑处理
        if self.BEST_POINT_CONV_SIZE > 0 and len(gap_slice) >= self.BEST_POINT_CONV_SIZE:
            averaged_max_gap = np.convolve(gap_slice, np.ones(self.BEST_POINT_CONV_SIZE), 'same') / self.BEST_POINT_CONV_SIZE
        else:
            averaged_max_gap = gap_slice
        
        # 找到平均值最大的位置,并加上起始索引得到原始数组中的索引
        return averaged_max_gap.argmax() + start_i

    def get_angle(self, range_index, range_len):
        """
        五、计算最大间隙中的最佳目标点对应的阿克曼转向角（通过激光雷达数据索引位置计算角度,并将其转换）
        
        参数:
            range_index: 雷达数据中的索引位置
            range_len: 雷达数据的总长度
            
        返回:
            steering_angle: 计算出的阿克曼转向角
        """
        # 计算相对于雷达数据中心的偏移量,转换为弧度
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        
        # 使用动态转向增益
        steering_angle = lidar_angle * self.STEERING_GAIN
        
        return steering_angle

    def determine_speed(self, steering_angle):
        """
        六、根据转向角与直行和拐角分界的角度阈值判断当前是直行还是拐角状态

        参数:
            steering_angle (float): 计算出的目标转向角。

        返回:
            float: 根据当前路况（直行或拐角）决定的目标速度。
        """
        # 如果转向角的绝对值大于设定的阈值，则判断为拐角，使用低速
        if abs(steering_angle) > self.STRAIGHTS_STEERING_ANGLE:
            return self.CORNERS_SPEED
        # 否则，判断为直行，使用高速
        else:
            return self.STRAIGHTS_SPEED
        
    def publish_control_command(self, speed, steering_angle):
        """
        七、创建并发布阿克曼驱动消息。

        参数:
            speed (float): 车辆的目标速度。
            steering_angle (float): 车辆的目标转向角。
        """
        # 创建一个阿克曼驱动消息对象
        drive_stamped_msg = AckermannDriveStamped()
        
        # 填充消息的header部分
        drive_stamped_msg.header.stamp = rospy.Time.now()
        drive_stamped_msg.header.frame_id = "base_link"  # 或者 "tianracer/base_link"
        
        # 填充消息的drive部分
        drive_stamped_msg.drive.steering_angle = steering_angle
        drive_stamped_msg.drive.speed = speed
        
        # 通过发布者发布消息
        self.drive_pub.publish(drive_stamped_msg)

    def process_lidar(self, ranges):
        """
        雷达寻隙算法(Follow The Gap)的主体实现
        
        参数:
            ranges: 雷达原始数据
        """
        # 1. 预处理雷达数据
        proc_ranges = self.preprocess_lidar(ranges)
        
        # 2. 创建安全气泡
        proc_ranges = self.create_safety_bubble(proc_ranges, self.BUBBLE_RADIUS)
        
        # 3. 找到最大间隙
        gap_start, gap_end = self.find_max_gap(proc_ranges)
        
        # 4. 在最大间隙中找到最佳目标点
        best = self.find_best_point(gap_start, gap_end, proc_ranges)
        
        # 5. 计算转向角
        steering_angle = self.get_angle(best, len(proc_ranges))
        
        # 6. 根据转向角与直行和拐角分界的角度阈值判断当前是直行还是拐角状态,调整速度
        speed = self.determine_speed(steering_angle)
        
        # 7. 发布控制指令
        self.publish_control_command(speed, steering_angle)

    def execute_maneuver(self, angle, speed, duration, maneuver_name):
        """
        执行一个预设的、固定时长的转向动作。
        
        参数:
            angle: 转向角度（弧度）
            speed: 执行速度（米/秒）
            duration: 持续时间（秒）
            maneuver_name: 机动动作名称
        """
        rospy.logwarn("Executing hardcoded {}!".format(maneuver_name))
        drive_msg = AckermannDriveStamped()
        drive_msg.header.frame_id = "base_link"
        
        # 设置转向动作
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed
        
        # 发送固定时长的转向命令
        rate = rospy.Rate(100)  # 100Hz
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration:
            drive_msg.header.stamp = rospy.Time.now()
            self.drive_pub.publish(drive_msg)
            rate.sleep()
        
        rospy.loginfo("{} finished. Resuming FTG control.".format(maneuver_name))

    def pos_callback(self, odommsg):
        """
        位置回调函数,用于处理里程计消息
        在特定位置触发转向动作
        
        参数:
            odommsg: 里程计消息
        """
        pos_x = odommsg.pose.pose.position.x
        pos_y = odommsg.pose.pose.position.y
        
        # 检测 Maneuver 1
        if abs(pos_x - self.M1_TRIGGER_X) <= self.M1_TRIGGER_TOLERANCE and \
           abs(pos_y - self.M1_TRIGGER_Y) <= self.M1_TRIGGER_TOLERANCE:
            self.execute_maneuver(
                angle=self.M1_ANGLE,
                speed=self.M1_SPEED,
                duration=self.M1_DURATION,
                maneuver_name="Maneuver 1"
            )
        # 检测 Maneuver 2
        elif abs(pos_x - self.M2_TRIGGER_X) <= self.M2_TRIGGER_TOLERANCE and \
             abs(pos_y - self.M2_TRIGGER_Y) <= self.M2_TRIGGER_TOLERANCE:
            self.execute_maneuver(
                angle=self.M2_ANGLE,
                speed=self.M2_SPEED,
                duration=self.M2_DURATION,
                maneuver_name="Maneuver 2"
            )

    def radar_callback(self, data):
        """
        雷达回调函数,处理激光雷达数据
        
        参数:
            data: 雷达数据消息
        """
        # 直接调用雷达处理函数,进行避障控制
        self.process_lidar(data.ranges)

    def run(self):
        """
        保持节点运行
        """
        rospy.loginfo("Follow The Gap Node is running. Use rqt_reconfigure to adjust parameters.")
        rospy.spin()


if __name__ == '__main__':
    try:
        # 创建类的实例并运行
        node = FollowTheGapNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Follow The Gap Node has been interrupted.")
        pass