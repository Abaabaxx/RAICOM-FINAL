#!/usr/bin/env python
# coding=utf-8

# 版本说明：将FTG消息类型封装，为解除FTG算法与可视化的耦合做准备
# 新新新新增：FTG消息类型进行封装
# 新新新增：添加导航决策器的逻辑，根据当前状态判断是否执行FTG或TEB导航。
# 新新新增：删除硬编码的转向逻辑
# 新新增：导航仲裁器 (Navigation Arbitrator) 和 指令多路复用器 (Command Multiplexer, Mux) 的方案。将 cmd_vel-->ftg_cml_vel，不直接控制小车
# 新增：为Follow The Gap算法添加RViz可视化功能，显示滤波和限距后的雷达点、安全气泡区域、识别出的最大间隙、最近的障碍物点（作为标记）、最佳目标点（作为标记）。
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
roslaunch /home/lby/tianbot_ws/src/f1/launch/RVIZ.launch
python /home/lby/tianbot_ws/src/f1/scripts/7-1-test3.py

rosrun rviz rviz -d $(rospack find f1)/rviz/all.rviz

rosrun rqt_reconfigure rqt_reconfigure

rosservice call /gazebo/reset_simulation "{}"

"""

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from std_msgs.msg import Header, String  # <-- 【新增】导入 String 消息类型
import sensor_msgs.point_cloud2 as pc2

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
        
        # --- 【新增】初始化导航模式 ---
        # 默认启动时，假设为FTG模式，直到导航管理器接管
        self.current_mode = "FTG_MODE"
        
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
        

        self.radians_per_elem = 0.00436

        # 创建 dynamic_reconfigure 服务器实例
        # 当参数改变时，它会自动调用 self.reconfigure_callback
        self.dyn_reconfig_server = Server(F1TuningConfig, self.reconfigure_callback)

        # 创建发布者和订阅者
        # 【注意】这里的发布话题名 /tianracer/ackermann_cmd_stamped 是正确的，
        # 因为我们最终方案决定让FTG直接发布到这个最终控制话题上。
        self.drive_pub = rospy.Publisher('/tianracer/ackermann_cmd_stamped', AckermannDriveStamped, queue_size=1)
        self.scan_sub = rospy.Subscriber('/tianracer/scan', LaserScan, self.radar_callback)

        # --- 【新增】订阅导航模式话题 ---
        self.mode_sub = rospy.Subscriber('/navigation/mode', String, self.mode_callback)

        # --- 新增：用于可视化的发布者 ---
        # 1. 发布处理后的点云 (滤波+限距)
        self.processed_points_pub = rospy.Publisher('/ftg/processed_points', PointCloud2, queue_size=1)
        # 2. 发布安全气泡的点云
        self.bubble_pub = rospy.Publisher('/ftg/safety_bubble', PointCloud2, queue_size=1)
        # 3. 发布最大间隙的点云
        self.gap_pub = rospy.Publisher('/ftg/max_gap', PointCloud2, queue_size=1)
        # 4. 发布标记 (最近点, 目标点)
        self.marker_pub = rospy.Publisher('/ftg/markers', MarkerArray, queue_size=1)

        rospy.loginfo("FTG Planner Node has been initialized and is waiting for mode commands.")


    # --- 【新增】导航模式的回调函数 ---
    def mode_callback(self, msg):
        """
        接收来自导航管理器的模式指令，并更新当前状态。
        """
        # 避免在模式未改变时重复打印日志
        if self.current_mode != msg.data:
            rospy.loginfo("FTG_Node: Received new mode command: {}. Updating status.".format(msg.data))
            self.current_mode = msg.data

    def convert_to_cartesian(self, ranges, start_index=0):
        """
        将雷达的 ranges 数据转换为笛卡尔坐标点列表。
        返回一个 [(x, y, z), ...] 格式的点的列表。
        
        参数:
            ranges: 雷达距离数据
            start_index: 数据的起始索引偏移（用于处理截取后的数据）
        """
        points = []
        # 假设 ranges 的中心是车辆的正前方 (角度为0)
        center_index = len(ranges) / 2.0
        
        for i, r in enumerate(ranges):
            # 忽略无效点 (例如被设为0的点)
            if r > 0.01: 
                angle = (i - center_index) * self.radians_per_elem
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append((x, y, 0.0))
                
        return points

    def convert_index_to_cartesian(self, ranges, index):
        """
        将指定索引的雷达点转换为笛卡尔坐标。
        
        参数:
            ranges: 雷达距离数据
            index: 要转换的索引
            
        返回:
            (x, y, z): 笛卡尔坐标
        """
        if index < 0 or index >= len(ranges) or ranges[index] <= 0.01:
            return (0.0, 0.0, 0.0)
            
        center_index = len(ranges) / 2.0
        angle = (index - center_index) * self.radians_per_elem
        r = ranges[index]
        x = r * np.cos(angle)
        y = r * np.sin(angle)
        return (x, y, 0.0)

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
     
        # 【【【添加这些新行】】】
        # --- 读取分层控制的角度阈值 ---
        self.ANGLE_THRESH_1 = config.angle_thresh_1
        self.ANGLE_THRESH_2 = config.angle_thresh_2
        self.ANGLE_THRESH_3 = config.angle_thresh_3

        # --- 读取层级1的参数 ---
        self.SPEED_TIER_1 = config.speed_tier_1
        self.GAIN_TIER_1 = config.gain_tier_1

        # --- 读取层级2的参数 ---
        self.SPEED_TIER_2 = config.speed_tier_2
        self.GAIN_TIER_2 = config.gain_tier_2

        # --- 读取层级3的参数 ---
        self.SPEED_TIER_3 = config.speed_tier_3
        self.GAIN_TIER_3 = config.gain_tier_3

        # --- 读取层级4的参数 ---
        self.SPEED_TIER_4 = config.speed_tier_4
        self.GAIN_TIER_4 = config.gain_tier_4
        self.DESIRED_FOV = config.desired_fov
        
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
            带有安全气泡的雷达数据, 最近障碍物索引
        """
        # 找到最近的障碍物的索引
        closest_index = proc_ranges.argmin()
        
        # 计算气泡的左右边界
        min_index = max(0, closest_index - bubble_radius)
        max_index = min(len(proc_ranges) - 1, closest_index + bubble_radius)
        
        # 创建气泡（将范围内的值设为0）
        proc_ranges[int(min_index):int(max_index)] = 0
        
        return proc_ranges, closest_index

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


    def get_tiered_control(self, raw_angle_rad):
        """
        新的分层控制函数：根据目标角度范围动态调整线速度和转向增益
        
        参数:
            raw_angle_rad: 未经增益处理的原始目标角度（弧度）
            
        返回:
            speed: 计算出的目标速度
            final_steering_angle: 最终的转向角
        """
        angle_deg = abs(np.degrees(raw_angle_rad))
        
        speed = 0.0
        steering_gain = 0.0
        
        # 分层控制逻辑
        if angle_deg < self.ANGLE_THRESH_1: # <-- 使用动态参数
            speed = self.SPEED_TIER_1       # <-- 使用动态参数
            steering_gain = self.GAIN_TIER_1 # <-- 使用动态参数
            rospy.logdebug("Tiered Control: High-speed straight ({}°)".format(angle_deg))
        elif angle_deg < self.ANGLE_THRESH_2: # <-- 使用动态参数
            speed = self.SPEED_TIER_2
            steering_gain = self.GAIN_TIER_2
            rospy.logdebug("Tiered Control: Medium-speed gentle turn ({}°)".format(angle_deg))
        elif angle_deg < self.ANGLE_THRESH_3: # <-- 使用动态参数
            speed = self.SPEED_TIER_3
            steering_gain = self.GAIN_TIER_3
            rospy.logdebug("Tiered Control: Low-speed sharp turn ({}°)".format(angle_deg))
        else:
            speed = self.SPEED_TIER_4
            steering_gain = self.GAIN_TIER_4
            rospy.logdebug("Tiered Control: Very low-speed hairpin/emergency ({}°)".format(angle_deg))
        
        final_steering_angle = raw_angle_rad * steering_gain
        
        return speed, final_steering_angle


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

    def publish_visualization(self, header, proc_ranges, proc_ranges_before_bubble, closest_index, gap_start, gap_end, best_point_index):
        """
        发布可视化数据到RViz
        
        参数:
            header: LaserScan消息的header
            proc_ranges: 处理后的雷达数据（包含安全气泡）
            proc_ranges_before_bubble: 创建安全气泡前的雷达数据
            closest_index: 最近障碍物的索引
            gap_start: 最大间隙起始索引
            gap_end: 最大间隙结束索引
            best_point_index: 最佳目标点索引
        """
        
        # 1. 发布处理后的点云 (滤波+限距，不包含气泡)
        points_processed = self.convert_to_cartesian(proc_ranges_before_bubble)
        if points_processed:
            cloud_processed = pc2.create_cloud_xyz32(header, points_processed)
            self.processed_points_pub.publish(cloud_processed)
        
        # 2. 发布安全气泡的点云
        # 创建气泡区域的点云（使用原始距离数据显示被气泡覆盖的区域）
        bubble_points = []
        min_index = max(0, closest_index - self.BUBBLE_RADIUS)
        max_index = min(len(proc_ranges_before_bubble) - 1, closest_index + self.BUBBLE_RADIUS)
        
        for i in range(int(min_index), int(max_index) + 1):
            if i < len(proc_ranges_before_bubble) and proc_ranges_before_bubble[i] > 0.01:
                point = self.convert_index_to_cartesian(proc_ranges_before_bubble, i)
                if point[0] != 0 or point[1] != 0:  # 确保不是无效点
                    bubble_points.append(point)
        
        if bubble_points:
            cloud_bubble = pc2.create_cloud_xyz32(header, bubble_points)
            self.bubble_pub.publish(cloud_bubble)
        
        # 3. 发布最大间隙的点云
        gap_points = []
        for i in range(gap_start, gap_end):
            if i < len(proc_ranges) and proc_ranges[i] > 0.01:
                point = self.convert_index_to_cartesian(proc_ranges, i)
                if point[0] != 0 or point[1] != 0:  # 确保不是无效点
                    gap_points.append(point)
        
        if gap_points:
            cloud_gap = pc2.create_cloud_xyz32(header, gap_points)
            self.gap_pub.publish(cloud_gap)
        
        # 4. 发布标记 (最近点和最佳点)
        marker_array = MarkerArray()
        
        # 创建红色球体标记用于最近障碍物点
        marker_closest = Marker()
        marker_closest.header = header
        marker_closest.ns = "ftg_markers"
        marker_closest.id = 0
        marker_closest.type = Marker.SPHERE
        marker_closest.action = Marker.ADD
        
        # 设置最近点的位置
        closest_point = self.convert_index_to_cartesian(proc_ranges_before_bubble, closest_index)
        marker_closest.pose.position.x = closest_point[0]
        marker_closest.pose.position.y = closest_point[1]
        marker_closest.pose.position.z = closest_point[2]
        marker_closest.pose.orientation.w = 1.0
        
        # 设置标记的大小和颜色（红色）
        marker_closest.scale.x = 0.3
        marker_closest.scale.y = 0.3
        marker_closest.scale.z = 0.3
        marker_closest.color.r = 1.0
        marker_closest.color.g = 0.0
        marker_closest.color.b = 0.0
        marker_closest.color.a = 1.0
        
        marker_array.markers.append(marker_closest)
        
        # 创建蓝色球体标记用于最佳目标点
        marker_best = Marker()
        marker_best.header = header
        marker_best.ns = "ftg_markers"
        marker_best.id = 1
        marker_best.type = Marker.SPHERE
        marker_best.action = Marker.ADD
        
        # 设置最佳点的位置
        best_point = self.convert_index_to_cartesian(proc_ranges, best_point_index)
        marker_best.pose.position.x = best_point[0]
        marker_best.pose.position.y = best_point[1]
        marker_best.pose.position.z = best_point[2]
        marker_best.pose.orientation.w = 1.0
        
        # 设置标记的大小和颜色（蓝色）
        marker_best.scale.x = 0.3
        marker_best.scale.y = 0.3
        marker_best.scale.z = 0.3
        marker_best.color.r = 0.0
        marker_best.color.g = 0.0
        marker_best.color.b = 1.0
        marker_best.color.a = 1.0
        
        marker_array.markers.append(marker_best)
        
        # 发布标记数组
        self.marker_pub.publish(marker_array)

    def process_lidar(self, data):
        """
        雷达寻隙算法(Follow The Gap)的主体实现，包含可视化功能和新的分层控制逻辑
        
        参数:
            data: LaserScan消息
        """
        # 获取 header 用于所有可视化消息
        header = data.header
        
        # 1. 预处理雷达数据
        proc_ranges = self.preprocess_lidar(data.ranges)
        
        # 保存创建安全气泡前的数据用于可视化
        proc_ranges_before_bubble = np.copy(proc_ranges)
        
        # 2. 创建安全气泡
        proc_ranges, closest_index = self.create_safety_bubble(proc_ranges, self.BUBBLE_RADIUS)
        
        # 3. 找到最大间隙
        gap_start, gap_end = self.find_max_gap(proc_ranges)
        
        # 4. 在最大间隙中找到最佳目标点
        best_point_index = self.find_best_point(gap_start, gap_end, proc_ranges)
        
        # 5. 发布可视化数据
        self.publish_visualization(header, proc_ranges, proc_ranges_before_bubble, 
                                   closest_index, gap_start, gap_end, best_point_index)
        
        # --- 新的分层控制逻辑 ---
        # 6. 计算原始目标角度（未经增益处理）
        center_index = len(proc_ranges) / 2.0
        raw_angle = (best_point_index - center_index) * self.radians_per_elem
        
        # 7. 使用新的分层控制函数一次性获取速度和最终转向角
        speed, final_steering_angle = self.get_tiered_control(raw_angle)
        
        # 8. 发布控制指令
        self.publish_control_command(speed, final_steering_angle)


    def radar_callback(self, data):
        """
        雷达回调函数,处理激光雷达数据
        
        参数:
            data: 雷达数据消息
        """
        # --- 【核心修改】在这里加入模式判断 ---
        # 如果当前的导航模式不是FTG模式，则立即返回，不执行任何操作。
        # 这会有效地"暂停"FTG算法。
        if self.current_mode != "FTG_MODE":
            # (可选) 可以在这里打印一个debug信息，但为了避免刷屏，通常省略
            # rospy.logdebug("FTG_Node: Not in FTG_MODE, skipping processing.")
            return

        # 如果模式正确，则执行原有的所有逻辑
        self.process_lidar(data)

    def run(self):
        """
        保持节点运行
        """
        rospy.loginfo("Follow The Gap Node with tiered control and visualization is running.")
        rospy.loginfo("New tiered control system:")
        rospy.loginfo("  - 0°-10°: High speed (3.5 m/s), Low gain (0.4)")
        rospy.loginfo("  - 10°-20°: Medium speed (2.5 m/s), Medium gain (0.6)")
        rospy.loginfo("  - 20°-35°: Low speed (1.8 m/s), High gain (0.8)")
        rospy.loginfo("  - >35°: Very low speed (1.2 m/s), Max gain (1.0)")
        rospy.loginfo("Use rqt_reconfigure to adjust other parameters.")
        rospy.loginfo("Visualization topics:")
        rospy.loginfo("  - /ftg/processed_points: 处理后的雷达点云 (白色)")
        rospy.loginfo("  - /ftg/safety_bubble: 安全气泡区域 (黄色)")
        rospy.loginfo("  - /ftg/max_gap: 最大间隙 (绿色)")
        rospy.loginfo("  - /ftg/markers: 最近障碍物(红球)和目标点(蓝球)")
        rospy.spin()


if __name__ == '__main__':
    try:
        # 创建类的实例并运行
        node = FollowTheGapNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Follow The Gap Node has been interrupted.")
        pass