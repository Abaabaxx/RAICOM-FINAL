#!/usr/bin/env python
# coding=utf-8

# 该版本为最终比赛使用的FTG算法节点！！！
# 新新新新新增：：解耦合后的纯FTG算法节点
# 新新新新增：将FTG消息类型进行封装，一并打包发送给可视化节点
# 新新新增：添加导航决策器的逻辑，根据当前状态判断是否执行FTG或TEB导航。
# 新新新增：删除硬编码的转向逻辑（xwc逻辑残留）
# 新新增：导航仲裁器 (Navigation Arbitrator) 和 指令多路复用器 (Command Multiplexer, Mux) 的方案。将 cmd_vel-->ftg_cml_vel，不直接控制小车
# 否定了指令多路复用器MUX的方案：最终实测因为mux的方案不支持不相同的消息类型，阿克曼底盘可以由ackermann_msgs/AckermannDriveStamped或者geometry_msgs/Twist速度消息类型控制，但teb发送的和FTG节点发送的数据类型不同，不能多路复用）
# 新增：将可视化逻辑分离到独立节点，当前节点专注于FTG计算和控制
# 新增：添加外部控制的启停机制，默认关闭状态（裁判系统点击启动即可直接启动）

"""
雷达寻隙算法(Follow The Gap)实现 - 纯计算节点版本

本代码实现了Follow The Gap (FTG)算法的计算部分，专注于：

1.  **数据预处理**: 对原始雷达数据进行切片、平滑和限距，使其更易于分析。
2.  **创建安全气泡**: 找到最近的障碍物点，并将其周围一定范围的区域标记为"禁区"（距离设为0），以保证安全距离。
3.  **寻找最大间隙**: 在处理后的数据中，搜索所有可通行的路径（非0区域），并找出其中最宽的一条。
4.  **确定最佳目标点**: 在最宽的间隙内，找到距离最远（最开阔）的点作为前进的目标方向。
5.  **生成控制指令**: 根据目标方向计算出阿克曼转向角，并根据转向角的缓急动态调整车速，最后发布控制指令。
6.  **发布调试数据**: 将中间计算数据打包发布，供可视化节点使用。
7.  **外部启停控制**: 通过/ftg/enable话题接收外部控制指令，默认关闭状态确保安全。

使用dynamic_reconfigure支持实时参数调整。
"""
"""
发布话题启动：
rostopic pub /ftg/enable std_msgs/Bool "data: true"

发布话题停止：
rostopic pub /ftg/enable std_msgs/Bool "data: false"
"""

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, String, Bool

# 【新增】导入我们定义的消息类型
from f1.msg import FtgData

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
        
        # --- 【新增】初始化激活状态 ---
        # 默认为False，确保节点启动后处于安全的待机状态
        self.is_active = False
        rospy.loginfo("FTG Node initialized in INACTIVE state for safety.")
        
        # --- 【原有】初始化导航模式 ---
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
        # 【修改】话题名和消息类型都必须与实体小车匹配
        self.drive_pub = rospy.Publisher('/Tianracer/ackermann_cmd', AckermannDrive, queue_size=1)
        self.scan_sub = rospy.Subscriber('/Tianracer/scan', LaserScan, self.radar_callback)

        # --- 【原有】订阅导航模式话题 ---
        self.mode_sub = rospy.Subscriber('/navigation/mode', String, self.mode_callback)

        # --- 【新增】订阅外部启停控制话题 ---
        self.enable_sub = rospy.Subscriber('/ftg/enable', Bool, self.enable_callback)

        # --- 【原有】发布调试数据的Publisher ---
        self.ftg_data_pub = rospy.Publisher('/ftg/debug_data', FtgData, queue_size=1)

        rospy.loginfo("FTG Computation Node has been initialized.")
        rospy.loginfo("Waiting for activation signal on /ftg/enable topic...")
        rospy.loginfo("Also waiting for mode commands from navigation manager.")

    # --- 【新增】外部启停控制的回调函数 ---
    def enable_callback(self, msg):
        """
        接收来自外部的启停控制指令，更新节点的激活状态。
        
        参数:
            msg: Bool类型消息，True表示激活，False表示停止
        """
        # 避免在状态未改变时重复打印日志
        if self.is_active != msg.data:
            if msg.data:
                rospy.loginfo("FTG_Node: Received ACTIVATION signal. Node is now ACTIVE.")
                self.is_active = True
            else:
                rospy.loginfo("FTG_Node: Received DEACTIVATION signal. Node is now INACTIVE.")
                self.is_active = False
        else:
            rospy.logdebug("FTG_Node: Received redundant enable signal: {}".format(msg.data))

    # --- 【原有】导航模式的回调函数 ---
    def mode_callback(self, msg):
        """
        接收来自导航管理器的模式指令，并更新当前状态。
        """
        # 避免在模式未改变时重复打印日志
        if self.current_mode != msg.data:
            rospy.loginfo("FTG_Node: Received new mode command: {}. Updating status.".format(msg.data))
            self.current_mode = msg.data

    def _package_ftg_data(self, header, proc_ranges, proc_ranges_before_bubble, closest_index, gap_start, gap_end, best_point_index):
        """
        一个私有辅助函数，用于将所有中间数据打包成一个FtgData消息对象。
        
        参数:
            header: LaserScan消息的header
            proc_ranges: 处理后的雷达数据（包含安全气泡）
            proc_ranges_before_bubble: 创建安全气泡前的雷达数据
            closest_index: 最近障碍物的索引
            gap_start: 最大间隙起始索引
            gap_end: 最大间隙结束索引
            best_point_index: 最佳目标点索引
            
        返回:
            ftg_msg: 打包好的FtgData消息对象
        """
        ftg_msg = FtgData()
        ftg_msg.header = header
        ftg_msg.proc_ranges = proc_ranges.tolist()
        ftg_msg.proc_ranges_before_bubble = proc_ranges_before_bubble.tolist()
        ftg_msg.radians_per_elem = self.radians_per_elem
        ftg_msg.closest_index = closest_index
        ftg_msg.gap_start = gap_start
        ftg_msg.gap_end = gap_end
        ftg_msg.best_point_index = best_point_index

        return ftg_msg

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
        
        rospy.loginfo("Parameters updated: bubble_radius={}, max_lidar_dist={:.2f}".format(
            self.BUBBLE_RADIUS, self.MAX_LIDAR_DIST))
        
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
        if angle_deg < self.ANGLE_THRESH_1:
            speed = self.SPEED_TIER_1
            steering_gain = self.GAIN_TIER_1
            rospy.logdebug("Tiered Control: High-speed straight ({}°)".format(angle_deg))
        elif angle_deg < self.ANGLE_THRESH_2:
            speed = self.SPEED_TIER_2
            steering_gain = self.GAIN_TIER_2
            rospy.logdebug("Tiered Control: Medium-speed gentle turn ({}°)".format(angle_deg))
        elif angle_deg < self.ANGLE_THRESH_3:
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
        【修改】适配实体小车的 AckermannDrive 消息类型 (非Stamped版本)

        参数:
            speed (float): 车辆的目标速度。
            steering_angle (float): 车辆的目标转向角。
        """
        # 创建一个更简单的 AckermannDrive 消息对象 (没有header)
        drive_msg = AckermannDrive()
        
        # 直接填充消息的 drive 部分
        drive_msg.steering_angle = steering_angle
        drive_msg.speed = speed
        
        # 通过发布者发布消息
        self.drive_pub.publish(drive_msg)

    def process_lidar(self, data):
        """
        雷达寻隙算法(Follow The Gap)的主体实现，专注于计算和控制
        
        参数:
            data: LaserScan消息
        """
        # 获取 header 用于数据打包
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
        
        # 5. 调用新的打包函数，将所有数据打包成一个对象
        ftg_data_package = self._package_ftg_data(header, proc_ranges, proc_ranges_before_bubble, 
                                                   closest_index, gap_start, gap_end, best_point_index)

        # 6. 通过新创建的Publisher将这个数据包发布出去，供可视化节点使用
        self.ftg_data_pub.publish(ftg_data_package)
        
        # --- 分层控制逻辑 ---
        # 7. 计算原始目标角度（未经增益处理）
        center_index = len(proc_ranges) / 2.0
        raw_angle = (best_point_index - center_index) * self.radians_per_elem
        
        # 8. 使用分层控制函数一次性获取速度和最终转向角
        speed, final_steering_angle = self.get_tiered_control(raw_angle)
        
        # 9. 发布控制指令
        self.publish_control_command(speed, final_steering_angle)


    def radar_callback(self, data):
        """
        雷达回调函数,处理激光雷达数据
        
        参数:
            data: 雷达数据消息
        """
        # --- 【新增】第一层保护：检查激活状态 ---
        # 如果节点未被激活，则立即返回，不执行任何操作
        if not self.is_active:
            rospy.logdebug("FTG Node is INACTIVE. Ignoring radar data.")
            return

        # --- 【原有】第二层保护：检查导航模式 ---
        # 如果当前的导航模式不是FTG模式，则立即返回，不执行任何操作。
        # 这会有效地"暂停"FTG算法。
        if self.current_mode != "FTG_MODE":
            rospy.logdebug("Current navigation mode is not FTG_MODE. Ignoring radar data.")
            return

        # 如果两个条件都满足，则执行原有的所有逻辑
        self.process_lidar(data)

    def run(self):
        """
        保持节点运行
        """
        rospy.loginfo("Follow The Gap Computation Node is running.")
        rospy.loginfo("This node focuses on FTG algorithm computation and control commands.")
        rospy.loginfo("Visualization data is published to /ftg/debug_data for visualization node.")
        rospy.loginfo("Use rqt_reconfigure to adjust parameters.")
        rospy.loginfo("Send activation signal to /ftg/enable topic to start FTG algorithm.")
        rospy.spin()


if __name__ == '__main__':
    try:
        # 创建类的实例并运行
        node = FollowTheGapNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Follow The Gap Computation Node has been interrupted.")
        pass