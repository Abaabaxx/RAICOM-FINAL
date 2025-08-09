#!/usr/bin/env python
# coding=utf-8

# 版本说明：该版本为将 FTG节点（含 RVIZ可视化功能）的耦合节点解耦合后的专注于 RVIZ可视化 的节点
# 功能说明：FTG的RVIZ可视化节点

"""
FTG可视化节点 - 专门用于RViz可视化

本节点负责：
1. 订阅来自FTG计算节点的调试数据
2. 将数据转换为RViz可视化格式
3. 发布各种可视化话题：
   - 处理后的雷达点云
   - 安全气泡区域
   - 最大间隙
   - 关键点标记（最近障碍物点和最佳目标点）

使用方法：
python /home/lby/tianbot_ws/src/f1/scripts/ftg_visualization_node.py

可视化话题：
- /ftg/processed_points: 处理后的雷达点云 (白色)
- /ftg/safety_bubble: 安全气泡区域 (黄色)  
- /ftg/max_gap: 最大间隙 (绿色)
- /ftg/markers: 最近障碍物(红球)和目标点(蓝球)
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

# 导入FTG数据消息类型
from f1.msg import FtgData


class FtgVisualizerNode:
    def __init__(self):
        """
        构造函数，初始化可视化节点
        """
        rospy.init_node("ftg_visualizer_node")
        
        # --- 【新增】一个订阅者，订阅打包好的数据 ---
        rospy.Subscriber('/ftg/debug_data', FtgData, self.visualization_callback)

        # --- 【新增】所有原来在 ftg_node.py 中用于可视化的Publisher ---
        self.processed_points_pub = rospy.Publisher('/ftg/processed_points', PointCloud2, queue_size=1)
        self.bubble_pub = rospy.Publisher('/ftg/safety_bubble', PointCloud2, queue_size=1)
        self.gap_pub = rospy.Publisher('/ftg/max_gap', PointCloud2, queue_size=1)
        self.marker_pub = rospy.Publisher('/ftg/markers', MarkerArray, queue_size=1)

        # 初始化一些参数，从参数服务器获取或设为默认值
        self.BUBBLE_RADIUS = rospy.get_param('~bubble_radius', 100)
        
        rospy.loginfo("FTG Visualizer Node has been initialized.")
        rospy.loginfo("Subscribing to /ftg/debug_data for visualization data.")
        rospy.loginfo("Publishing visualization topics:")
        rospy.loginfo("  - /ftg/processed_points: 处理后的雷达点云 (白色)")
        rospy.loginfo("  - /ftg/safety_bubble: 安全气泡区域 (黄色)")
        rospy.loginfo("  - /ftg/max_gap: 最大间隙 (绿色)")
        rospy.loginfo("  - /ftg/markers: 最近障碍物(红球)和目标点(蓝球)")

    def visualization_callback(self, msg):
        """
        接收来自计算节点的数据包，并调用主函数进行可视化处理。
        
        参数:
            msg: FtgData消息对象
        """
        # 直接将收到的消息传递给可视化处理函数
        self.publish_visualization(msg)

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

    def publish_visualization(self, ftg_data):
        """
        发布可视化数据到RViz。
        接收一个FtgData消息对象作为参数。
        
        参数:
            ftg_data: 包含所有FTG中间数据的FtgData消息对象
        """
        # --- 从消息对象中解包数据 ---
        header = ftg_data.header
        proc_ranges = np.array(ftg_data.proc_ranges)
        proc_ranges_before_bubble = np.array(ftg_data.proc_ranges_before_bubble)
        closest_index = ftg_data.closest_index
        gap_start = ftg_data.gap_start
        gap_end = ftg_data.gap_end
        best_point_index = ftg_data.best_point_index
        # 从ftg_data中获取radians_per_elem以供转换函数使用
        self.radians_per_elem = ftg_data.radians_per_elem
        
        # --- 以下的所有代码与原来的可视化逻辑完全相同 ---
        
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

    def run(self):
        """
        保持节点运行
        """
        rospy.loginfo("FTG Visualizer Node is running and ready to receive data.")
        rospy.spin()


if __name__ == '__main__':
    try:
        # 创建类的实例并运行
        node = FtgVisualizerNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("FTG Visualizer Node has been interrupted.")
        pass