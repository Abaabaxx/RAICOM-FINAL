#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 该版本只支持3个TEB区域和3个目标航点
#该版本实现了触发TEB导航的区域的可视化，并且可视化了目标航点
"""
独立的TEB区域可视化节点 (Standalone TEB Zone Visualizer)
作者: Gemini
功能: 作为一个dynamic_reconfigure客户端，监听navigation_manager节点的参数，
      并在RViz中可视化TEB区域，无需修改navigation_manager源码。
"""

import rospy
import math
from visualization_msgs.msg import Marker, MarkerArray
from dynamic_reconfigure import client
from tf.transformations import quaternion_from_euler  # 添加：导入四元数转换函数

class StandaloneVisualizer:
    def __init__(self):
        """初始化独立的可视化节点"""
        rospy.init_node('teb_zone_standalone_visualizer', anonymous=True)
        rospy.loginfo("独立的TEB区域可视化节点启动...")

        # 1. 创建发布者，用于向RViz发布MarkerArray
        #    RViz将订阅这个'/visualization/teb_zones'话题
        self.marker_publisher = rospy.Publisher(
            '/visualization/teb_zones',
            MarkerArray,
            queue_size=1,
            latch=True  #保证了信息不会丢失

        )

        # 2. 【核心】创建dynamic_reconfigure客户端
        #    它会连接到 'navigation_manager' 节点的reconfigure服务
        #    一旦服务端的参数变化，就会自动调用 self.reconfig_callback
        try:
            self.dyn_client = client.Client(
                'nav_manager',  # 这是您第一个脚本中rospy.init_node的节点名
                timeout=30,            # 连接超时时间
                config_callback=self.reconfig_callback
            )
            rospy.loginfo("成功连接到 'navigation_manager' 的参数服务器。")
        except rospy.ROSException as e:
            rospy.logerr(f"无法连接到 'navigation_manager' 节点的参数服务器: {e}")
            rospy.logerr("请确保 'navigation_manager' 节点正在运行！")
            return

        self.last_marker_count = 0

    def reconfig_callback(self, config):
        """
        当 'navigation_manager' 的参数更新时，此回调函数被调用。
        
        Args:
            config (dict): 包含所有最新参数的字典。
        """
        if not config:
            rospy.logwarn("收到了空的配置信息，跳过处理。")
            return

        rospy.loginfo("检测到来自 'navigation_manager' 的参数更新，正在生成可视化Marker...")
        
        active_zones = []
        MAX_ZONES = 3  # 与您的navigation_manager保持一致

        # 从收到的config字典中解析出启用的区域信息
        for i in range(1, MAX_ZONES + 1):
            if config.get(f'zone_{i}_enable'):
                try:
                    zone_info = {
                        "name":     config[f'zone_{i}_name'],
                        "center_x": config[f'zone_{i}_center_x'],
                        "center_y": config[f'zone_{i}_center_y'],
                        "radius":   config[f'zone_{i}_radius'],
                        # 添加：目标点位置和朝向信息
                        "goal_x": config[f'zone_{i}_goal_x'],
                        "goal_y": config[f'zone_{i}_goal_y'],
                        "goal_yaw_deg": config[f'zone_{i}_goal_yaw_deg'],
                    }
                    active_zones.append(zone_info)
                except KeyError as e:
                    rospy.logwarn(f"处理区域 {i} 时缺少关键参数: {e}")
        
        # 调用函数来发布Marker
        self.publish_zone_markers(active_zones)

    def publish_zone_markers(self, zones):
        """
        根据区域信息列表，创建并发布MarkerArray。

        Args:
            zones (list of dict): 包含所有启用的TEB区域信息的列表。
        """
        marker_array = MarkerArray()
        
        # 推荐做法：先发布一条DELETEALL消息，清除之前所有同命名空间的Marker
        # 这样可以确保禁用区域后，旧的Marker会消失，代码也更简洁。
        delete_all_marker = Marker()
        delete_all_marker.ns = "teb_zones"
        delete_all_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_all_marker)
        
        # 添加：为导航目标点创建一个DELETEALL消息
        delete_all_goals = Marker()
        delete_all_goals.ns = "teb_goals"
        delete_all_goals.action = Marker.DELETEALL
        marker_array.markers.append(delete_all_goals)
        
        # 为当前所有启用的区域创建新的Marker
        for i, zone in enumerate(zones):
            # 创建圆柱体Marker (区域)
            marker = Marker()
            marker.header.frame_id = "map"  # 假设区域定义在 "map" 坐标系
            marker.header.stamp = rospy.Time.now()
            
            marker.ns = "teb_zones"
            marker.id = i  # 每个marker有唯一的ID
            
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = zone["center_x"]
            marker.pose.position.y = zone["center_y"]
            marker.pose.position.z = 0.01  # 稍微抬高，避免与地面重叠
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = zone["radius"] * 2.0
            marker.scale.y = zone["radius"] * 2.0
            marker.scale.z = 0.1 # 圆柱体高度
            
            # 设置颜色（绿色，半透明）
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.4
            
            marker.lifetime = rospy.Duration() # 永久存在，直到被覆盖或删除
            
            marker_array.markers.append(marker)
            
            # 添加：创建箭头Marker (导航目标点)
            goal_marker = Marker()
            goal_marker.header.frame_id = "map"
            goal_marker.header.stamp = rospy.Time.now()
            
            goal_marker.ns = "teb_goals"
            goal_marker.id = i
            
            goal_marker.type = Marker.ARROW
            goal_marker.action = Marker.ADD
            
            # 设置位置 - 使用导航目标点坐标
            goal_marker.pose.position.x = zone["goal_x"]
            goal_marker.pose.position.y = zone["goal_y"]
            goal_marker.pose.position.z = 0.05  # 稍微抬高，避免与地面重叠
            
            # 设置朝向 - 将角度转换为四元数
            yaw_rad = math.radians(zone["goal_yaw_deg"])
            q = quaternion_from_euler(0, 0, yaw_rad)
            goal_marker.pose.orientation.x = q[0]
            goal_marker.pose.orientation.y = q[1]
            goal_marker.pose.orientation.z = q[2]
            goal_marker.pose.orientation.w = q[3]
            
            # 设置箭头尺寸
            goal_marker.scale.x = 0.8  # 箭头长度
            goal_marker.scale.y = 0.2  # 箭头宽度
            goal_marker.scale.z = 0.2  # 箭头高度
            
            # 设置颜色（红色，不透明）
            goal_marker.color.r = 1.0
            goal_marker.color.g = 0.0
            goal_marker.color.b = 0.0
            goal_marker.color.a = 1.0
            
            goal_marker.lifetime = rospy.Duration() # 永久存在
            
            marker_array.markers.append(goal_marker)

        # 发布最终的MarkerArray
        self.marker_publisher.publish(marker_array)
        rospy.loginfo(f"已向RViz发布 {len(zones)} 个TEB区域及其导航目标点。")

    def run(self):
        """保持节点运行"""
        rospy.spin()

if __name__ == '__main__':
    try:
        visualizer = StandaloneVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass