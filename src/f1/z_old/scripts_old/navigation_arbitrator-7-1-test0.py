#!/usr/bin/env python
# coding=utf-8

"""
Navigation Arbitrator Node
导航仲裁器节点

根据机器人的位置，自动决定激活FTG还是TEB导航模式。
当机器人在指定区域内时使用TEB，在区域外时使用FTG。
"""

import rospy
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class NavigationArbitrator:
    """
    导航仲裁器节点。
    根据机器人的位置，决定激活FTG还是TEB导航模式。
    """
    
    def __init__(self):
        """
        构造函数：初始化节点、参数、发布者和订阅者。
        """
        # 1. 初始化ROS节点
        rospy.init_node('navigation_arbitrator', anonymous=True)
        
        # 2. 加载参数
        self.load_parameters()
        
        # 3. 初始化内部状态变量
        self.current_mode = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        # 4. 初始化发布者
        self.mode_publisher = rospy.Publisher(
            '/navigation/select_mode', 
            String, 
            queue_size=1, 
            latch=True
        )
        
        # 5. 初始化订阅者
        self.odom_subscriber = rospy.Subscriber(
            '/tianracer/odom', 
            Odometry, 
            self.odom_callback
        )
        
        # 6. 日志输出初始化信息
        rospy.loginfo("Navigation Arbitrator initialized")
        rospy.loginfo(f"TEB zone: center=({self.teb_center_x:.2f}, {self.teb_center_y:.2f}), radius={self.teb_radius:.2f}")
    
    def load_parameters(self):
        """
        从ROS参数服务器加载配置参数。
        """
        # TEB区域参数
        self.teb_center_x = rospy.get_param('~teb_zone/center_x', 1.0)
        self.teb_center_y = rospy.get_param('~teb_zone/center_y', 1.0)
        self.teb_radius = rospy.get_param('~teb_zone/radius', 1.0)
        
        # 发布频率控制
        self.update_rate = rospy.get_param('~update_rate', 10.0)
        
        # 里程计话题名
        self.odom_topic = rospy.get_param('~odom_topic', '/tianracer/odom')
        
        # 模式选择话题名
        self.mode_topic = rospy.get_param('~mode_topic', '/navigation/select_mode')
    
    def odom_callback(self, odom_msg):
        """
        里程计回调函数。每当收到新的位置信息时，此函数被调用。
        
        Args:
            odom_msg (nav_msgs/Odometry): 里程计消息
        """
        # 提取机器人位置
        self.robot_x = odom_msg.pose.pose.position.x
        self.robot_y = odom_msg.pose.pose.position.y
        
        # 决定导航模式
        desired_mode = self.determine_navigation_mode()
        
        # 检查是否需要切换模式
        if desired_mode != self.current_mode:
            self.switch_mode(desired_mode)
    
    def determine_navigation_mode(self):
        """
        根据当前机器人位置确定应该使用的导航模式。
        
        Returns:
            str: 'teb' 或 'ftg'
        """
        # 计算到TEB区域中心的距离
        distance_to_center = math.sqrt(
            (self.robot_x - self.teb_center_x)**2 + 
            (self.robot_y - self.teb_center_y)**2
        )
        
        # 判断模式
        if distance_to_center <= self.teb_radius:
            return 'teb'
        else:
            return 'ftg'
    
    def switch_mode(self, new_mode):
        """
        切换导航模式。
        
        Args:
            new_mode (str): 新的导航模式 ('teb' 或 'ftg')
        """
        # 更新内部状态
        self.current_mode = new_mode
        
        # 创建并发布消息
        mode_msg = String()
        mode_msg.data = self.current_mode
        self.mode_publisher.publish(mode_msg)
        
        # 记录切换信息
        rospy.loginfo(
            f"Navigation mode switched to: {self.current_mode.upper()} "
            f"at position ({self.robot_x:.2f}, {self.robot_y:.2f})"
        )
    
    def get_status_info(self):
        """
        获取当前状态信息，用于调试和监控。
        
        Returns:
            dict: 包含当前状态的字典
        """
        distance_to_center = math.sqrt(
            (self.robot_x - self.teb_center_x)**2 + 
            (self.robot_y - self.teb_center_y)**2
        )
        
        return {
            'current_mode': self.current_mode,
            'robot_position': (self.robot_x, self.robot_y),
            'teb_zone_center': (self.teb_center_x, self.teb_center_y),
            'teb_zone_radius': self.teb_radius,
            'distance_to_center': distance_to_center,
            'in_teb_zone': distance_to_center <= self.teb_radius
        }
    
    def run(self):
        """
        主运行函数，让节点持续运行并等待回调。
        """
        rospy.loginfo("Navigation Arbitrator is running...")
        
        # 设置状态输出定时器（可选，用于调试）
        if rospy.get_param('~debug_output', False):
            rospy.Timer(rospy.Duration(2.0), self.debug_timer_callback)
        
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation Arbitrator shutting down...")
    
    def debug_timer_callback(self, event):
        """
        调试输出定时器回调函数。
        """
        status = self.get_status_info()
        rospy.loginfo(
            f"Status: mode={status['current_mode']}, "
            f"pos=({status['robot_position'][0]:.2f}, {status['robot_position'][1]:.2f}), "
            f"dist={status['distance_to_center']:.2f}, "
            f"in_zone={status['in_teb_zone']}"
        )


if __name__ == '__main__':
    """
    程序入口点
    """
    try:
        # 创建并运行导航仲裁器节点
        arbitrator = NavigationArbitrator()
        arbitrator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation Arbitrator node terminated.")
    except Exception as e:
        rospy.logerr(f"Navigation Arbitrator encountered an error: {e}")