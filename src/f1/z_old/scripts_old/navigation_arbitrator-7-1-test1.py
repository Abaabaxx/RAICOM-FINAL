#!/usr/bin/env python
# coding=utf-8

#支持多区域转换TEB
"""
Navigation Arbitrator Node
导航仲裁器节点

根据机器人的位置，自动决定激活FTG还是TEB导航模式。
当机器人在指定区域内时使用TEB，在区域外时使用FTG。
支持多个TEB区域配置。
"""

import rospy
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class NavigationArbitrator:
    """
    导航仲裁器节点。
    根据机器人的位置，决定激活FTG还是TEB导航模式。
    支持多个TEB区域配置。
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
            self.mode_topic, 
            String, 
            queue_size=1, 
            latch=True
        )
        
        # 5. 初始化订阅者
        self.odom_subscriber = rospy.Subscriber(
            self.odom_topic, 
            Odometry, 
            self.odom_callback
        )
        
        # 6. 日志输出初始化信息
        rospy.loginfo("Navigation Arbitrator initialized")
        rospy.loginfo(f"Loaded {len(self.teb_zones)} TEB zones")
        for i, zone in enumerate(self.teb_zones):
            rospy.loginfo(f"Zone {i+1}: center=({zone['center_x']:.2f}, {zone['center_y']:.2f}), radius={zone['radius']:.2f}")
    
    def load_parameters(self):
        """
        从ROS参数服务器加载配置参数。
        """
        # 加载TEB区域列表（新的多区域支持）
        self.teb_zones = rospy.get_param('~teb_zones', [])
        
        # 如果没有配置任何区域，使用原来的单区域参数作为后备
        if not self.teb_zones:
            rospy.logwarn("No teb_zones configured, falling back to legacy single zone parameters")
            # 尝试加载旧的参数格式
            try:
                legacy_center_x = rospy.get_param('~teb_zone/center_x', 1.0)
                legacy_center_y = rospy.get_param('~teb_zone/center_y', 1.0)
                legacy_radius = rospy.get_param('~teb_zone/radius', 1.0)
                
                # 创建一个包含单个区域的列表
                self.teb_zones = [{
                    'center_x': legacy_center_x,
                    'center_y': legacy_center_y,
                    'radius': legacy_radius
                }]
                rospy.loginfo("Successfully loaded legacy single zone configuration")
            except Exception as e:
                rospy.logwarn(f"Failed to load legacy parameters: {e}")
                # 如果都失败了，使用默认区域
                self.teb_zones = [{
                    'center_x': 1.0,
                    'center_y': 1.0,
                    'radius': 1.0
                }]
                rospy.logwarn("Using default zone configuration")
        
        # 验证区域配置的有效性
        self.validate_zones()
        
        # 发布频率控制
        self.update_rate = rospy.get_param('~update_rate', 10.0)
        
        # 里程计话题名
        self.odom_topic = rospy.get_param('~odom_topic', '/tianracer/odom')
        
        # 模式选择话题名
        self.mode_topic = rospy.get_param('~mode_topic', '/navigation/select_mode')
    
    def validate_zones(self):
        """
        验证加载的区域配置是否有效。
        """
        valid_zones = []
        for i, zone in enumerate(self.teb_zones):
            try:
                # 检查必需的键是否存在
                if not all(key in zone for key in ['center_x', 'center_y', 'radius']):
                    rospy.logwarn(f"Zone {i+1} missing required keys, skipping")
                    continue
                
                # 检查半径是否为正数
                if zone['radius'] <= 0:
                    rospy.logwarn(f"Zone {i+1} has invalid radius ({zone['radius']}), skipping")
                    continue
                
                # 检查坐标是否为数字
                float(zone['center_x'])
                float(zone['center_y'])
                float(zone['radius'])
                
                valid_zones.append(zone)
                
            except (TypeError, ValueError) as e:
                rospy.logwarn(f"Zone {i+1} has invalid parameters: {e}, skipping")
                continue
        
        self.teb_zones = valid_zones
        
        if not self.teb_zones:
            rospy.logerr("No valid TEB zones configured! Using emergency default zone.")
            self.teb_zones = [{
                'center_x': 0.0,
                'center_y': 0.0,
                'radius': 1.0
            }]
    
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
        遍历所有TEB区域，如果机器人在任何一个区域内，就使用TEB模式。
        
        Returns:
            str: 'teb' 或 'ftg'
        """
        # 遍历所有TEB区域
        for i, zone in enumerate(self.teb_zones):
            # 计算到当前区域中心的距离
            distance_to_center = math.sqrt(
                (self.robot_x - zone['center_x'])**2 + 
                (self.robot_y - zone['center_y'])**2
            )
            
            # 如果机器人在当前区域内
            if distance_to_center <= zone['radius']:
                # 可选：记录是哪个区域触发了TEB模式
                if rospy.get_param('~debug_output', False):
                    rospy.logdebug(f"Robot in TEB zone {i+1} (distance: {distance_to_center:.2f})")
                return 'teb'
        
        # 如果机器人不在任何TEB区域内，使用FTG模式
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
        # 计算到所有区域的距离信息
        zone_distances = []
        in_any_zone = False
        
        for i, zone in enumerate(self.teb_zones):
            distance = math.sqrt(
                (self.robot_x - zone['center_x'])**2 + 
                (self.robot_y - zone['center_y'])**2
            )
            is_in_zone = distance <= zone['radius']
            if is_in_zone:
                in_any_zone = True
            
            zone_distances.append({
                'zone_id': i + 1,
                'center': (zone['center_x'], zone['center_y']),
                'radius': zone['radius'],
                'distance': distance,
                'is_inside': is_in_zone
            })
        
        return {
            'current_mode': self.current_mode,
            'robot_position': (self.robot_x, self.robot_y),
            'total_zones': len(self.teb_zones),
            'in_any_teb_zone': in_any_zone,
            'zone_details': zone_distances
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
            f"zones={status['total_zones']}, "
            f"in_zone={status['in_any_teb_zone']}"
        )
        
        # 详细输出每个区域的状态
        for zone_info in status['zone_details']:
            if zone_info['is_inside']:
                rospy.loginfo(f"  -> In Zone {zone_info['zone_id']}: distance={zone_info['distance']:.2f}")


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