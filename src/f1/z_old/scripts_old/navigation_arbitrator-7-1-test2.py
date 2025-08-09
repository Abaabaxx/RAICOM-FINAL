#!/usr/bin/env python
# coding=utf-8

"""
Navigation Arbitrator Node with Goal Publishing
导航仲裁器节点 - 带目标点发布功能

根据机器人的位置，自动决定激活FTG还是TEB导航模式。
当机器人在指定区域内时使用TEB，在区域外时使用FTG。
支持多个TEB区域配置，并能自动发送对应的导航目标点。
"""

import rospy
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID

class NavigationArbitrator:
    """
    导航仲裁器节点 - 任务分发器版本。
    根据机器人的位置，决定激活FTG还是TEB导航模式，
    并自动发送对应区域的导航目标点。
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
        self.current_teb_zone_name = None  # 记录当前激活的TEB区域名称
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.last_goal_time = rospy.Time(0)  # 记录上次发送目标的时间
        
        # 4. 初始化发布者
        self.mode_publisher = rospy.Publisher(
            self.mode_topic, 
            String, 
            queue_size=1, 
            latch=True
        )
        
        # 新增：目标点发布者
        self.goal_publisher = rospy.Publisher(
            '/move_base_simple/goal', 
            PoseStamped, 
            queue_size=1
        )
        
        # 新增：目标取消发布者（可选）
        self.cancel_publisher = rospy.Publisher(
            '/move_base/cancel', 
            GoalID, 
            queue_size=1
        )
        
        # 5. 初始化订阅者
        self.odom_subscriber = rospy.Subscriber(
            self.odom_topic, 
            Odometry, 
            self.odom_callback
        )
        
        # 6. 日志输出初始化信息
        rospy.loginfo("Navigation Arbitrator with Goal Publishing initialized")
        rospy.loginfo(f"Loaded {len(self.teb_zones)} TEB zones with goal points")
        for i, zone in enumerate(self.teb_zones):
            zone_name = zone.get('name', f'Zone_{i+1}')
            rospy.loginfo(f"{zone_name}: center=({zone['center_x']:.2f}, {zone['center_y']:.2f}), radius={zone['radius']:.2f}")
            if 'goal' in zone:
                goal_pos = zone['goal']['position']
                rospy.loginfo(f"  -> Goal: ({goal_pos['x']:.2f}, {goal_pos['y']:.2f})")
    
    def load_parameters(self):
        """
        从ROS参数服务器加载配置参数。
        """
        # 加载TEB区域列表（新的多区域支持）
        self.teb_zones = rospy.get_param('~teb_zones', [])
        
        # 如果没有配置任何区域，使用原来的单区域参数作为后备
        if not self.teb_zones:
            rospy.logwarn("No teb_zones configured, falling back to legacy single zone parameters")
            try:
                legacy_center_x = rospy.get_param('~teb_zone/center_x', 1.0)
                legacy_center_y = rospy.get_param('~teb_zone/center_y', 1.0)
                legacy_radius = rospy.get_param('~teb_zone/radius', 1.0)
                
                # 创建一个包含单个区域的列表（不包含目标点）
                self.teb_zones = [{
                    'name': 'legacy_zone',
                    'center_x': legacy_center_x,
                    'center_y': legacy_center_y,
                    'radius': legacy_radius
                }]
                rospy.loginfo("Successfully loaded legacy single zone configuration (no goal points)")
            except Exception as e:
                rospy.logwarn(f"Failed to load legacy parameters: {e}")
                # 使用默认区域
                self.teb_zones = [{
                    'name': 'default_zone',
                    'center_x': 1.0,
                    'center_y': 1.0,
                    'radius': 1.0
                }]
                rospy.logwarn("Using default zone configuration (no goal points)")
        
        # 验证区域配置的有效性
        self.validate_zones()
        
        # 其他参数
        self.update_rate = rospy.get_param('~update_rate', 10.0)
        self.odom_topic = rospy.get_param('~odom_topic', '/tianracer/odom')
        self.mode_topic = rospy.get_param('~mode_topic', '/navigation/select_mode')
        
        # 新增：目标发布相关参数
        self.goal_timeout = rospy.get_param('~goal_timeout', 5.0)  # 防止重复发送目标的时间间隔
        self.auto_cancel_goals = rospy.get_param('~auto_cancel_goals', True)  # 是否自动取消目标
    
    def validate_zones(self):
        """
        验证加载的区域配置是否有效，包括目标点的验证。
        """
        valid_zones = []
        for i, zone in enumerate(self.teb_zones):
            try:
                # 检查基本必需的键
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
                
                # 如果没有名称，自动生成一个
                if 'name' not in zone:
                    zone['name'] = f'zone_{i+1}'
                
                # 验证目标点配置（如果存在）
                if 'goal' in zone:
                    goal = zone['goal']
                    if 'position' in goal and 'orientation' in goal:
                        # 验证位置
                        pos = goal['position']
                        if all(key in pos for key in ['x', 'y', 'z']):
                            float(pos['x'])
                            float(pos['y'])
                            float(pos['z'])
                        else:
                            rospy.logwarn(f"Zone {zone['name']} has invalid goal position, removing goal")
                            del zone['goal']
                            continue
                        
                        # 验证姿态
                        ori = goal['orientation']
                        if all(key in ori for key in ['x', 'y', 'z', 'w']):
                            float(ori['x'])
                            float(ori['y'])
                            float(ori['z'])
                            float(ori['w'])
                        else:
                            rospy.logwarn(f"Zone {zone['name']} has invalid goal orientation, removing goal")
                            del zone['goal']
                            continue
                        
                        rospy.loginfo(f"Zone {zone['name']} has valid goal point")
                    else:
                        rospy.logwarn(f"Zone {zone['name']} has incomplete goal definition, removing goal")
                        del zone['goal']
                
                valid_zones.append(zone)
                
            except (TypeError, ValueError) as e:
                rospy.logwarn(f"Zone {i+1} has invalid parameters: {e}, skipping")
                continue
        
        self.teb_zones = valid_zones
        
        if not self.teb_zones:
            rospy.logerr("No valid TEB zones configured! Using emergency default zone.")
            self.teb_zones = [{
                'name': 'emergency_zone',
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
        
        # 决定导航模式和区域
        desired_mode, active_zone = self.determine_mode_and_zone()
        
        # 处理模式切换和目标发布逻辑
        self.handle_mode_and_goal_logic(desired_mode, active_zone)
    
    def determine_mode_and_zone(self):
        """
        根据当前机器人位置确定应该使用的导航模式和所在的区域。
        
        Returns:
            tuple: (mode_string, zone_dict_or_None)
                - mode_string: 'teb' 或 'ftg'
                - zone_dict_or_None: 如果在TEB区域内则返回区域字典，否则返回None
        """
        # 遍历所有TEB区域
        for zone in self.teb_zones:
            # 计算到当前区域中心的距离
            distance_to_center = math.sqrt(
                (self.robot_x - zone['center_x'])**2 + 
                (self.robot_y - zone['center_y'])**2
            )
            
            # 如果机器人在当前区域内
            if distance_to_center <= zone['radius']:
                if rospy.get_param('~debug_output', False):
                    rospy.logdebug(f"Robot in TEB zone '{zone['name']}' (distance: {distance_to_center:.2f})")
                return 'teb', zone
        
        # 如果机器人不在任何TEB区域内，使用FTG模式
        return 'ftg', None
    
    def handle_mode_and_goal_logic(self, desired_mode, active_zone):
        """
        处理模式切换和目标发布的核心逻辑。
        
        Args:
            desired_mode (str): 期望的导航模式 ('teb' 或 'ftg')
            active_zone (dict or None): 当前激活的区域信息
        """
        if desired_mode == 'teb' and active_zone is not None:
            # 获取当前区域名称
            zone_name = active_zone['name']
            
            # 检查是否进入了一个新的TEB区域
            if zone_name != self.current_teb_zone_name:
                rospy.loginfo(f"Entering new TEB zone: '{zone_name}'")
                
                # 切换模式到TEB（如果当前不是TEB的话）
                if self.current_mode != 'teb':
                    self.switch_mode_topic('teb')
                
                # 发布此区域对应的导航目标（如果有的话）
                if 'goal' in active_zone:
                    self.publish_goal(active_zone['goal'], zone_name)
                else:
                    rospy.loginfo(f"Zone '{zone_name}' has no goal point configured")
                
                # 更新当前激活的区域名称
                self.current_teb_zone_name = zone_name
            
            # 如果仍在同一个TEB区域内，什么都不做（避免重复发送）
            
        else:  # desired_mode is 'ftg' or active_zone is None
            # 如果之前在TEB区域，现在出来了
            if self.current_mode == 'teb':
                rospy.loginfo("Exiting TEB zone, switching to FTG mode")
                self.switch_mode_topic('ftg')
                
                # 可选：取消当前的导航目标
                if self.auto_cancel_goals:
                    self.cancel_current_goal()
            
            # 重置当前激活的TEB区域名称
            self.current_teb_zone_name = None
    
    def switch_mode_topic(self, new_mode):
        """
        发布模式切换消息。
        
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
    
    def publish_goal(self, goal_data, zone_name):
        """
        根据传入的goal数据，构建并发布一个PoseStamped目标。
        
        Args:
            goal_data (dict): 包含position和orientation的目标点数据
            zone_name (str): 区域名称，用于日志记录
        """
        current_time = rospy.Time.now()
        
        # 防止短时间内重复发送目标（可选的保护机制）
        if (current_time - self.last_goal_time).to_sec() < 1.0:
            rospy.logdebug("Skipping goal publication due to rate limiting")
            return
        
        try:
            # 构建PoseStamped消息
            goal_msg = PoseStamped()
            goal_msg.header.stamp = current_time
            goal_msg.header.frame_id = "map"  # 或者你的全局坐标系
            
            # 设置位置
            goal_msg.pose.position.x = float(goal_data['position']['x'])
            goal_msg.pose.position.y = float(goal_data['position']['y'])
            goal_msg.pose.position.z = float(goal_data['position']['z'])
            
            # 设置姿态
            goal_msg.pose.orientation.x = float(goal_data['orientation']['x'])
            goal_msg.pose.orientation.y = float(goal_data['orientation']['y'])
            goal_msg.pose.orientation.z = float(goal_data['orientation']['z'])
            goal_msg.pose.orientation.w = float(goal_data['orientation']['w'])
            
            # 发布目标
            self.goal_publisher.publish(goal_msg)
            self.last_goal_time = current_time
            
            rospy.loginfo(
                f"Published new goal for zone '{zone_name}': "
                f"pos({goal_msg.pose.position.x:.2f}, {goal_msg.pose.position.y:.2f}), "
                f"ori({goal_msg.pose.orientation.z:.3f}, {goal_msg.pose.orientation.w:.3f})"
            )
            
        except (KeyError, ValueError, TypeError) as e:
            rospy.logerr(f"Failed to publish goal for zone '{zone_name}': {e}")
    
    def cancel_current_goal(self):
        """
        取消当前的move_base目标。
        """
        try:
            cancel_msg = GoalID()
            # 空的GoalID表示取消所有目标
            self.cancel_publisher.publish(cancel_msg)
            rospy.loginfo("Cancelled current move_base goal")
        except Exception as e:
            rospy.logwarn(f"Failed to cancel goal: {e}")
    
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
            
            zone_info = {
                'zone_name': zone.get('name', f'zone_{i+1}'),
                'center': (zone['center_x'], zone['center_y']),
                'radius': zone['radius'],
                'distance': distance,
                'is_inside': is_in_zone,
                'has_goal': 'goal' in zone
            }
            
            if 'goal' in zone:
                goal_pos = zone['goal']['position']
                zone_info['goal_position'] = (goal_pos['x'], goal_pos['y'])
            
            zone_distances.append(zone_info)
        
        return {
            'current_mode': self.current_mode,
            'current_teb_zone': self.current_teb_zone_name,
            'robot_position': (self.robot_x, self.robot_y),
            'total_zones': len(self.teb_zones),
            'in_any_teb_zone': in_any_zone,
            'zone_details': zone_distances,
            'goal_publishing_enabled': True
        }
    
    def run(self):
        """
        主运行函数，让节点持续运行并等待回调。
        """
        rospy.loginfo("Navigation Arbitrator with Goal Publishing is running...")
        
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
            f"zone={status['current_teb_zone']}, "
            f"pos=({status['robot_position'][0]:.2f}, {status['robot_position'][1]:.2f}), "
            f"total_zones={status['total_zones']}"
        )
        
        # 详细输出当前激活区域的状态
        for zone_info in status['zone_details']:
            if zone_info['is_inside']:
                goal_info = f", goal: {zone_info['goal_position']}" if zone_info['has_goal'] else ", no goal"
                rospy.loginfo(f"  -> In Zone '{zone_info['zone_name']}': distance={zone_info['distance']:.2f}{goal_info}")


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