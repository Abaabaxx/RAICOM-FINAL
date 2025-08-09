#!/usr/bin/env python
# coding=utf-8

# TEB模式下，完成导航任务后才会继续仲裁
"""
Navigation Arbitrator Node with State Machine
状态化导航仲裁器节点

基于状态机的导航任务管理器，支持以下三种状态：
- MONITORING: 监控机器人位置，检测是否进入任务区域
- NAVIGATING: 执行导航任务，等待move_base完成
- COOLDOWN: 任务完成后的冷却期，防止重复触发
"""

import rospy
import math
import actionlib
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

class NavigationArbitrator:
    """
    状态化导航仲裁器节点
    使用状态机管理导航任务的完整生命周期
    """
    
    def __init__(self):
        """
        构造函数：初始化节点、参数、发布者、订阅者和action客户端
        """
        # 1. 初始化ROS节点
        rospy.init_node('navigation_arbitrator_stateful', anonymous=True)
        
        # 2. 加载参数
        self.load_parameters()
        
        # 3. 初始化状态机相关变量
        self.state = 'MONITORING'  # 初始状态为监控状态
        self.active_goal_zone_name = None  # 记录当前/上一个任务的区域名
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.current_mode = None
        self.cooldown_timer = None
        
        # 4. 初始化action客户端
        rospy.loginfo("Connecting to move_base action server...")
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        # 等待move_base服务器连接
        if not self.move_base_client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Failed to connect to move_base action server!")
            rospy.signal_shutdown("Cannot connect to move_base")
            return
        
        rospy.loginfo("Successfully connected to move_base action server")
        
        # 5. 初始化发布者
        self.mode_publisher = rospy.Publisher(
            self.mode_topic, 
            String, 
            queue_size=1, 
            latch=True
        )
        
        # 6. 初始化订阅者
        self.odom_subscriber = rospy.Subscriber(
            self.odom_topic, 
            Odometry, 
            self.odom_callback
        )
        
        # 7. 日志输出初始化信息
        rospy.loginfo("Stateful Navigation Arbitrator initialized")
        rospy.loginfo(f"Loaded {len(self.teb_zones)} TEB zones")
        rospy.loginfo(f"Cooldown duration: {self.cooldown_duration} seconds")
        rospy.loginfo(f"Initial state: {self.state}")
        
        for i, zone in enumerate(self.teb_zones):
            zone_name = zone.get('name', f'Zone_{i+1}')
            rospy.loginfo(f"{zone_name}: center=({zone['center_x']:.2f}, {zone['center_y']:.2f}), radius={zone['radius']:.2f}")
            if 'goal' in zone:
                goal_pos = zone['goal']['position']
                rospy.loginfo(f"  -> Goal: ({goal_pos['x']:.2f}, {goal_pos['y']:.2f})")
    
    def load_parameters(self):
        """
        从ROS参数服务器加载配置参数
        """
        # 加载TEB区域列表
        self.teb_zones = rospy.get_param('~teb_zones', [])
        
        # 如果没有配置任何区域，使用默认配置
        if not self.teb_zones:
            rospy.logwarn("No teb_zones configured, using default configuration")
            self.teb_zones = [{
                'name': 'default_zone',
                'center_x': 1.0,
                'center_y': 1.0,
                'radius': 1.0
            }]
        
        # 验证区域配置
        self.validate_zones()
        
        # 其他参数
        self.update_rate = rospy.get_param('~update_rate', 10.0)
        self.odom_topic = rospy.get_param('~odom_topic', '/tianracer/odom')
        self.mode_topic = rospy.get_param('~mode_topic', '/navigation/select_mode')
        
        # 新增：状态机相关参数
        self.cooldown_duration = rospy.get_param('~cooldown_duration', 5.0)  # 冷却时间
    
    def validate_zones(self):
        """
        验证加载的区域配置是否有效，包括目标点的验证
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
        里程计回调函数 - 状态机的主驱动器
        
        Args:
            odom_msg (nav_msgs/Odometry): 里程计消息
        """
        # 无条件更新机器人位置
        self.robot_x = odom_msg.pose.pose.position.x
        self.robot_y = odom_msg.pose.pose.position.y
        
        # 根据当前状态执行相应操作
        if self.state == 'MONITORING':
            self.handle_monitoring_state()
        # NAVIGATING和COOLDOWN状态下不执行任何操作，忽略位置更新
    
    def handle_monitoring_state(self):
        """
        处理MONITORING状态下的逻辑
        """
        # 确定当前应该使用的模式和区域
        desired_mode, active_zone = self.determine_mode_and_zone()
        
        # 判断是否需要启动新任务
        if (desired_mode == 'teb' and 
            active_zone is not None and 
            'goal' in active_zone and 
            active_zone['name'] != self.active_goal_zone_name):
            
            # 检测到新任务
            rospy.loginfo(f"Detected new navigation task in zone '{active_zone['name']}'")
            self.send_goal_and_wait(active_zone)
            
        elif desired_mode == 'teb' and self.current_mode != 'teb':
            # 只需要切换模式到TEB，无任务
            self.switch_mode_topic('teb')
            
        elif desired_mode == 'ftg':
            # 切换到FTG模式，重置active_goal_zone_name
            if self.current_mode != 'ftg':
                self.switch_mode_topic('ftg')
            self.active_goal_zone_name = None
    
    def determine_mode_and_zone(self):
        """
        根据当前机器人位置确定应该使用的导航模式和所在的区域
        
        Returns:
            tuple: (mode_string, zone_dict_or_None)
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
                return 'teb', zone
        
        # 如果机器人不在任何TEB区域内，使用FTG模式
        return 'ftg', None
    
    def send_goal_and_wait(self, zone_data):
        """
        启动导航任务的核心函数
        
        Args:
            zone_data (dict): 包含目标信息的区域数据
        """
        # 第一步：立即切换状态
        self.state = 'NAVIGATING'
        rospy.loginfo(f"State changed to: {self.state}")
        
        # 第二步：记录当前任务的区域名
        self.active_goal_zone_name = zone_data['name']
        
        # 第三步：切换模式到TEB
        self.switch_mode_topic('teb')
        
        # 第四步：构建MoveBaseGoal消息
        try:
            goal = MoveBaseGoal()
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.header.frame_id = "map"
            
            # 设置位置
            goal_data = zone_data['goal']
            goal.target_pose.pose.position.x = float(goal_data['position']['x'])
            goal.target_pose.pose.position.y = float(goal_data['position']['y'])
            goal.target_pose.pose.position.z = float(goal_data['position']['z'])
            
            # 设置姿态
            goal.target_pose.pose.orientation.x = float(goal_data['orientation']['x'])
            goal.target_pose.pose.orientation.y = float(goal_data['orientation']['y'])
            goal.target_pose.pose.orientation.z = float(goal_data['orientation']['z'])
            goal.target_pose.pose.orientation.w = float(goal_data['orientation']['w'])
            
            # 第五步：发送目标并设置完成回调
            rospy.loginfo(f"Sending navigation goal for zone '{zone_data['name']}':")
            rospy.loginfo(f"  Position: ({goal.target_pose.pose.position.x:.2f}, {goal.target_pose.pose.position.y:.2f})")
            
            self.move_base_client.send_goal(goal, done_cb=self.goal_done_callback)
            
        except (KeyError, ValueError, TypeError) as e:
            rospy.logerr(f"Failed to send goal for zone '{zone_data['name']}': {e}")
            # 发生错误时，立即进入冷却状态
            self.enter_cooldown()
    
    def goal_done_callback(self, status, result):
        """
        导航任务完成回调函数
        
        Args:
            status (int): 任务状态码
            result: 任务结果
        """
        status_text = {
            GoalStatus.SUCCEEDED: "SUCCEEDED",
            GoalStatus.ABORTED: "ABORTED", 
            GoalStatus.PREEMPTED: "PREEMPTED",
            GoalStatus.REJECTED: "REJECTED"
        }.get(status, f"UNKNOWN({status})")
        
        rospy.loginfo(f"Navigation task completed with status: {status_text}")
        rospy.loginfo(f"Zone '{self.active_goal_zone_name}' task finished")
        
        # 无论任务结果如何，都进入冷却状态
        self.enter_cooldown()
    
    def enter_cooldown(self):
        """
        进入冷却状态
        """
        self.state = 'COOLDOWN'
        rospy.loginfo(f"State changed to: {self.state}")
        rospy.loginfo(f"Entering cooldown period for {self.cooldown_duration} seconds")
        
        # 启动一次性定时器
        self.cooldown_timer = rospy.Timer(
            rospy.Duration(self.cooldown_duration), 
            self.exit_cooldown, 
            oneshot=True
        )
    
    def exit_cooldown(self, event):
        """
        退出冷却状态的定时器回调函数
        
        Args:
            event: 定时器事件
        """
        self.state = 'MONITORING'
        rospy.loginfo(f"State changed to: {self.state}")
        rospy.loginfo("Cooldown period ended, resuming monitoring")
        
        # 清理定时器引用
        if self.cooldown_timer:
            self.cooldown_timer.shutdown()
            self.cooldown_timer = None
    
    def switch_mode_topic(self, new_mode):
        """
        发布模式切换消息
        
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
        获取当前状态信息，用于调试和监控
        
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
            'state_machine_state': self.state,
            'current_mode': self.current_mode,
            'active_goal_zone': self.active_goal_zone_name,
            'robot_position': (self.robot_x, self.robot_y),
            'total_zones': len(self.teb_zones),
            'in_any_teb_zone': in_any_zone,
            'zone_details': zone_distances,
            'cooldown_duration': self.cooldown_duration
        }
    
    def run(self):
        """
        主运行函数，让节点持续运行并等待回调
        """
        rospy.loginfo("Stateful Navigation Arbitrator is running...")
        
        # 设置状态输出定时器（可选，用于调试）
        if rospy.get_param('~debug_output', False):
            rospy.Timer(rospy.Duration(3.0), self.debug_timer_callback)
        
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation Arbitrator shutting down...")
            # 清理资源
            if self.cooldown_timer:
                self.cooldown_timer.shutdown()
    
    def debug_timer_callback(self, event):
        """
        调试输出定时器回调函数
        """
        status = self.get_status_info()
        rospy.loginfo(
            f"Status: state={status['state_machine_state']}, "
            f"mode={status['current_mode']}, "
            f"active_zone={status['active_goal_zone']}, "
            f"pos=({status['robot_position'][0]:.2f}, {status['robot_position'][1]:.2f})"
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
        # 创建并运行状态化导航仲裁器节点
        arbitrator = NavigationArbitrator()
        arbitrator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Stateful Navigation Arbitrator node terminated.")
    except Exception as e:
        rospy.logerr(f"Stateful Navigation Arbitrator encountered an error: {e}")