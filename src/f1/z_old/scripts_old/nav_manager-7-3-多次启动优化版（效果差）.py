#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 支持多次启动
# 该版本支持7个TEB区域
# 【修改】将TEB区域的配置改为从dynamic_reconfigure加载和动态调整
# 【新增】TEB导航7秒超时保护机制
# 【新增】命令话题支持，实现清晰的命令与参数分离
"""
导航管理器节点 (Navigation Manager Node)
作者: Auto-generated based on design specification
功能: 根据机器人位置自动切换FTG和TEB导航模式，支持RQT动态调参，具备超时保护，支持命令话题控制
"""

import rospy
import actionlib
import math
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

# 【新增】导入dynamic_reconfigure相关的库
from dynamic_reconfigure.server import Server
# 【新增】导入刚刚由catkin_make生成的配置文件模块 
# 注意：您需要将YOUR_PACKAGE_NAME替换为您的实际包名
from f1.cfg import NavigationParamsConfig


class NavigationManager:
    """导航管理器类 - 负责自动切换导航模式，具备超时保护和命令话题控制"""
    
    def __init__(self):
        """初始化导航管理器"""
        rospy.init_node('navigation_manager', anonymous=True)
        rospy.loginfo("导航管理器节点启动中...")
        
        # 内部状态变量
        self.current_mode = "FTG_MODE"
        self.active_teb_zone_name = None
        self.teb_zones = []  # 【修改】teb_zones现在由动态配置回调函数管理
        
        # 【新增】用于TEB任务超时的计时器
        self.teb_timeout_timer = None

        # 【【【核心修改：设置Dynamic Reconfigure服务器】】】
        # 这会启动一个服务，RQT可以连接到这个服务
        # 当RQT中的参数被修改时，它会自动调用 self.dynamic_reconfigure_callback
        self.dyn_reconfig_server = Server(NavigationParamsConfig, self.dynamic_reconfigure_callback)
        rospy.loginfo("Dynamic Reconfigure服务器已启动，您现在可以使用RQT进行调参。")
        
        # 【【【核心新增：创建命令订阅者】】】
        # 订阅 /navigation/command 话题，消息类型为 String
        # 当收到消息时，调用 self.command_callback 函数
        self.command_subscriber = rospy.Subscriber(
            '/navigation/command', 
            String, 
            self.command_callback,
            queue_size=5  # 可以缓存几条命令
        )
        rospy.loginfo("已启动命令监听器，在话题 /navigation/command 上等待命令。")
        
        # 创建发布者 - latching发布者确保新节点能收到当前状态
        self.mode_publisher = rospy.Publisher('/navigation/mode', String, 
                                            queue_size=1, latch=True)
        
        # 创建订阅者
        self.odom_subscriber = rospy.Subscriber('/tianracer/odom', Odometry, 
                                              self.on_odometry_received)
        
        # 创建move_base action客户端
        self.move_base_client = actionlib.SimpleActionClient('tianracer/move_base', MoveBaseAction)
        
        # 等待move_base服务器启动
        rospy.loginfo("等待move_base服务器...")
        if self.move_base_client.wait_for_server(timeout=rospy.Duration(30.0)):
            rospy.loginfo("move_base服务器连接成功")
        else:
            rospy.logwarn("move_base服务器连接超时，但节点将继续运行")
        
        # 发布初始状态
        self.publish_mode(self.current_mode)
        rospy.loginfo(f"导航管理器已启动，初始模式: {self.current_mode}")
        
        # 当前机器人位置（用于调试和日志）
        self.current_x = 0.0
        self.current_y = 0.0

    # 【【【核心新增：命令回调函数】】】
    def command_callback(self, msg):
        """
        处理从 /navigation/command 话题接收到的命令。
        """
        command = msg.data.strip().upper()  # 获取命令字符串，去除首尾空格并转为大写
        rospy.loginfo(f"接收到命令: '{command}'")

        if command == "RESET_TO_FTG_MODE":
            # 如果当前已经是FTG模式，可以简单提示一下
            if self.current_mode == "FTG_MODE":
                rospy.loginfo("当前已经是FTG模式，无需重置。")
                return
            
            rospy.logwarn("执行来自命令的强制复位，切换到FTG模式...")
            # 调用我们新设计的安全切换逻辑
            self.safe_switch_to_ftg("命令 'RESET_TO_FTG_MODE' 触发")
        
        elif command == "CANCEL_CURRENT_GOAL":
            rospy.loginfo("接收到取消当前目标的命令")
            self.cancel_all_move_base_goals()
            
        elif command == "STATUS":
            status = self.get_current_status()
            rospy.loginfo(f"状态查询 - 模式: {status['current_mode']}, "
                         f"位置: ({status['current_position'][0]:.2f}, "
                         f"{status['current_position'][1]:.2f}), "
                         f"活动区域: {status['active_zone']}, "
                         f"总区域数: {status['total_zones']}")
        
        # 你未来还可以扩展其他命令
        # elif command == "PAUSE_AUTONOMY":
        #     # 执行暂停逻辑
        # elif command == "RESUME_AUTONOMY":
        #     # 执行恢复逻辑
        else:
            rospy.logwarn(f"接收到未知命令: '{command}'，已忽略。")
            rospy.loginfo("支持的命令: RESET_TO_FTG_MODE, CANCEL_CURRENT_GOAL, STATUS")

    def safe_switch_to_ftg(self, reason):
        """
        一个安全的、集中的切换到FTG模式的函数。
        Args:
            reason (str): 切换原因，用于日志记录。
        """
        rospy.loginfo(f"正在安全切换到FTG模式。原因: {reason}")
        
        # 1. 如果有活动的超时计时器，关闭它
        if self.teb_timeout_timer:
            self.teb_timeout_timer.shutdown()
            self.teb_timeout_timer = None
            rospy.loginfo("已关闭活动的TEB超时计时器。")
            
        # 2. 如果有活动的move_base任务，取消它
        self.cancel_all_move_base_goals()
        
        # 3. 重置活动区域名称
        self.active_teb_zone_name = None
        
        # 4. 如果已经是FTG模式，就没必要再切换了（防止重复调用）
        if self.current_mode == "FTG_MODE":
            rospy.loginfo("当前已经是FTG模式。")
            return
            
        # 5. 更新内部状态并发布
        self.current_mode = "FTG_MODE"
        self.publish_mode(self.current_mode)
        rospy.loginfo("已成功切换到FTG模式。")

    # 【【【核心修改：Dynamic Reconfigure的回调函数】】】
    def dynamic_reconfigure_callback(self, config, level):
        """
        当通过RQT修改参数时，此函数被调用。
        它负责根据新的配置更新 self.teb_zones 列表。
        
        Args:
            config: 包含所有参数的配置对象
            level: 配置级别（通常不使用）
            
        Returns:
            config: 必须返回配置对象
        """
        rospy.loginfo("检测到参数更新，正在重新加载TEB区域配置...")
        
        new_zones = []
        # 【关键修改】从3个区域扩展到7个区域
        MAX_ZONES = 7

        for i in range(1, MAX_ZONES + 1):
            # 检查该区域是否被启用
            if config[f'zone_{i}_enable']:
                try:
                    zone = {
                        "name":     config[f'zone_{i}_name'],
                        "center_x": config[f'zone_{i}_center_x'],
                        "center_y": config[f'zone_{i}_center_y'],
                        "radius":   config[f'zone_{i}_radius'],
                        "goal": {
                            "x":      config[f'zone_{i}_goal_x'],
                            "y":      config[f'zone_{i}_goal_y'],
                            "yaw":    math.radians(config[f'zone_{i}_goal_yaw_deg'])
                        }
                    }
                    new_zones.append(zone)
                    rospy.logdebug(f"已加载动态配置区域: {zone['name']}")
                except KeyError as e:
                    rospy.logerr(f"处理动态区域 {i} 时出错: 键 {e} 缺失")

        # 原子性地更新区域列表，防止在检查时列表被修改
        self.teb_zones = new_zones
        rospy.loginfo(f"配置更新完成，当前已启用 {len(self.teb_zones)} 个TEB区域。")
        
        # 必须返回config对象
        return config

    def on_odometry_received(self, odom_msg):
        """
        里程计回调函数 - 主要决策点
        
        Args:
            odom_msg (Odometry): 里程计消息
        """
        # 获取当前位置
        self.current_x = odom_msg.pose.pose.position.x
        self.current_y = odom_msg.pose.pose.position.y
                
        if self.current_mode == "FTG_MODE":
            # 检查是否进入了任何一个可以触发TEB的区域
            zone_found = self.check_if_in_any_zone(self.current_x, self.current_y)
            
            if zone_found is not None:
                # 机器人刚从FTG区域进入TEB区域，触发切换
                rospy.loginfo(f"机器人进入 {zone_found['name']} 区域，切换到TEB模式")
                self.switch_to_teb_mode(zone_found)

    def check_if_in_any_zone(self, x, y):
        """
        检查给定坐标是否在任何一个圆形的TEB区域内
        
        Args:
            x (float): 机器人当前X坐标
            y (float): 机器人当前Y坐标
            
        Returns:
            dict or None: 如果在区域内返回区域信息，否则返回None
        """
        for zone in self.teb_zones:
            # 计算机器人位置到区域中心的距离的平方
            # (使用平方距离可以避免开根号，计算效率更高)
            dist_sq = (x - zone["center_x"])**2 + (y - zone["center_y"])**2
            
            # 计算区域半径的平方
            radius_sq = zone["radius"]**2
            
            # 如果距离的平方小于等于半径的平方，则点在圆内
            if dist_sq <= radius_sq:
                # 防止在同一区域重复触发
                if zone["name"] != self.active_teb_zone_name:
                    return zone
        return None

    def switch_to_teb_mode(self, zone_info):
        """
        切换到TEB模式，并启动一个7秒的超时计时器。
        
        Args:
            zone_info (dict): 区域信息字典
        """
        # 更新内部状态
        self.current_mode = "TEB_MODE"
        self.active_teb_zone_name = zone_info["name"]
        
        # 公告新状态
        self.publish_mode(self.current_mode)
        
        # 准备并发送导航目标给move_base
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # 设置目标位置
        goal.target_pose.pose.position.x = zone_info["goal"]["x"]
        goal.target_pose.pose.position.y = zone_info["goal"]["y"]
        goal.target_pose.pose.position.z = 0.0
        
        # 转换yaw角度为四元数
        quat = quaternion_from_euler(0, 0, zone_info["goal"]["yaw"])
        goal.target_pose.pose.orientation = Quaternion(*quat)
        
        # 发送目标，并指定任务完成后的回调函数
        rospy.loginfo(f"发送TEB导航目标到 ({zone_info['goal']['x']:.2f}, "
                     f"{zone_info['goal']['y']:.2f}, "
                     f"{math.degrees(zone_info['goal']['yaw']):.1f}°)")
        
        self.move_base_client.send_goal(goal, done_cb=self.on_teb_goal_done)
        
        # 【【【核心新增：启动超时计时器】】】
        # 创建一个7秒后触发一次的Timer
        # oneshot=True 表示它只触发一次，然后自动停止
        rospy.loginfo("启动7秒导航超时计时器...")
        self.teb_timeout_timer = rospy.Timer(rospy.Duration(7.0), 
                                             self.on_teb_timeout, 
                                             oneshot=True)

    # 【【【核心修改：超时回调函数】】】
    def on_teb_timeout(self, event):
        """
        TEB导航任务超时后调用的函数。
        """
        # 检查是否仍处于TEB模式，防止在模式切换的瞬间发生冲突
        if self.current_mode == "TEB_MODE":
            rospy.logwarn(f"TEB导航任务 '{self.active_teb_zone_name}' 超时 (超过7秒)!")
            # 【修改】调用安全切换函数
            self.safe_switch_to_ftg("TEB任务超时")

    def on_teb_goal_done(self, status, result):
        """
        move_base任务完成的回调函数
        
        Args:
            status: 目标状态
            result: 执行结果
        """
        # 【【【核心修改：关闭超时计时器】】】
        # 任务已经结束（无论成功失败），必须关闭计时器，防止它稍后触发
        if self.teb_timeout_timer:
            self.teb_timeout_timer.shutdown()
            self.teb_timeout_timer = None
            rospy.logdebug("TEB任务正常结束，已关闭超时计时器。")

        from actionlib_msgs.msg import GoalStatus
        
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo(f"TEB任务 {self.active_teb_zone_name} 成功完成")
        elif status == GoalStatus.PREEMPTED:
            rospy.logwarn(f"TEB任务 {self.active_teb_zone_name} 被取消")
        elif status == GoalStatus.ABORTED:
            rospy.logerr(f"TEB任务 {self.active_teb_zone_name} 执行失败")
        else:
            rospy.logwarn(f"TEB任务 {self.active_teb_zone_name} 结束，状态: {status}")
        
        # 【修改】无论成功失败，都通过安全函数切换回FTG模式
        self.safe_switch_to_ftg("TEB任务结束")

    def cancel_all_move_base_goals(self):
        """取消所有move_base目标"""
        if self.move_base_client.get_state() in [actionlib.GoalStatus.PENDING, 
                                                actionlib.GoalStatus.ACTIVE]:
            rospy.loginfo("取消当前move_base目标")
            self.move_base_client.cancel_all_goals()

    def publish_mode(self, mode):
        """
        发布导航模式
        
        Args:
            mode (str): 导航模式
        """
        mode_msg = String()
        mode_msg.data = mode
        self.mode_publisher.publish(mode_msg)
        rospy.logdebug(f"发布导航模式: {mode}")

    def get_current_status(self):
        """获取当前状态信息（用于调试）"""
        return {
            "current_mode": self.current_mode,
            "active_zone": self.active_teb_zone_name,
            "current_position": (self.current_x, self.current_y),
            "total_zones": len(self.teb_zones),
            "has_active_timer": self.teb_timeout_timer is not None
        }

    def run(self):
        """主运行循环"""
        rospy.loginfo("导航管理器正在运行...")
        
        # 创建定时器，定期输出状态信息（可选）
        status_timer = rospy.Timer(rospy.Duration(10.0), self.print_status)
        
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("导航管理器节点关闭")
        finally:
            status_timer.shutdown()
            # 【新增】清理超时计时器
            if self.teb_timeout_timer:
                self.teb_timeout_timer.shutdown()

    def print_status(self, event):
        """定期打印状态信息"""
        status = self.get_current_status()
        timer_status = "有活动计时器" if status['has_active_timer'] else "无计时器"
        rospy.loginfo(f"状态报告 - 模式: {status['current_mode']}, "
                     f"位置: ({status['current_position'][0]:.2f}, "
                     f"{status['current_position'][1]:.2f}), "
                     f"活动区域: {status['active_zone']}, "
                     f"计时器状态: {timer_status}")


def main():
    """主函数"""
    try:
        # 创建并运行导航管理器
        nav_manager = NavigationManager()
        nav_manager.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被用户中断")
    except Exception as e:
        rospy.logerr(f"导航管理器发生错误: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())


if __name__ == '__main__':
    main()