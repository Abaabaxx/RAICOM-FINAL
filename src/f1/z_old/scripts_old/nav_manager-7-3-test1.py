#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 优化启动失败，废弃该版本
"""
导航管理器节点 (Navigation Manager Node) - 状态机版本
作者: Auto-generated based on design specification
功能: 作为状态机管理导航模式和使能状态，接受外部控制命令
"""

import rospy
import actionlib
import math
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from f1.cfg import NavigationParamsConfig


class NavigationManager:
    """导航管理器类 - 状态机，管理导航模式和使能状态"""
    
    def __init__(self):
        """初始化导航管理器"""
        rospy.init_node('navigation_manager', anonymous=True)
        rospy.loginfo("导航管理器节点(状态机版本)启动中...")
        
        # 【核心状态变量】- 系统的权威状态
        self.current_mode = "FTG_MODE"  # 导航模式
        self.is_enabled = False         # 使能状态
        
        # 内部工作变量
        self.active_teb_zone_name = None
        self.teb_zones = []
        self.teb_timeout_timer = None
        self.current_x = 0.0
        self.current_y = 0.0

        # 设置Dynamic Reconfigure服务器
        self.dyn_reconfig_server = Server(NavigationParamsConfig, self.dynamic_reconfigure_callback)
        rospy.loginfo("Dynamic Reconfigure服务器已启动")
        
        # 【状态发布者】- 广播权威状态 (使用latching确保新节点能收到)
        self.mode_publisher = rospy.Publisher('/navigation/mode', String, 
                                            queue_size=1, latch=True)
        self.enable_publisher = rospy.Publisher('/ftg/enable', Bool, 
                                              queue_size=1, latch=True)
        
        # 【命令接收者】- 接受外部控制命令
        self.set_enable_subscriber = rospy.Subscriber('/navigation/set_enable', Bool, 
                                                    self.on_set_enable_received)
        self.set_mode_subscriber = rospy.Subscriber('/navigation/set_mode', String, 
                                                   self.on_set_mode_received)
        
        # 里程计订阅者
        self.odom_subscriber = rospy.Subscriber('/tianracer/odom', Odometry, 
                                              self.on_odometry_received)
        
        # move_base客户端
        self.move_base_client = actionlib.SimpleActionClient('tianracer/move_base', MoveBaseAction)
        
        # 等待move_base服务器
        rospy.loginfo("等待move_base服务器...")
        if self.move_base_client.wait_for_server(timeout=rospy.Duration(30.0)):
            rospy.loginfo("move_base服务器连接成功")
        else:
            rospy.logwarn("move_base服务器连接超时，但节点将继续运行")
        
        # 【立即发布初始状态】
        self.publish_current_state()
        rospy.loginfo(f"导航管理器已启动，初始状态: mode={self.current_mode}, enabled={self.is_enabled}")

    def dynamic_reconfigure_callback(self, config, level):
        """Dynamic Reconfigure回调函数"""
        rospy.loginfo("检测到参数更新，正在重新加载TEB区域配置...")
        
        new_zones = []
        MAX_ZONES = 15

        for i in range(1, MAX_ZONES + 1):
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
                except KeyError as e:
                    rospy.logerr(f"处理动态区域 {i} 时出错: 键 {e} 缺失")

        self.teb_zones = new_zones
        rospy.loginfo(f"配置更新完成，当前已启用 {len(self.teb_zones)} 个TEB区域")
        return config

    # 【外部控制接口】
    def on_set_enable_received(self, msg):
        """
        接收外部使能控制命令
        
        Args:
            msg (Bool): 使能命令
        """
        old_enabled = self.is_enabled
        self.is_enabled = msg.data
        
        rospy.loginfo(f"收到使能命令: {self.is_enabled} (之前: {old_enabled})")
        
        # 【安全措施】如果被禁用，立即取消所有move_base目标
        if not self.is_enabled and old_enabled:
            rospy.loginfo("系统被禁用，取消所有导航目标")
            self.cancel_all_move_base_goals()
        
        # 重新发布使能状态
        self.publish_enable_state()

    def on_set_mode_received(self, msg):
        """
        接收外部模式控制命令
        
        Args:
            msg (String): 模式命令
        """
        old_mode = self.current_mode
        self.current_mode = msg.data
        
        rospy.loginfo(f"收到模式命令: {self.current_mode} (之前: {old_mode})")
        
        # 【安全措施】如果被强制设为FTG_MODE，取消目标并重置TEB状态
        if self.current_mode == "FTG_MODE" and old_mode != "FTG_MODE":
            rospy.loginfo("被强制切换到FTG模式，重置TEB状态")
            self.cancel_all_move_base_goals()
            self.reset_teb_state()
        
        # 重新发布模式状态
        self.publish_mode_state()

    def on_odometry_received(self, odom_msg):
        """
        里程计回调函数 - 自动模式切换逻辑
        
        Args:
            odom_msg (Odometry): 里程计消息
        """
        # 更新当前位置
        self.current_x = odom_msg.pose.pose.position.x
        self.current_y = odom_msg.pose.pose.position.y
        
        # 【关键逻辑】仅当模式为FTG_MODE且系统使能时，才检查TEB区域
        if self.current_mode == "FTG_MODE" and self.is_enabled:
            zone_found = self.check_if_in_any_zone(self.current_x, self.current_y)
            
            if zone_found is not None:
                rospy.loginfo(f"机器人进入 {zone_found['name']} 区域，切换到TEB模式")
                self.switch_to_teb_mode(zone_found)

    def check_if_in_any_zone(self, x, y):
        """检查是否在任何TEB区域内"""
        for zone in self.teb_zones:
            dist_sq = (x - zone["center_x"])**2 + (y - zone["center_y"])**2
            radius_sq = zone["radius"]**2
            
            if dist_sq <= radius_sq:
                if zone["name"] != self.active_teb_zone_name:
                    return zone
        return None

    def switch_to_teb_mode(self, zone_info):
        """
        切换到TEB模式
        
        Args:
            zone_info (dict): 区域信息
        """
        # 【状态机逻辑】先禁用系统，再切换模式
        rospy.loginfo("执行TEB切换：先禁用系统，再切换模式")
        
        # 1. 首先将系统禁用
        self.is_enabled = False
        self.publish_enable_state()
        
        # 2. 然后切换到TEB模式
        self.current_mode = "TEB_MODE"
        self.active_teb_zone_name = zone_info["name"]
        self.publish_mode_state()
        
        # 3. 发送move_base目标
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = zone_info["goal"]["x"]
        goal.target_pose.pose.position.y = zone_info["goal"]["y"]
        goal.target_pose.pose.position.z = 0.0
        
        quat = quaternion_from_euler(0, 0, zone_info["goal"]["yaw"])
        goal.target_pose.pose.orientation = Quaternion(*quat)
        
        rospy.loginfo(f"发送TEB导航目标到 ({zone_info['goal']['x']:.2f}, "
                     f"{zone_info['goal']['y']:.2f}, "
                     f"{math.degrees(zone_info['goal']['yaw']):.1f}°)")
        
        self.move_base_client.send_goal(goal, done_cb=self.on_teb_goal_done)
        
        # 启动超时计时器
        rospy.loginfo("启动15秒导航超时计时器...")
        self.teb_timeout_timer = rospy.Timer(rospy.Duration(15.0), 
                                           self.on_teb_timeout, 
                                           oneshot=True)

    def on_teb_timeout(self, event):
        """TEB导航超时处理"""
        if self.current_mode == "TEB_MODE":
            rospy.logwarn(f"TEB导航任务 '{self.active_teb_zone_name}' 超时 (超过15秒)!")
            self.cancel_all_move_base_goals()
            self.switch_to_ftg_mode("TEB任务超时")

    def on_teb_goal_done(self, status, result):
        """move_base任务完成回调"""
        # 关闭超时计时器
        if self.teb_timeout_timer:
            self.teb_timeout_timer.shutdown()
            self.teb_timeout_timer = None

        from actionlib_msgs.msg import GoalStatus
        
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo(f"TEB任务 {self.active_teb_zone_name} 成功完成")
        elif status == GoalStatus.PREEMPTED:
            rospy.logwarn(f"TEB任务 {self.active_teb_zone_name} 被取消")
        elif status == GoalStatus.ABORTED:
            rospy.logerr(f"TEB任务 {self.active_teb_zone_name} 执行失败")
        else:
            rospy.logwarn(f"TEB任务 {self.active_teb_zone_name} 结束，状态: {status}")
        
        self.switch_to_ftg_mode("TEB任务结束")

    def switch_to_ftg_mode(self, reason):
        """
        切换回FTG模式
        
        Args:
            reason (str): 切换原因
        """
        if self.current_mode == "FTG_MODE":
            return
        
        rospy.loginfo(f"切换回FTG模式。原因: {reason}")
        
        # 【状态机逻辑】只切换模式，不自动使能
        self.current_mode = "FTG_MODE"
        self.reset_teb_state()
        self.publish_mode_state()
        
        rospy.loginfo("已切换到FTG模式，系统保持禁用状态，等待外部使能命令")

    def reset_teb_state(self):
        """重置TEB相关状态"""
        self.active_teb_zone_name = None
        if self.teb_timeout_timer:
            self.teb_timeout_timer.shutdown()
            self.teb_timeout_timer = None

    def cancel_all_move_base_goals(self):
        """取消所有move_base目标"""
        if self.move_base_client.get_state() in [actionlib.GoalStatus.PENDING, 
                                                actionlib.GoalStatus.ACTIVE]:
            rospy.loginfo("取消当前move_base目标")
            self.move_base_client.cancel_all_goals()

    # 【状态发布函数】
    def publish_current_state(self):
        """发布当前完整状态"""
        self.publish_mode_state()
        self.publish_enable_state()

    def publish_mode_state(self):
        """发布模式状态"""
        mode_msg = String()
        mode_msg.data = self.current_mode
        self.mode_publisher.publish(mode_msg)
        rospy.logdebug(f"发布模式状态: {self.current_mode}")

    def publish_enable_state(self):
        """发布使能状态"""
        enable_msg = Bool()
        enable_msg.data = self.is_enabled
        self.enable_publisher.publish(enable_msg)
        rospy.logdebug(f"发布使能状态: {self.is_enabled}")

    def get_current_status(self):
        """获取当前状态信息"""
        return {
            "current_mode": self.current_mode,
            "is_enabled": self.is_enabled,
            "active_zone": self.active_teb_zone_name,
            "current_position": (self.current_x, self.current_y),
            "total_zones": len(self.teb_zones),
            "has_active_timer": self.teb_timeout_timer is not None
        }

    def run(self):
        """主运行循环"""
        rospy.loginfo("导航管理器状态机正在运行...")
        
        # 状态报告计时器
        status_timer = rospy.Timer(rospy.Duration(10.0), self.print_status)
        
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("导航管理器节点关闭")
        finally:
            status_timer.shutdown()
            if self.teb_timeout_timer:
                self.teb_timeout_timer.shutdown()

    def print_status(self, event):
        """定期打印状态"""
        status = self.get_current_status()
        timer_status = "有活动计时器" if status['has_active_timer'] else "无计时器"
        rospy.loginfo(f"状态报告 - 模式: {status['current_mode']}, "
                     f"使能: {status['is_enabled']}, "
                     f"位置: ({status['current_position'][0]:.2f}, "
                     f"{status['current_position'][1]:.2f}), "
                     f"活动区域: {status['active_zone']}, "
                     f"计时器: {timer_status}")


def main():
    """主函数"""
    try:
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