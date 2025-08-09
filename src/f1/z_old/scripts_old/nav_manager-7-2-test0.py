#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
导航管理器节点 (Navigation Manager Node)
作者: Auto-generated based on design specification
功能: 根据机器人位置自动切换FTG和TEB导航模式
"""

import rospy
import actionlib
import math
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler


class NavigationManager:
    """导航管理器类 - 负责自动切换导航模式"""
    
    def __init__(self):
        """初始化导航管理器"""
        rospy.init_node('navigation_manager', anonymous=True)
        rospy.loginfo("导航管理器节点启动中...")
        
        # 内部状态变量
        self.current_mode = "FTG_MODE"
        self.active_teb_zone_name = None

        # 【【【修改】】】 TEB区域定义 - 修改为圆形区域
        # 结构: {"name": str, "center_x": float, "center_y": float, "radius": float, "goal": dict}
        self.teb_zones = [
            {
                "name": "cross",
                "center_x": -20.0,  # 区域中心点X
                "center_y": 6.0,  # 区域中心点Y
                "radius": 0.5,    # 区域半径
                "goal": {"x": -17.86, "y": 1.946, "yaw": math.radians(-38.5)}
            },
            {
                "name": "Z_curve",
                "center_x": -8.5,
                "center_y": -1,
                "radius": 1,
                "goal": {"x": -6.1, "y": -2.8, "yaw": math.radians(-11.1)}
            },
            {
                "name": "alternative",
                "center_x": 999,
                "center_y": 999,
                "radius": 1.5,
                "goal": {"x": 0.0, "y": -4.0, "yaw": math.radians(180.0)}
            }
        ]
        
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
        【【【修改】】】 检查给定坐标是否在任何一个圆形的TEB区域内
        
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
        切换到TEB模式
        
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

    def on_teb_goal_done(self, status, result):
        """
        move_base任务完成的回调函数
        
        Args:
            status: 目标状态
            result: 执行结果
        """
        from actionlib_msgs.msg import GoalStatus
        
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo(f"TEB任务 {self.active_teb_zone_name} 成功完成")
        elif status == GoalStatus.PREEMPTED:
            rospy.logwarn(f"TEB任务 {self.active_teb_zone_name} 被取消")
        elif status == GoalStatus.ABORTED:
            rospy.logerr(f"TEB任务 {self.active_teb_zone_name} 执行失败")
        else:
            rospy.logwarn(f"TEB任务 {self.active_teb_zone_name} 结束，状态: {status}")
        
        # 无论成功还是失败，都切换回FTG模式
        self.switch_to_ftg_mode("TEB任务结束")

    def switch_to_ftg_mode(self, reason):
        """
        切换回FTG模式
        
        Args:
            reason (str): 切换原因
        """
        # 重置活动区域记录
        self.active_teb_zone_name = None
        
        # 如果已经是FTG模式，就没必要再切换了（防止重复调用）
        if self.current_mode == "FTG_MODE":
            return
        
        # 更新内部状态
        self.current_mode = "FTG_MODE"
        
        # 公告新状态
        self.publish_mode(self.current_mode)
        rospy.loginfo(f"切换回FTG模式。原因: {reason}")

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
            "total_zones": len(self.teb_zones)
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

    def print_status(self, event):
        """定期打印状态信息"""
        status = self.get_current_status()
        rospy.loginfo(f"状态报告 - 模式: {status['current_mode']}, "
                     f"位置: ({status['current_position'][0]:.2f}, "
                     f"{status['current_position'][1]:.2f}), "
                     f"活动区域: {status['active_zone']}")


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