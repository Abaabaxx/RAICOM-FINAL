#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 最终比赛说明：该版本为最终比赛使用的导航管理nav_manager节点！！！
# 版本说明：该版本是原始可用的，原始的最稳定版本（最后的事实证明，该版本的确为最终使用的版本）
# 该版本支持7个TEB区域（ctrl+f搜索7，将与TEB区域相关的7更换为想改变的区域数量即可【注意TEB导航超时时间为7S，不要修改错了】）
# 关键修改：将TEB区域的配置改为从dynamic_reconfigure加载和动态调整（最终证明，配置rqt_reconfigure的动态调参功能大大大大的方便了最后的调参，这个功能一定要好好学习）
# 新增：TEB导航7秒超时保护机制（ctrl+f搜索7，将与超时保护相关的7更换为想改变的超时时间即可【注意TEB区域默认数量也为7，不要修改错了】）
# 【新增】使用TF获取map下的base_link位姿，解决里程计累积误差问题

"""
导航管理器节点 (Navigation Manager Node)
作者: lby
功能: 根据机器人位置自动切换FTG和TEB导航模式，支持RQT动态调参，具备超时保护
"""

import rospy
import math
import tf  # 【新增】引入tf库
from std_msgs.msg import String, Bool  # 【新增】导入Bool消息类型
from ackermann_msgs.msg import AckermannDrive # 新增：用于直接发送速度指令

# 【新增】导入dynamic_reconfigure相关的库
from dynamic_reconfigure.server import Server
# 【新增】导入刚刚由catkin_make生成的配置文件模块 
# 注意：您需要将YOUR_PACKAGE_NAME替换为您的实际包名
from f1.cfg import NavigationParamsConfig


class NavigationManager:
    """导航管理器类 - 负责自动切换导航模式，具备超时保护"""
    
    def __init__(self):
        """初始化导航管理器"""
        rospy.init_node('navigation_manager', anonymous=True)
        rospy.loginfo("导航管理器节点启动中...")
        
        # 【新增】系统总开关状态变量
        self.system_active = False
        
        # 内部状态变量
        self.current_mode = "FTG_MODE"
        self.active_teb_zone_name = None
        self.teb_zones = []  # 【修改】teb_zones现在由动态配置回调函数管理
        
        # 【新增】位置检查定时器，初始为None，只有在系统激活时才会创建
        self.check_pose_timer = None

        # 【新增】TF监听器，用于获取map->base_link的修正后位姿
        self.tf_listener = tf.TransformListener()
        
        # 【新增】从参数服务器获取坐标系名称，增加灵活性
        self.map_frame = rospy.get_param("~map_frame", "Tianracer/map")
        self.base_frame = rospy.get_param("~base_frame", "Tianracer/base_link")
        rospy.loginfo("将使用TF树监听 {} -> {} 的位姿。".format(self.map_frame, self.base_frame))

        # 【【【核心修改：设置Dynamic Reconfigure服务器】】】
        # 这会启动一个服务，RQT可以连接到这个服务
        # 当RQT中的参数被修改时，它会自动调用 self.dynamic_reconfigure_callback
        self.dyn_reconfig_server = Server(NavigationParamsConfig, self.dynamic_reconfigure_callback)
        rospy.loginfo("Dynamic Reconfigure服务器已启动，您现在可以使用RQT进行调参。")
        
        # 创建发布者 - latching发布者确保新节点能收到当前状态
        self.mode_publisher = rospy.Publisher('/navigation/mode', String, 
                                            queue_size=1, latch=True)
        
        # 【新增】创建FTG控制发布者
        self.ftg_enable_publisher = rospy.Publisher('/ftg/enable', Bool, 
                                                queue_size=1, latch=True)
        
        # 新增：创建车辆控制指令发布者
        self.drive_pub = rospy.Publisher('/Tianracer/ackermann_cmd', AckermannDrive, queue_size=1)
        
        # 【新增】创建系统总开关订阅者
        self.toggle_sub = rospy.Subscriber('/navigation/toggle_system', Bool, 
                                        self.system_control_callback)
        
        # 发布初始状态
        self.publish_mode(self.current_mode)
        rospy.loginfo("导航管理器已启动，初始模式: {}".format(self.current_mode))
        rospy.loginfo("系统当前处于待机状态，等待通过 /navigation/toggle_system 话题激活...")
        
        # 当前机器人位置（用于调试和日志）
        self.current_x = 0.0
        self.current_y = 0.0

    # 【【【核心新增：Dynamic Reconfigure的回调函数】】】
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
            if config['zone_{}_enable'.format(i)]:
                try:
                    zone = {
                        "name":     config['zone_{}_name'.format(i)],
                        "center_x": config['zone_{}_center_x'.format(i)],
                        "center_y": config['zone_{}_center_y'.format(i)],
                        "radius":   config['zone_{}_radius'.format(i)],
                        "goal": {
                            "x":      config['zone_{}_goal_x'.format(i)],
                            "y":      config['zone_{}_goal_y'.format(i)],
                            "yaw":    math.radians(config['zone_{}_goal_yaw_deg'.format(i)])
                        }
                    }
                    new_zones.append(zone)
                    rospy.logdebug("已加载动态配置区域: {}".format(zone['name']))
                except KeyError as e:
                    rospy.logerr("处理动态区域 {} 时出错: 键 {} 缺失".format(i, e))

        # 原子性地更新区域列表，防止在检查时列表被修改
        self.teb_zones = new_zones
        rospy.loginfo("配置更新完成，当前已启用 {} 个TEB区域。".format(len(self.teb_zones)))
        
        # 必须返回config对象
        return config

    # 【新增】替换原有的on_odometry_received方法
    def check_position_callback(self, event):
        """
        定时器回调函数 - 这是新的主要决策点
        它通过TF获取机器人修正后的位姿，然后判断是否需要切换模式。
        
        Args:
            event: Timer event (not used)
        """
        # 通过TF查询机器人当前在map坐标系中的位姿
        try:
            # 查询最新的可用变换
            (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(2.0, "无法获取从 '{}' 到 '{}' 的位姿变换，暂时无法进行导航决策: {}".format(
                self.map_frame, self.base_frame, e))
            return

        # 获取当前位置
        self.current_x = trans[0]
        self.current_y = trans[1]
                
        zone_found = self.check_if_in_any_zone(self.current_x, self.current_y)

        if zone_found is not None:
            # 机器人进入TEB区域
            if self.current_mode != "TEB_MODE":
                rospy.loginfo("机器人进入 {} 区域 (位置: {:.2f}, {:.2f})，切换到TEB模式".format(
                    zone_found['name'], self.current_x, self.current_y))
                self.current_mode = "TEB_MODE"
                self.active_teb_zone_name = zone_found["name"]
                self.publish_mode(self.current_mode)
            
            # 持续发布直行指令
            self.publish_control_command(1.0, 0.0)

        else:
            # 机器人不在任何TEB区域内
            if self.current_mode != "FTG_MODE":
                self.switch_to_ftg_mode("已离开TEB区域")

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
                return zone
        return None

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
        rospy.loginfo("切换回FTG模式。原因: {}".format(reason))

    def publish_mode(self, mode):
        """
        发布导航模式
        
        Args:
            mode (str): 导航模式
        """
        mode_msg = String()
        mode_msg.data = mode
        self.mode_publisher.publish(mode_msg)
        rospy.logdebug("发布导航模式: {}".format(mode))

    def publish_control_command(self, speed, steering_angle):
        """创建并发布阿克曼驱动消息。"""
        drive_msg = AckermannDrive()
        drive_msg.steering_angle = steering_angle
        drive_msg.speed = speed
        self.drive_pub.publish(drive_msg)
        
    # 【新增】系统总开关的回调函数
    def system_control_callback(self, msg):
        """
        处理来自 /navigation/toggle_system 话题的指令。
        True = 激活系统, False = 停止系统。
        
        Args:
            msg: std_msgs/Bool 消息
        """
        if msg.data is True and not self.system_active:
            rospy.loginfo("收到系统启动指令，正在激活导航功能...")
            self.activate_system()
            self.system_active = True
        elif msg.data is False and self.system_active:
            rospy.loginfo("收到系统关闭指令，正在停止所有导航功能...")
            self.deactivate_system()
            self.system_active = False
        else:
            rospy.logdebug("收到冗余的系统控制指令，当前状态无需改变。")

    def activate_system(self):
        """
        激活整个导航系统:
        1. 启动FTG节点
        2. 设置默认导航模式为FTG
        3. 启动位置检查定时器
        """
        # 激活FTG节点
        ftg_msg = Bool()
        ftg_msg.data = True
        self.ftg_enable_publisher.publish(ftg_msg)
        
        # 确保导航模式为FTG_MODE
        self.current_mode = "FTG_MODE"
        self.publish_mode(self.current_mode)
        
        # 启动位置检查定时器
        if self.check_pose_timer is None:
            self.check_pose_timer = rospy.Timer(rospy.Duration(0.2), self.check_position_callback)  # 5Hz
            
        rospy.loginfo("导航系统已激活，当前模式: FTG_MODE")
        
    def deactivate_system(self):
        """
        停止整个导航系统:
        1. 停止FTG节点
        2. 取消所有move_base目标
        3. 停止位置检查定时器
        4. 重置内部状态
        """
        # 停止FTG节点
        ftg_msg = Bool()
        ftg_msg.data = False
        self.ftg_enable_publisher.publish(ftg_msg)
        
        # 发送停车指令
        self.publish_control_command(0.0, 0.0)
        
        # 停止位置检查定时器
        if self.check_pose_timer is not None:
            self.check_pose_timer.shutdown()
            self.check_pose_timer = None
            
        # 重置内部状态
        self.current_mode = "FTG_MODE"  # 默认模式
        self.active_teb_zone_name = None
            
        rospy.loginfo("导航系统已停止并重置，等待下次激活...")
        
    def get_current_status(self):
        """获取当前状态信息（用于调试）"""
        return {
            "system_active": self.system_active,
            "current_mode": self.current_mode,
            "active_zone": self.active_teb_zone_name,
            "current_position": (self.current_x, self.current_y),
            "total_zones": len(self.teb_zones),
            "has_active_timer": False # 删除了计时器
        }

    def run(self):
        """主运行循环"""
        rospy.loginfo("导航管理器正在运行...")
        
        # 创建定时器，定期输出状态信息（可选）
        status_timer = rospy.Timer(rospy.Duration(1.0), self.print_status)
        
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("导航管理器节点关闭")
        finally:
            status_timer.shutdown()
            # 【新增】清理位姿检查定时器
            if hasattr(self, 'check_pose_timer') and self.check_pose_timer:
                self.check_pose_timer.shutdown()

    def print_status(self, event):
        """定期打印状态信息"""
        status = self.get_current_status()
        timer_status = "有活动计时器" if status['has_active_timer'] else "无计时器"
        system_status = "已激活" if status['system_active'] else "待机中"
        rospy.loginfo("状态报告 - 系统: {}, 模式: {}, 位置: ({:.2f}, {:.2f}), 活动区域: {}, 计时器状态: {}".format(
            system_status,
            status['current_mode'],
            status['current_position'][0],
            status['current_position'][1],
            status['active_zone'],
            timer_status
        ))


def main():
    """主函数"""
    try:
        # 创建并运行导航管理器
        nav_manager = NavigationManager()
        nav_manager.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被用户中断")
    except Exception as e:
        rospy.logerr("导航管理器发生错误: {}".format(e))
        import traceback
        rospy.logerr(traceback.format_exc())


if __name__ == '__main__':
    main()