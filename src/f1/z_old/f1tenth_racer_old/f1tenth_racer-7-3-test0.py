#!/usr/bin/env python
# coding=utf-8

# 该版本有状态转换BUG，失败
#优化了启动和重置的逻辑，确保在FTG模式下未使能时只发送使能命令，
"""
智能切换脚本 (Flip Switch Script)
作者: Auto-generated based on design specification
功能: 智能控制导航管理器的状态机，实现启动/重置功能
"""

import rospy
from std_msgs.msg import String, Bool


class FlipSwitch:
    """智能切换控制器 - 不维护状态，每次运行时读取并决策"""
    
    def __init__(self):
        """初始化控制器"""
        rospy.init_node('flip_switch', anonymous=True)
        rospy.loginfo("智能切换脚本启动...")
        
        # 创建命令发布者
        self.mode_pub = rospy.Publisher('/navigation/set_mode', String, queue_size=1)
        self.enable_pub = rospy.Publisher('/navigation/set_enable', Bool, queue_size=1)
        
        # 等待发布者准备就绪
        rospy.sleep(0.05)

    def read_current_state(self):
        """
        读取当前系统状态
        
        Returns:
            tuple: (current_mode, is_enabled)
        """
        rospy.loginfo("正在读取当前系统状态...")
        
        try:
            # 读取当前模式
            mode_msg = rospy.wait_for_message('/navigation/mode', String, timeout=5.0)
            current_mode = mode_msg.data
            
            # 读取当前使能状态
            enable_msg = rospy.wait_for_message('/ftg/enable', Bool, timeout=5.0)
            is_enabled = enable_msg.data
            
            rospy.loginfo(f"当前状态: mode={current_mode}, enabled={is_enabled}")
            return current_mode, is_enabled
            
        except rospy.ROSException as e:
            rospy.logerr(f"读取状态失败: {e}")
            rospy.logerr("请确保导航管理器正在运行")
            return None, None

    def execute_decision(self, current_mode, is_enabled):
        """
        执行决策逻辑
        
        Args:
            current_mode (str): 当前模式
            is_enabled (bool): 当前使能状态
        """
        # 【启动条件】模式为FTG_MODE且未使能
        if current_mode == "FTG_MODE" and not is_enabled:
            rospy.loginfo("检测到启动条件：FTG模式且未使能 -> 发送启动命令")
            
            # 只发送使能命令
            enable_msg = Bool()
            enable_msg.data = True
            self.enable_pub.publish(enable_msg)
            
            rospy.loginfo("已发送启动命令：enable=True")
            
        # 【重置条件】其他所有情况
        else:
            rospy.loginfo("检测到重置条件 -> 执行完全重置")
            
            # 发送两个命令进行完全重置
            # 1. 强制切换到FTG模式
            mode_msg = String()
            mode_msg.data = "FTG_MODE"
            self.mode_pub.publish(mode_msg)
            
            # 2. 禁用系统
            enable_msg = Bool()
            enable_msg.data = False
            self.enable_pub.publish(enable_msg)
            
            rospy.loginfo("已发送重置命令：mode=FTG_MODE, enable=False")

    def run(self):
        """运行主逻辑"""
        rospy.loginfo("开始执行智能切换逻辑...")
        
        # 读取当前状态
        current_mode, is_enabled = self.read_current_state()
        
        # 如果读取失败，退出
        if current_mode is None:
            rospy.logerr("无法读取系统状态，退出")
            return False
        
        # 执行决策
        self.execute_decision(current_mode, is_enabled)
        
        # 给系统一点时间处理命令
        # rospy.sleep(0.5)
        
        # 验证结果（可选）
        rospy.loginfo("命令已发送，正在验证结果...")
        try:
            # 再次读取状态进行验证
            new_mode, new_enabled = self.read_current_state()
            if new_mode is not None:
                rospy.loginfo(f"操作完成，新状态: mode={new_mode}, enabled={new_enabled}")
        except:
            rospy.logwarn("无法验证操作结果，但命令已发送")
        
        return True


def main():
    """主函数"""
    try:
        # 创建并运行智能切换器
        flip_switch = FlipSwitch()
        success = flip_switch.run()
        
        if success:
            rospy.loginfo("智能切换脚本执行完成")
        else:
            rospy.logerr("智能切换脚本执行失败")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被用户中断")
    except Exception as e:
        rospy.logerr(f"智能切换脚本发生错误: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())


if __name__ == '__main__':
    main()