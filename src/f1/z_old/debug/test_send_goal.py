#!/usr/bin/env python
# -*- coding: utf-8 -*-

#经过测试，发现movebase服务名映射为tianracer/movebase，非默认的 movebase，需要修改

"""
独立测试节点: 发送单个move_base导航目标点
功能:
1. 初始化ROS节点。
2. 创建一个move_base的action客户端。
3. 等待move_base服务器连接。
4. 定义一个固定的导航目标点。
5. 发送目标点给move_base。
6. 等待并打印导航结果。
"""

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

def send_test_goal():
    """主函数，用于发送测试目标点"""
    
    # 1. 初始化ROS节点
    rospy.init_node('test_send_goal_node', anonymous=True)
    rospy.loginfo("测试节点启动，准备发送导航目标...")

    # 2. 创建一个move_base的action客户端
    #    'move_base' 是action服务器的名称，必须与你的系统匹配
    client = actionlib.SimpleActionClient('tianracer/move_base', MoveBaseAction)

    # 3. 等待move_base服务器连接
    rospy.loginfo("正在等待move_base action服务器...")
    # 设置一个超时时间，例如30秒。如果超时，程序会退出。
    if not client.wait_for_server(rospy.Duration(30.0)):
        rospy.logerr("连接move_base服务器超时！请确保move_base节点正在运行。")
        return
    rospy.loginfo("成功连接到move_base服务器！")

    # 4. 定义一个固定的导航目标点
    #    你可以修改下面的值来测试不同的目标点
    #    这里我们使用你导航管理器中的"cross"区域的目标点作为例子
    target_x = -17.86
    target_y = 1.946
    target_yaw_degrees = -38.5
    
    rospy.loginfo(f"准备发送目标点: (x={target_x}, y={target_y}, yaw={target_yaw_degrees}°)")

    # 创建一个MoveBaseGoal消息对象
    goal = MoveBaseGoal()
    
    # 设置坐标系和时间戳
    goal.target_pose.header.frame_id = "map"  # 确保这个坐标系是你的地图坐标系
    goal.target_pose.header.stamp = rospy.Time.now()

    # 设置目标位置
    goal.target_pose.pose.position.x = target_x
    goal.target_pose.pose.position.y = target_y
    goal.target_pose.pose.position.z = 0.0

    # 设置目标姿态（将角度转换为四元数）
    target_yaw_radians = math.radians(target_yaw_degrees)
    quat = quaternion_from_euler(0, 0, target_yaw_radians)
    goal.target_pose.pose.orientation = Quaternion(*quat)

    # 5. 发送目标点给move_base
    rospy.loginfo("正在发送目标...")
    client.send_goal(goal)

    # 6. 等待并打印导航结果
    rospy.loginfo("目标已发送，等待导航结果...")
    # wait_for_result() 是一个阻塞函数，它会一直等待直到任务结束
    client.wait_for_result()

    # 获取最终状态
    final_status = client.get_state()
    # from actionlib_msgs.msg import GoalStatus
    # GoalStatus.SUCCEEDED = 3
    # GoalStatus.ABORTED = 4
    # ...

    rospy.loginfo(f"导航任务结束，最终状态码: {final_status}")

    if final_status == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("导航成功！机器人已到达目标点。")
    else:
        rospy.logwarn("导航失败或被取消。")


if __name__ == '__main__':
    try:
        send_test_goal()
    except rospy.ROSInterruptException:
        rospy.loginfo("测试被用户中断。")
    except Exception as e:
        rospy.logerr(f"测试过程中发生错误: {e}")