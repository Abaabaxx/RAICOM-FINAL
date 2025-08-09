#!/usr/bin/env python
# coding=utf-8


import rospy
from std_msgs.msg import Bool

def send_start_signal():
    """
    一个高时效性的启动脚本，使用latch模式发布一次启动信号。
    """
    # 1. 初始化ROS节点。 anonymous=True 确保节点名唯一。
    # disable_signals=True 允许此脚本在被其他Python脚本调用时表现更佳，
    # 但对于独立运行不是必需的。
    rospy.init_node('ftg_starter_node', anonymous=True)

    # 2. 创建发布者。
    #    - 话题名: /ftg/enable
    #    - 消息类型: std_msgs.msg.Bool
    #    - queue_size=1: 对于单次发布，1就足够了。
    #    - latch=True: 这是实现高时效性的关键！
    #      它会保存最后发布的消息，并将其发送给任何之后连接的订阅者。
    pub = rospy.Publisher('/ftg/enable', Bool, queue_size=1, latch=True)

    # 3. 创建要发布的消息
    start_msg = Bool(data=True)

    # 4. 立即发布消息。
    #    我们不需要 rospy.sleep() 来等待连接，因为 latch=True 会处理好。
    rospy.loginfo("正在向 /ftg/enable 发送 'true' (latch=True)...")
    pub.publish(start_msg)
    rospy.loginfo("...消息已发布并锁存。脚本将退出。")

    # (可选) 短暂休眠以确保消息有足够的时间在网络上传播，
    # 尽管对于latch发布来说，这通常不是必需的，但这是一个好的实践。
    rospy.sleep(0.02)


if __name__ == '__main__':
    try:
        send_start_signal()
    except rospy.ROSInterruptException:
        # 如果在执行过程中按下了 Ctrl+C，则静默退出。
        pass