#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import String  # 假设你要等待的消息类型是 String
# from your_package.msg import YourCustomMessageType # 如果是自定义消息类型

def main_operations_after_message(received_message):
    """
    在接收到第一条消息后执行的主要逻辑。
    """
    rospy.loginfo("Received first message: %s", received_message.data)
    rospy.loginfo("Now proceeding with the rest of the operations...")

    # 在这里执行你的节点的主要逻辑
    # 例如：开始发布消息、调用服务、进行计算等
    rate = rospy.Rate(1) # 1 Hz
    count = 0
    while not rospy.is_shutdown() and count < 5:
        rospy.loginfo("Doing work... item %d", count)
        # my_publisher.publish(some_data)
        count += 1
        rate.sleep()
    rospy.loginfo("Main operations complete.")


if __name__ == '__main__':
    try:
        rospy.init_node('my_waiting_node', anonymous=True)

        # --- 配置等待的参数 ---
        topic_to_wait_for = "/other_node/status_topic"  # 替换成你实际要等待的话题名称
        message_type = String                           # 替换成该话题实际的消息类型
        timeout_duration = 30.0                         # 等待的超时时间（秒）

        rospy.loginfo("Waiting for the first message on topic '%s' (timeout: %.1f s)...", 
                      topic_to_wait_for, timeout_duration)

        try:
            # 等待消息，会阻塞直到收到消息或超时
            # message 变量将包含接收到的第一条消息
            message = rospy.wait_for_message(topic_to_wait_for, message_type, timeout=timeout_duration)
            
            # 如果成功接收到消息，则执行后续操作
            main_operations_after_message(message)

        except rospy.ROSException as e:
            # 超时或其他ROS相关的错误 (例如话题不存在，但这种情况 wait_for_message 通常会一直等待)
            rospy.logerr("Timeout or ROS error while waiting for message on %s: %s", topic_to_wait_for, e)
            rospy.logerr("Node will not proceed with main logic.")
            # 根据需要，你可以在这里决定是退出节点还是尝试其他操作

        except rospy.ROSInterruptException:
            rospy.loginfo("Node shutdown requested while waiting for message.")
        
        # 如果 main_operations_after_message 内部没有 rospy.spin() 或持续循环，
        # 并且你希望节点在执行完操作后继续运行（例如，作为一个服务提供者），
        # 你可能需要在这里添加 rospy.spin()。
        # 但如果 main_operations_after_message 执行完就代表节点任务完成，则不需要。
        # rospy.spin() # 如果节点还需要响应回调等，则取消注释

        rospy.loginfo("My waiting node has finished its main task or timed out.")

    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutdown.")
    except Exception as e:
        rospy.logerr("An unexpected error occurred: %s", e)