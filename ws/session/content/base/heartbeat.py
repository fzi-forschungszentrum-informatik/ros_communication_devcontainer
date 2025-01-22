#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

def publish_heartbeat():
    rospy.init_node("heartbeat_publisher")  # Initialisiere Node

    # Topicname aus Parameter holen, Standardwert ist "/heartbeat_topic"
    topic_name = rospy.get_param("~topic_name", "/heartbeat_topic")
    pub = rospy.Publisher(topic_name, PoseStamped, queue_size=10)  # Publisher erstellen
    rate = rospy.Rate(1)  # 1 Hz Frequenz

    seq = 0  # Sequenznummer starten

    while not rospy.is_shutdown():
        heartbeat_msg = PoseStamped()
        heartbeat_msg.header.seq = seq  # Setze Sequenznummer
        heartbeat_msg.header.stamp = rospy.Time.now()  # ROS-Zeit

        rospy.loginfo(f"Publishing Heartbeat on {topic_name}: seq={seq}, stamp={heartbeat_msg.header.stamp.to_sec()}")
        pub.publish(heartbeat_msg)  # Nachricht publizieren

        seq += 1  # Sequenznummer erhöhen
        rate.sleep()  # Warte bis zum nächsten Zyklus

if __name__ == "__main__":
    try:
        publish_heartbeat()
    except rospy.ROSInterruptException:
        pass
