#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

def publish_heartbeat():
    rospy.init_node("heartbeat_publisher")  # Initialisiere Node

    # Topicnamen aus Parameter holen, Standardwert ist "/heartbeat_topic"
    topic_list = rospy.get_param("~topic_names", "/heartbeat_topic")

    # Falls ein einzelner String angegeben wurde, in eine Liste umwandeln
    if not isinstance(topic_list, list):
        topic_list = [topic_list]

    # Publisher für alle Topics erstellen
    publishers = [rospy.Publisher(topic, PoseStamped, queue_size=10) for topic in topic_list]
    rate = rospy.Rate(1)  # 1 Hz Frequenz

    seq = 0  # Sequenznummer starten

    while not rospy.is_shutdown():
        heartbeat_msg = PoseStamped()
        heartbeat_msg.header.seq = seq  # Setze Sequenznummer
        heartbeat_msg.header.stamp = rospy.Time.now()  # ROS-Zeit

        for topic, pub in zip(topic_list, publishers):
            rospy.loginfo(f"Publishing Heartbeat on {topic}: seq={seq}, stamp={heartbeat_msg.header.stamp.to_sec()}")
            pub.publish(heartbeat_msg)  # Nachricht publizieren

        seq += 1  # Sequenznummer erhöhen
        rate.sleep()  # Warte bis zum nächsten Zyklus

if __name__ == "__main__":
    try:
        publish_heartbeat()
    except rospy.ROSInterruptException:
        pass
