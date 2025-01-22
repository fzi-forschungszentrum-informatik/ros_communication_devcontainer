#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header as Heartbeat

def publish_heartbeat():
    rospy.init_node("heartbeat_publisher", anonymous=True)  # Initialisiere Node

    # Topicname aus Parameter holen, Standardwert ist "/heartbeat_topic"
    topic_name = rospy.get_param("~topic_name", "/heartbeat_topic")
    pub = rospy.Publisher(topic_name, Heartbeat, queue_size=10)  # Publisher erstellen
    rate = rospy.Rate(1)  # 1 Hz Frequenz

    seq = 0  # Sequenznummer starten

    while not rospy.is_shutdown():
        heartbeat_msg = Heartbeat()
        heartbeat_msg.seq = seq  # Setze Sequenznummer
        heartbeat_msg.stamp = rospy.Time.now()  # ROS-Zeit

        rospy.loginfo(f"Publishing Heartbeat on {topic_name}: seq={seq}, stamp={heartbeat_msg.stamp.to_sec()}, frame_id={heartbeat_msg.frame_id}")
        pub.publish(heartbeat_msg)  # Nachricht publizieren

        seq += 1  # Sequenznummer erhöhen
        rate.sleep()  # Warte bis zum nächsten Zyklus

if __name__ == "__main__":
    try:
        publish_heartbeat()
    except rospy.ROSInterruptException:
        pass
