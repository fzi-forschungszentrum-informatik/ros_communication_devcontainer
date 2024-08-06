#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import psutil
import time

class TrafficMonitor:
    def __init__(self, interface='eth0'):
        rospy.init_node('traffic_monitor', anonymous=True)
        self.interface = interface
        self.upload_rate_pub = rospy.Publisher('traffic_monitor/upload_rate_kbps', Float64, queue_size=1)
        self.download_rate_pub = rospy.Publisher('traffic_monitor/download_rate_kbps', Float64, queue_size=1)
        self.total_upload_pub = rospy.Publisher('traffic_monitor/total_upload_mb', Float64, queue_size=1)
        self.total_download_pub = rospy.Publisher('traffic_monitor/total_download_mb', Float64, queue_size=1)
        self.rate = rospy.Rate(1)  # 1 Hz
        self.last_stats = psutil.net_io_counters(pernic=True)[self.interface]
        self.last_time = time.time()

    def run(self):
        while not rospy.is_shutdown():
            current_stats = psutil.net_io_counters(pernic=True)[self.interface]
            current_time = time.time()
            time_interval = current_time - self.last_time
            
            if time_interval > 0:
                upload_rate = (current_stats.bytes_sent - self.last_stats.bytes_sent) * 8 / time_interval / 1000  # Convert to Kbit/s
                download_rate = (current_stats.bytes_recv - self.last_stats.bytes_recv) * 8 / time_interval / 1000  # Convert to Kbit/s
                
                self.upload_rate_pub.publish(upload_rate)
                self.download_rate_pub.publish(download_rate)
                
                self.total_upload_pub.publish(current_stats.bytes_sent / 10**6)  # Convert to MB
                self.total_download_pub.publish(current_stats.bytes_recv / 10**6)  # Convert to MB
                
                rospy.loginfo("Up: %8.0f Kbit/s | Down: %8.0f Kbit/s | Tot. Up: %9.2f MB | Tot. Down: %9.2f MB",
                              upload_rate, download_rate, current_stats.bytes_sent / 10**6, current_stats.bytes_recv / 10**6)
            
            self.last_stats = current_stats
            self.last_time = current_time
            
            self.rate.sleep()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Monitor network traffic for a specific network interface.")
    parser.add_argument('-i', '--interface', type=str, default='eth0', help='Network interface to monitor')
    args = parser.parse_args()
    
    try:
        publisher = TrafficMonitor(interface=args.interface)
        publisher.run()
    except rospy.ROSInterruptException:
        pass
