#!/usr/bin/env python

# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
# 
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
# 
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.                                                
# -- END LICENSE BLOCK ------------------------------------------------
#
# ---------------------------------------------------------------------
# !\file
#
# \author  Martin Gontscharow <gontscharow@fzi.de>
# \date    2024-04-03
#
#
# ---------------------------------------------------------------------

import rospy
from std_msgs.msg import Float64
import psutil
import time

class TrafficMonitor:
    def __init__(self):
        rospy.init_node('traffic_monitor', anonymous=True)
        self.upload_rate_pub = rospy.Publisher('traffic_monitor/upload_rate_mbps', Float64, queue_size=10)
        self.download_rate_pub = rospy.Publisher('traffic_monitor/download_rate_mbps', Float64, queue_size=10)
        self.total_upload_pub = rospy.Publisher('traffic_monitor/total_upload_mb', Float64, queue_size=10)
        self.total_download_pub = rospy.Publisher('traffic_monitor/total_download_mb', Float64, queue_size=10)
        self.rate = rospy.Rate(1)  # 1 Hz
        self.last_upload = psutil.net_io_counters().bytes_sent
        self.last_download = psutil.net_io_counters().bytes_recv
        self.total_upload = 0
        self.total_download = 0
        self.last_time = time.time()

    def run(self):
        while not rospy.is_shutdown():
            current_upload = psutil.net_io_counters().bytes_sent
            current_download = psutil.net_io_counters().bytes_recv
            current_time = time.time()
            time_interval = current_time - self.last_time
            
            if time_interval > 0:
                upload_rate = (current_upload - self.last_upload) * 8 / time_interval / 10**6  # Convert to Mbps
                download_rate = (current_download - self.last_download) * 8 / time_interval / 10**6  # Convert to Mbps
                
                self.upload_rate_pub.publish(upload_rate)
                self.download_rate_pub.publish(download_rate)
                
                self.total_upload += (current_upload - self.last_upload) / 10**6  # Convert to MB
                self.total_download += (current_download - self.last_download) / 10**6  # Convert to MB
                self.total_upload_pub.publish(self.total_upload)
                self.total_download_pub.publish(self.total_download)
                
                rospy.loginfo("Upload Traffic Rate: %.2f Mbps | Download Traffic Rate: %.2f Mbps",
                              upload_rate, download_rate)
                rospy.loginfo("Total Uploaded: %.2f MB | Total Downloaded: %.2f MB",
                              self.total_upload, self.total_download)
            
            self.last_upload = current_upload
            self.last_download = current_download
            self.last_time = current_time
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = TrafficMonitor()
        publisher.run()
    except rospy.ROSInterruptException:
        pass

