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
import speedtest
import time

class SpeedTest:
    def __init__(self):
        rospy.init_node('speed_test', anonymous=True)
        self.ping_pub = rospy.Publisher('speed_test/ping_latency_ms', Float64, queue_size=10)
        self.download_pub = rospy.Publisher('speed_test/download_mbps', Float64, queue_size=10)
        self.upload_pub = rospy.Publisher('speed_test/upload_mbps', Float64, queue_size=10)
        self.rate = rospy.Rate(1)

    def run(self):
        next_speedtest_time = time.time()
        
        while not rospy.is_shutdown():
            current_time = time.time()
            
            if current_time >= next_speedtest_time:
                self.execute_speedtest()
                next_speedtest_time = current_time + 120  # Schedule the next speed test 120 seconds later
            else:
                self.publish_status(current_time, next_speedtest_time)
            
            self.rate.sleep()
    
    def execute_speedtest(self):
        rospy.loginfo("Executing speed test...")
        
        st = speedtest.Speedtest()
        st.get_best_server()  # Find the best server for speed test
        
        start_time = time.time()  # Start the timer
        ping_latency = st.results.ping
        download_speed = st.download() / 10**6  # Convert to Mbps
        upload_speed = st.upload() / 10**6  # Convert to Mbps
        end_time = time.time()  # End the timer
        
        elapsed_time = end_time - start_time
        self.publish_speedtest_results(ping_latency, download_speed, upload_speed)
        self.log_speedtest_results(ping_latency, download_speed, upload_speed, elapsed_time)
    
    def publish_speedtest_results(self, ping_latency, download_speed, upload_speed):
        ping_msg = Float64(ping_latency)
        download_msg = Float64(download_speed)
        upload_msg = Float64(upload_speed)
        
        self.ping_pub.publish(ping_msg)
        self.download_pub.publish(download_msg)
        self.upload_pub.publish(upload_msg)

    def publish_status(self, current_time, next_speedtest_time):
        remaining_time = next_speedtest_time - current_time
        rospy.loginfo("Sleeping... Time until next speed test: %.2f seconds", remaining_time)
    
    def log_speedtest_results(self, ping_latency, download_speed, upload_speed, elapsed_time):
        rospy.loginfo("Ping Latency: %.2f ms | Download Speed: %.2f Mbps | Upload Speed: %.2f Mbps",
                      ping_latency, download_speed, upload_speed)
        
        rospy.loginfo("Speed test completed in %.2f seconds.", elapsed_time)

if __name__ == '__main__':
    try:
        publisher = SpeedTest()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
