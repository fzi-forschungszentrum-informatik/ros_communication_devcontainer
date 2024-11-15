#!/usr/bin/env python3

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
# \date    2024-11-13
#
#
# ---------------------------------------------------------------------

import os
import rospy
import subprocess
import sys

ws_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))))
sys.path.append(ws_path)

from session.content.get_data_dict_entries import main as get_data_dict_entries

def start_fkie_master_discovery(rpc_port, discover_host_keys):

    command = [
        'rosrun',
        'fkie_master_discovery',
        'master_discovery',
        "_hide_services:=['/*']",
    ]

    if rpc_port:
        command.append(f'_rpc_port:={rpc_port}')

    if discover_host_keys:
        robot_host_list = get_data_dict_entries(discover_host_keys)
        robot_hosts = ','.join(robot_host_list)  # Convert list to a comma-separated string
        command.append(f'_robot_hosts:=[{robot_hosts}]')

    # Print the command
    print("Executing command:", ' '.join(command))
    
    # Start the synchronization process
    subprocess.Popen(command)

if __name__ == "__main__":
    rospy.init_node('start_fkie_master_discovery', anonymous=True)

    # Retrieve parameters from ROS parameter server
    rpc_port = rospy.get_param('~rpc_port', None)
    discover_host_keys = rospy.get_param('~discover_host_keys', None)
    print("discover_host_keys:", str(discover_host_keys))

    start_fkie_master_discovery(rpc_port, discover_host_keys)

    rospy.spin()  # Keep the node running
