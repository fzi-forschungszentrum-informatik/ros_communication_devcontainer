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
# \author  Daniel Bogdoll <bogdoll@fzi.de>
# \author  Martin Gontscharow <gontscharow@fzi.de>
# \date    2024-04-03
#
#
# ---------------------------------------------------------------------

import rospy
import time
from std_msgs.msg import String, Int64
from pysnmp.hlapi import *
import re
import argparse


import subprocess

def get_wifi_info_iwconfig():
    for wlan_interfaces in ["wlp3s0", "wlp0s20f3"]:
        try:
            # Run iwconfig to get wireless interface information
            output = subprocess.check_output(
                ["iwconfig", wlan_interfaces], stderr=subprocess.STDOUT
            ).decode()  # get wireless interface name with command "iwconfig"
            break
        except subprocess.CalledProcessError as e:
            rospy.loginfo(e)
            output = None

    if output is None: 
        rospy.logerr("iwconfig did not yield any output for the queried interfaces")
        return None, None, None

    # Find SSID, signal level, and connected access point from the iwconfig output
    ssid = None
    signal_level = None
    access_point = None

    lines = output.split("\n")
    for line in lines:
        if "ESSID" in line:
            ssid = line.split(":")[1].strip().strip('"')
        elif "Signal level" in line:
            signal_level = re.split("[/ ]", line.split("Signal level=")[1])[0]
        elif "Access Point" in line:
            access_point = line.split("Access Point: ")[1].strip()

    return ssid, signal_level, access_point


def get_wifi_info_snmp(
    router_ip,
    community,
    retries=3,
    timeout=1,
):
    # SNMP OIDs for the relevant wireless interface information
    ssid_oid = ".1.3.6.1.4.1.14988.1.1.1.3.1.4"  # SSID OID
    signal_level_oid = ".1.3.6.1.4.1.14988.1.1.1.2.1.3"  # Signal Level OID (most important one)
    access_point_oid = ".1.3.6.1.4.1.14988.1.1.1.2.1.1"  # Access Point OID
    # signalfloor_oid = ".1.3.6.1.4.1.14988.1.1.1.3.1.9"  # Noisefloor (only for computation)
    # signaltonoise_oid = ".1.3.6.1.4.1.14988.1.1.1.2.1.12"

    for _ in range(retries):
        try:
            # Create an SNMP GETNEXT operation for each OID
            oid_bindings = [
                ObjectType(ObjectIdentity(ssid_oid)),
                ObjectType(ObjectIdentity(signal_level_oid)),
                ObjectType(ObjectIdentity(access_point_oid)),
            ]

            error_indication, error_status, error_index, var_binds = next(
                nextCmd(
                    SnmpEngine(),
                    CommunityData(community, mpModel=1),  # Use SNMPv2c
                    UdpTransportTarget((router_ip, 161)),
                    ContextData(),
                    *oid_bindings,
                    lexicographicMode=False,
                    timeout=timeout,
                    maxRows=1,
                )
            )

            if error_indication:
                rospy.logerr("SNMP Error: " + str(error_indication))
                return None, None, None

            if error_status:
                rospy.logerr("SNMP Error: " + str(error_status.prettyPrint()))
                return None, None, None

            # Extract values from the SNMP response
            var_binds = [x[1] for x in var_binds]
            ssid = var_binds[0].prettyPrint()
            signal_level = var_binds[1].prettyPrint()
            access_point = var_binds[2].prettyPrint()

            return ssid, signal_level, access_point

        except TimeoutError as e:
            rospy.logerr("Timeout occurred: " + str(e))
        except Exception as e:
            rospy.logerr("Error occurred: " + str(e))

        # Sleep briefly before retrying
        time.sleep(1)

    return None, None, None


def logger(iwconfig_instead_of_snmp, snmp_router_ip, snmp_community):
    # Publish to ROS
    pub_wifi_ssid = rospy.Publisher("wifi_ssid", String, queue_size=1)
    pub_wifi_ap = rospy.Publisher("wifi_ap", String, queue_size=1)
    pub_wifi_sl = rospy.Publisher("wifi_sl", Int64, queue_size=1)
    rospy.init_node("logger", anonymous=True)
    rate = rospy.Rate(2)  # 2Hz
    while not rospy.is_shutdown():
        if iwconfig_instead_of_snmp:
            ssid, signal_level, access_point = get_wifi_info_iwconfig()
        else:
            ssid, signal_level, access_point = get_wifi_info_snmp(snmp_router_ip, snmp_community)

        # ssid
        pub_wifi_ssid.publish(ssid)
        rospy.loginfo("ssid: " + str(ssid or ''))

        # accesss point
        pub_wifi_ap.publish(access_point)
        rospy.loginfo("access_point: " + str(access_point or ''))

        # signal_level
        # good level -30, bad level -80, no value 0
        if (
            signal_level is None
            or signal_level == "No Such Object currently exists at this OID"
            or signal_level == "No more variables left in this MIB View"
            or signal_level == ""
        ):  signal_level = 0
        # signal_level = signal_level[: len(signal_level) - 6] #only or iwconfig
        signal_level = int(signal_level)
        pub_wifi_sl.publish(signal_level)
        rospy.loginfo("signal_level: " + str(signal_level))
        
        rospy.loginfo("----------------")
        rate.sleep()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="WiFi Logger Script")
    parser.add_argument(
        "--iwconfig-instead-of-snmp",
        type=str,
        default="false",
        help="Use iwconfig instead of SNMP to get WiFi info",
    )
    parser.add_argument(
        "--snmp-router-ip",
        type=str,
        default="192.168.1.254",
        help="Router IP address for SNMP",
    )
    parser.add_argument(
        "--snmp-community",
        type=str,
        default="public",
        help="SNMP community string",
    )
    
    args = parser.parse_args()

    iwconfig_instead_of_snmp = args.iwconfig_instead_of_snmp.lower() == 'true'

    try:
        logger(iwconfig_instead_of_snmp, args.snmp_router_ip, args.snmp_community)
    except rospy.ROSInterruptException:
        rospy.logerr("Exception: " + rospy.ROSInterruptException)
        pass
