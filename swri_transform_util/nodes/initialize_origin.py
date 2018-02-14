#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (C) 2013-2017, Southwest Research Institute® (SwRI®)
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
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

import os
#import rospy
#import tf2
import time
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from geometry_msgs.msg import PoseStamped
from gps_common_msgs.msg import GPSFix
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

# Global variables
_origin = None
_gps_fix = None

def make_origin_msg(frame_id, latitude, longitude, altitude):
    origin = PoseStamped()
    origin.header.frame_id = frame_id
    origin.pose.position.y = latitude
    origin.pose.position.x = longitude
    origin.pose.position.z = altitude
    # Heading is always 0
    origin.pose.orientation.x = 0.0
    origin.pose.orientation.y = 0.0
    origin.pose.orientation.z = 0.0
    origin.pose.orientation.w = 1.0
    return origin

def parse_origin(local_xy_origin):
    global _gps_fix
    #
    # local_xy_origins = rospy.get_param('~local_xy_origins', [])
    #
    # for origin in local_xy_origins:
    #     if origin["name"] == local_xy_origin:
    #
    #         _gps_fix = GPSFix()
    #         _gps_fix.header.frame_id = _local_xy_frame
    #         _gps_fix.status.header.frame_id = local_xy_origin
    #         _gps_fix.latitude = origin["latitude"]
    #         _gps_fix.longitude = origin["longitude"]
    #         _gps_fix.altitude = origin["altitude"]
    #         _gps_fix.track = 90
    #
    #         _origin_pub.publish(_gps_fix)

def make_origin_from_list(frame_id, origin_name, origin_list):
    print("")
    # if len(origin_list) == 0:
    #     rospy.logwarn("No origins specified--defaulting to auto")
    #     return None
    # if len(origin_list) == 1:
    #     origin = origin_list[0]
    #     return make_origin_msg(frame_id,
    #                            origin["latitude"],
    #                            origin["longitude"],
    #                            origin["altitude"])
    # for origin in origin_list:
    #     if origin["name"] == origin_name:
    #         return make_origin_msg(frame_id,
    #                                origin["latitude"],
    #                                origin["longitude"],
    #                                origin["altitude"])
    # rospy.logerror('Origin "{}" is not in the origin list. Available origins are {}.'.format(origin_name, ",".join(['"{}"'.format(x.name) for x in origin_list])))
    # rospy.logerror("No origin found--defaulting to auto")
    # return None

_frame_id = ""
def navsatfix_callback(data):#, (frame_id, pub, sub)):
    if data.status.status == -1:
        # This fix is invalid, ignore it and wait until we get a valid one
        _node.get_logger().info("Got invalid fix.  Waiting for a valid one...")
        return
    global _origin
    global _sub
    global _pub
    global _frame_id
    global _sub_gps
    #_node.get_logger().info("Got NavSat message. Setting origin and unsubscribing from NavSat.")
    _sub = None#.unregister()
    _sub_gps = None
    if _origin is None:
        _origin = make_origin_msg(_frame_id,
                                  data.latitude,
                                  data.longitude,
                                  data.altitude)
        _pub.publish(_origin)


def gps_callback(data):
    #_node.get_logger().info("Got gps");
    if data.status.status == -1:
        # This fix is invalid, ignore it and wait until we get a valid one
        _node.get_logger().info("Got invalid fix.  Waiting for a valid one...")
        return

    global _origin
    global _sub
    global _pub
    global _frame_id
    global _sub_gps
    global _node
    #_node.get_logger().info("Got NavSat message. Setting origin and unsubscribing from NavSat.")
    #_sub = None#.unregister()
    #_sub_gps = None
    if _origin is None:
        _node.destroy_subscription(_sub_gps)
        _node.destroy_subscription(_sub)
        _origin = make_origin_msg(_frame_id,
                                  data.latitude,
                                  data.longitude,
                                  data.altitude)
        _pub.publish(_origin)

# def make_diagnostic(origin, static_origin):
#     diagnostic = DiagnosticArray()
#     diagnostic.header.stamp = rospy.Time.now()
#     status = DiagnosticStatus()
#     status.name = "LocalXY Origin"
#     status.hardware_id = "origin_publisher"
#     if origin == None:
#         status.level = DiagnosticStatus.ERROR
#         status.message = "No Origin"
#     else:
#         if not static_origin:
#             status.level = DiagnosticStatus.OK
#             status.message = "Has Origin (auto)"
#         else:
#             status.level = DiagnosticStatus.WARN
#             status.message = "Origin is static (non-auto)"
#
#         frame_id = origin.header.frame_id
#         status.values.append(KeyValue(key="Origin Frame ID", value=frame_id))
#
#         latitude = "%f" % origin.pose.position.y
#         status.values.append(KeyValue(key="Latitude", value=latitude))
#
#         longitude = "%f" % origin.pose.position.x
#         status.values.append(KeyValue(key="Longitude", value=longitude))
#
#         altitude = "%f" % origin.pose.position.z
#         status.values.append(KeyValue(key="Altitude", value=altitude))
#
#     diagnostic.status.append(status)
#     return diagnostic
_node = None
_sub = None
_sub_gps = None
def initialize_origin():
    rclpy.init()
    global _origin
    global _node
    _node = Node("initialize_origin")#rospy.init_node('initialize_origin', anonymous=True)

    ros_distro = "ardent"#os.environ.get('ROS_DISTRO')

    if ros_distro == 'indigo':
        print("")
    else:
        # ROS distros later than Indigo use NavSatFix
        origin_pub = _node.create_publisher(PoseStamped, '/local_xy_origin')#, latch=True, queue_size=2)
        global _pub
        _pub = origin_pub
        #diagnostic_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=2)
        origin_name = "auto"#rospy.get_param('~local_xy_origin', 'auto')
        print('Local XY origin is "' + origin_name + '"')
        origin_frame_id = "/far_field"#rospy.get_param(rospy.search_param('local_xy_frame'), 'map')
        origin_frame_identity = "/far_field__identity"#rospy.get_param('~local_xy_frame_identity', origin_frame_id + "__identity")
        _node.get_logger().info('Local XY frame ID is "' + origin_frame_id + '"')
        global _frame_id
        _frame_id = origin_frame_id
        if len(origin_frame_id):
            tf_broadcaster = _node.create_publisher(TFMessage, "/tf")# tf.TransformBroadcaster()
        else:
            tf_broadcaster = None
    
        if origin_name != "auto":
            origin_list = rospy.get_param('~local_xy_origins', [])
            _origin = make_origin_from_list(origin_frame_id, origin_name, origin_list)
            if _origin is not None:
                origin_pub.publish(_origin)
            else:
                origin_name = "auto"
        if origin_name == "auto":
            global _sub
            global _sub_gps
            _sub = _node.create_subscription(NavSatFix, "/localization/fix", navsatfix_callback)
            _sub_gps = _node.create_subscription(GPSFix, "/localization/gps", gps_callback)
            #sub.impl.add_callback(navsatfix_callback, (origin_frame_id, origin_pub, sub))
            _node.get_logger().info('Subscribed to GPSFix on ' + "/localization/gps")
        while rclpy.ok():#not rospy.is_shutdown():
            #_node.get_logger().info("Looping");
            #if tf_broadcaster:
                # Publish transform involving map (to an anonymous unused
                # frame) so that TransformManager can support /tf<->/wgs84
                # conversions without requiring additional nodes.
                # tf = TFMessage()
                # ts = TransformStamped()
                # ts.header.stamp.sec = 2147000000
                # ts.header.frame_id = origin_frame_id
                # ts.transform.translation.x = 0.0
                # ts.transform.translation.y = 0.0
                # ts.transform.translation.z = 0.0
                # ts.transform.rotation.x = 0.0
                # ts.transform.rotation.y = 0.0
                # ts.transform.rotation.z = 0.0
                # ts.transform.rotation.w = 1.0
                # #ts.header.stamp
                # ts.child_frame_id = origin_frame_identity
                # tf.transforms.append(ts)
                # tf_broadcaster.publish(tf)
                #print("publishing")
                #tf_broadcaster.sendTransform(
                #    (0, 0, 0),
                #    (0, 0, 0, 1),
                #    rospy.Time.now(),
                #    origin_frame_identity, origin_frame_id)
            if _origin != None:
                _pub.publish(_origin)
            rclpy.spin_once(_node, timeout_sec = 2.0)
            #diagnostic_pub.publish(make_diagnostic(_origin, origin_name != "auto"))
            #time.sleep(1.0)

if __name__ == '__main__':
    #try:
    initialize_origin()
    #except rospy.ROSInterruptException: pass
