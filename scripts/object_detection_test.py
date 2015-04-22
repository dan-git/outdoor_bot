#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Jenny Barry

'''
Module containing the TablewareDetection class and other functions useful for detecting objects on tables.
'''

__docformat__ = "restructuredtext en"

import roslib
roslib.load_manifest('outdoor_bot')

import rospy
from outdoor_bot.object_detection import ObjectDetector
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud

import copy


def marker_at(pose_stamped, ns='', mid=0, mtype=Marker.SPHERE, sx=0.05, sy=0.05, sz=0.05, r=0.0, g=0.0, b=1.0,
              a=0.8):
    '''                                                                                                     
    Returns a single marker at a pose.                                                                      
                                                                                                            
    See the visualization_msgs.msg.Marker documentation for more details on any of these fields.            
                                                                                                            
    **Args:**                                                                                               
                                                                                                            
        **pose_stamped (geometry_msgs.msg.PoseStamped):** Pose for marker                                   
                                                                                                            
        *ns (string):* namespace                                                                            
                                                                                                            
        *mid (int):* ID                                                                                     
                                                                                                            
        *mtype (int):* Shape type                                                                           
                                                                                                            
        *sx (double):* Scale in x                                                                           
                                                                                                            
        *sy (double):* Scale in y                                                                           
                                                                                                            
        *sz (double):* scale in z                                                                           
                                                                                                            
        *r (double):* Red (scale 0 to
         *g (double):* Green (scale 0 to 1)                                                                  
                                                                                                            
        *b (double):* Blue (scale 0 to 1)                                                                   
                                                                                                            
        *a (double):* Alpha (scale 0 to 1)                                                                  
                                                                                                            
    **Returns:**                                                                                            
        visualization_msgs.msg.Marker at pose_stamped                                                       
    '''
    
    marker = Marker()
    marker.header = copy.deepcopy(pose_stamped.header)
    marker.ns = ns
    marker.id = mid
    marker.type = mtype
    marker.action = marker.ADD
    marker.pose = copy.deepcopy(pose_stamped.pose)
    marker.scale.x = sx
    marker.scale.y = sy
    marker.scale.z = sz
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = a
    return marker

def main():
    viz_pub = rospy.Publisher('/cluster_boxes', Marker)
    cloud_pub = rospy.Publisher('/cluster_points', PointCloud)
    od = ObjectDetector()
    objs = od.detect_objects()
    rospy.loginfo('Found %d clusters', len(objs.pickable_objects))
    rospy.loginfo('Table pose is:\n' + str(objs.table.pose))
    table = marker_at(objs.table.pose, mtype = Marker.TRIANGLE_LIST, ns = 'table', sx = 1.0,
                      sy = 1.0, sz = 1.0, r = 0.0, g = 1.0, b = 0.0, a = 0.4)
    for tri in objs.table.convex_hull.triangles:
        for ind in tri.vertex_indices:
            table.points.append(objs.table.convex_hull.vertices[ind])
    viz_pub.publish(table)
    
    for po in objs.pickable_objects:
        marker = marker_at(
            po.bounding_box.pose, ns = po.graspable_object.collision_name, mtype = Marker.CUBE,
            sx = po.bounding_box.box_dims.x, sy = po.bounding_box.box_dims.y, sz = po.bounding_box.box_dims.z, a = 0.5)
        viz_pub.publish(marker)
        rospy.loginfo('Cluster frame is ' + po.graspable_object.reference_frame_id)

rospy.init_node('detection_tester_node')
main()
