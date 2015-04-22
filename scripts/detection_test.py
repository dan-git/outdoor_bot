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
roslib.load_manifest('myBot')

import rospy

from tabletop_object_detector.srv import TabletopSegmentation, TabletopSegmentationRequest
from visualization_msgs.msg import Marker
from object_manipulation_msgs.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud

import copy


SEGMENTATION_SERVICE = '/tabletop_segmentation'

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
    segmentation_service = rospy.ServiceProxy(SEGMENTATION_SERVICE, TabletopSegmentation)
    rospy.loginfo('Waiting for object recognition service')
    segmentation_service.wait_for_service()
    rospy.loginfo('Found segmentation service!')
    box_service = rospy.ServiceProxy('/find_cluster_bounding_box', FindClusterBoundingBox)
    rospy.loginfo('Waiting for find cluster bounding box service')
    box_service.wait_for_service()
    rospy.loginfo('Found cluster bounding box service!')
    req = TabletopSegmentationRequest()
    resp = segmentation_service(req)
    rospy.loginfo('Found %d clusters', len(resp.clusters))
    rospy.loginfo('Table pose is:\n' + str(resp.table.pose))
    table = marker_at(resp.table.pose, mtype = Marker.TRIANGLE_LIST, ns = 'table', sx = 1.0,
                      sy = 1.0, sz = 1.0, r = 0.0, g = 1.0, b = 0.0, a = 0.4)
    for tri in resp.table.convex_hull.triangles:
        for ind in tri.vertex_indices:
            table.points.append(resp.table.convex_hull.vertices[ind])
    viz_pub.publish(table)
    
    pc = PointCloud()
    pc.header = resp.table.pose.header

    for (mid, cluster) in enumerate(resp.clusters):
        box_resp = box_service.call(FindClusterBoundingBoxRequest(cluster = cluster))
        marker = marker_at(
            box_resp.pose, ns = str(mid), mid = mid, mtype = Marker.CUBE, sx = box_resp.box_dims.x, sy = box_resp.box_dims.y,
            sz = box_resp.box_dims.z, a = 0.5)
        viz_pub.publish(marker)
        pose = PoseStamped()
        pose.header = cluster.header
        pose.pose.orientation.w = 1.0
        marker = marker_at(pose, ns = 'cluster', mid = mid, mtype = Marker.POINTS, sx = 1.0,
                           sy = 1.0, sz = 1.0, r = 1.0, g = 0.0, b = 0.0, a = 1.0)
        marker.points = cluster.points
        #viz_pub.publish(marker)
        pc.points += cluster.points
        pc.channels += cluster.channels
        rospy.loginfo('Cluster frame is ' + cluster.header.frame_id)

    cloud_pub.publish(pc)


rospy.init_node('detection_tester_node')
main()
