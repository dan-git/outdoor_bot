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
Module containing the ObjectDetector class and other functions useful for detecting objects on tables.
'''

__docformat__ = "restructuredtext en"

import roslib
roslib.load_manifest('myBot')

import rospy

from tabletop_object_detector.srv import TabletopSegmentation, TabletopSegmentationRequest, TabletopSegmentationResponse
from manipulation_msgs.msg import GraspableObject
from object_manipulation_msgs.srv import FindClusterBoundingBox, FindClusterBoundingBoxResponse
from geometry_msgs.msg import Point, Pose
import myBot.transform_listener as tl
import myBot.geometry_tools as gt

import copy

# Names of pertinent services
FIND_CLUSTER_BOUNDING_BOX_SERVICE = '/find_cluster_bounding_box'
'''
Service for finding bounding boxes of point clouds.
'''

SEGMENTATION_SERVICE = '/tabletop_segmentation'
'''
Service for segmenting clusters on a flat surface
'''

DEFAULT_VERTICAL_THRESHOLD = 0.5
'''
Default value for the dot product with the z axis above which a table is considered vertical.
'''


# The following defaults are all defaults for initializing the detector class.  In general,
# you just want to set these when you initialize the class; there is no reason to change them here.

def find_height_above_table(pose_stamped_in, table_pose_stamped):
    '''
    Utility function that, given the pose (presumably of an object) and the pose of a table
    will return the height of that object above the table.

    **Args:**
       
        **pose_stamped_in (geometry_msgs.msg.PoseStamped):** The pose above the table

        **table_pose_stamped (geometry_msgs.msg.PoseStamped):** The pose of the table

    **Returns:**
        The height of the pose above the table.

    **Raises:**
        All exceptions a TransformListener can raise.
    '''
    pose_stamped = tl.transform_pose_stamped(table_pose_stamped.header.frame_id, pose_stamped_in) 
    #take this coordinate and transform it into the table's
    tp = gt.inverse_transform_point(pose_stamped.pose.position, table_pose_stamped.pose)
    rospy.logdebug('Height above table is '+str(tp.z))
    return tp.z

class DetectionError(Exception):
    '''
    Exception raised if anything goes wrong in detection.
    '''

    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        return self.msg

class PickableObject(object):
    '''
    Stores the bounding box with the graspable object.

    **Attributes:**
    
    **graspable_object (manipulation_msgs.msg.GraspableObject):** A graspable object, which is a wrapper around a
    cluster or a mesh and can be fed to grasp planners.

    **bounding_box (object_manipulation_msgs.srv.FindClusterBoundingBoxResponse):** The bounding box for the object.
    '''

    def __init__(self, graspable_object, bounding_box):
        self.graspable_object = graspable_object
        self.bounding_box = bounding_box

class ObjectDetectionResult(object):
    '''
    A class for storing the result of an object detection.  This is a Table and a list of pickable objects.

    **Attributes:**

    **table (tabletop_object_detector.msg.Table):** The table used in detecting these objects.

    **pickable_objects ([PickableObject]):** The list of pickable objects detected.
    '''

    def __init__(self, table, pickable_objects):
        self.table = table
        self.pickable_objects = pickable_objects

class ObjectDetector(object):
    '''
    Calls tabletop segmentation.  The usual use of this class is to instantiate an instance and then use detect_objects
    to find objects on a table in front of the robot:
    detector = ObjectDetector()
    res = detector.detect_objects([optional arguments])
    
    This class is a wrapper around the tabletop_object_detector, which assumes that the robot is standing in 
    front of a table on which objects are positioned at least 3cm apart.  It first locates the principle 
    horizontal plane in the image and then segments clusters above that plane based on Euclidean distance.  
    It returns those clusters in the form of GraspableObjects.
    
    The attributes of this class can all be changed by passing in different defaults although the default settings 
    should work well in most cases.  However, most of the parameters pertinent to detection must be changed 
    service side.  Look at the tabletop_object_detector package for that.
    
    Many of the parameters in this class are for avoiding working with "vertical tables".
    The detector finds the "principal plane" in the image.  Unfortunately, this plane may not be horizontal.  
    In general, it is best if any detection that does not return a table that is approximately perpendicular to 
    the z axis in the world is considered as failed.
    
    **Attributes:**
        
        **vertical_threshold (double):** The dot product of the normal to the table plane and the z axis must be greater
        than this for the table to be considered horizontal.        
        
        **error_map (dict):** A mapping from error codes to strings with which to create DetectionError exceptions.
    '''

    def __init__(self, vertical_threshold = DEFAULT_VERTICAL_THRESHOLD, error_map = None):
        '''
        Constructor for ObjectDetector.
        
        **Args:**
        
            *vertical_threshold (double):* The dot product of the normal to the table plane and the z axis must be 
            greater than this for the table to be considered horizontal.
        
            *error_map (dict):* A dictionary mapping error codes to strings.  If left at None, will default to:
                TabletopSegmentationResponse.NO_CLOUD_RECEIVED : 'No cloud received'
                TabletopSegmentationResponse.NO_TABLE :          'Unable to detect a flat plane in the image.'
                TabletopSegmentationResponse.OTHER_ERROR :       'Unknown error during detection.  Please check the'
                                                                 ' output of the segment service.'
        '''
        self.vertical_threshold = vertical_threshold
        self.error_map = error_map
        if self.error_map == None:
            self.error_map = {
                TabletopSegmentationResponse.NO_CLOUD_RECEIVED : 'No cloud received',
                TabletopSegmentationResponse.NO_TABLE : 'Unable to detect a flat plane in the image.',
                TabletopSegmentationResponse.OTHER_ERROR : 'Unknown error during detection.  ' +
                'Please check the output of the segment service.'
                }
        self._bounding_box_srv = rospy.ServiceProxy(FIND_CLUSTER_BOUNDING_BOX_SERVICE, FindClusterBoundingBox)
        rospy.loginfo('Waiting for bounding box service to fill pickup goal.')
        self._bounding_box_srv.wait_for_service()
        rospy.loginfo('Waiting for object segmentation service')
        self.segmentation_service = rospy.ServiceProxy(SEGMENTATION_SERVICE, TabletopSegmentation)
        self._object_id = 0
        rospy.loginfo('Ready to do object detections!')
        
    def detect_objects(self, table = None, allow_vertical_tables = True):
        '''
        This is the function you should call to get detections of objects in the scene.  This will return an
        ObjectDetectionResult if the detection is successful or raise a DetectionError if it is not.

        **Args:**
        
            *table (tabletop_object_detector.msg.Table)*: If not left at None, will use this table rather than detecting
            a new one.

            *allow_vertical_tables (boolean):* If True will do segmentation and return even if a vertical table is 
            found.

        **Returns:**
            A ObjectDetectionResult.
            
         **Raises:**
         
             **exceptions.DetectionError:** if a table is not found or only vertical tables can be found and
             allow_vertical_tables is false.
         '''

        res = self.get_segmentation_results(table = table)
        if res.result != res.SUCCESS:
            raise DetectionError(self.error_map[res.result])
        if not allow_vertical_tables:
            # check that the table is horizontal
            vert = Point()
            vert.z = 1.0
            oripose = Pose()
            oripose.orientation = res.table.pose.pose.orientation
            table_vertical = gt.transform_point(vert, oripose)
            if abs(table_vertical.z) < self.vertical_threshold:
                raise DetectionError('Table vertical is [%f, %f, %f].  This is not a horizontal table' %
                                     (table_vertical.x, table_vertical.y, table_vertical.z))
        pos = []
        # Compute bounding boxes
        for (i, cluster) in enumerate(res.clusters):
            go = GraspableObject()
            go.reference_frame_id = cluster.header.frame_id
            go.cluster = cluster
            go.collision_name = 'object_' + str(self._object_id)
            self._object_id += 1
            po = PickableObject(go, None)
            # Compute the bounding box
            try:
                po.bounding_box = self._bounding_box_srv(cluster)
            except rospy.ServiceException, e:
                rospy.logwarn('Unable to call cluster bounding box service.  Exception was ' + str(e) +
                             '.  Bounding box for ' + go.collision_name + ' will be None.')
            if po.bounding_box.error_code != FindClusterBoundingBoxResponse.SUCCESS:
                rospy.logwarn('Cluster bounding box service returned error ' + str(po.bounding_box.error_code) + 
                              'cluster .  Bounding box for ' + go.collision_name + ' will be None.')
                po.bounding_box = None
            pos.append(po)
        if table != None:
            res.table = table
        return ObjectDetectionResult(res.table, pos)

    def get_segmentation_results(self, table = None):
        '''
        Does segmentation and returns the raw results without the post-processing of detect_objects.

        **Args:**
        
            *table (tabletop_object_detector/Table):* If not None, will use this table rather than detecting a new one.

        **Returns:**
            A TabletopSegmentationResponse
        '''

        req = TabletopSegmentationRequest()
        if table != None:
            req.table = table
        return self.segmentation_service(req)
