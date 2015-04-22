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
roslib.load_manifest('pr2_tasks')

import rospy
from exceptions import DetectionError

from tabletop_object_detector.srv import TabletopDetection, TabletopSegmentation, TabletopObjectRecognition,\
    TabletopDetectionRequest, TabletopSegmentationRequest, TabletopObjectRecognitionRequest
from arm_navigation_msgs.msg import CollisionObject, Shape
from object_manipulation_msgs.msg import GraspableObject
from object_manipulation_msgs.srv import FindClusterBoundingBox
from pickplace_definitions import PickupGoal
from household_objects_database_msgs.srv import GetModelMesh, GetModelDescription
from pr2_python.head import Head #so we can jiggle the head until we found a horizontal table
from pr2_python.world_interface import WorldInterface
from geometry_msgs.msg import Point, Pose
from mobile_manipulation_msgs.msg import Object, TabletopSnapshot

import pr2_python.transform_listener as tl
import pr2_python.geometry_tools as gt

import copy

#Names of pertinent services
OBJECT_DETECTION_SERVICE = '/object_detection'
'''
Service for detecting objects
'''

FIND_CLUSTER_BOUNDING_BOX_SERVICE = '/find_cluster_bounding_box'
'''
Service for finding bounding boxes of point clouds.
'''

GET_MODEL_MESH_SERVICE = '/objects_database_node/get_model_mesh'
'''
Service for getting a model's mesh from the database.
'''

GET_MODEL_DESCRIPTION_SERVICE = '/objects_database_node/get_model_description'
'''
Service for getting a model's description from the database.
'''

GET_PLANNING_SCENE_SERVICE = '/environment_server/get_planning_scene'
'''
Service for getting the planning scene.
'''

RECOGNITION_SERVICE = '/tabletop_object_recognition'
SEGMENTATION_SERVICE = '/tabletop_segmentation'

#The following defaults are all defaults for initializing the detector class.  In general,
#you just want to set these when you initialize the class; there is no reason to change them here.
DEFAULT_TABLE_THICKNESS = 0.02
'''
Default thickness with which table will be added to the map.
'''

TABLE_RESOLUTION = 0.02
'''
The resolution (in m) of the height of the table.  Table meshes will be added every TABLE_RESOLUTION m to make up the 
height. 
'''

DEFAULT_TABLEWARE_LABELS = ['plate', 'bowl', 'cup', 'utensil', 'knife', 'fork', 'spoon']
'''
Default labels for tableware.
'''

DEFAULT_TABLE_SEARCH_RESOLUTION = 0.1
'''
Default resolution for searching with the head for a horizontal table.
'''

DEFAULT_TABLE_SEARCH_MAX = 1.0
'''
Default maximum search for searching with the head for a horizontal table.
'''

DEFAULT_VERTICAL_THRESHOLD = 0.5
'''
Default value for the dot product with the z axis above which a table is considered vertical.
'''

DEFAULT_OBJECT_PADDING = 0.08
'''
Default padding to give objects when adding them to the collision map.
'''

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


class TablewareDetectionResult:
    '''
    The return from a tableware detection.
    
    A pickup goal is defined in pickplace_definitions.py.  Fields you may want to use after detection are
    'label', which will have a label corresponding to the recognition ('bowl', 'cup', etc) and
    object_pose_stamped, which has the pose of the object.  The pickup goal returned here MUST HAVE THE ARM
    NAME filled in and then can be passed directly to pickup.
    
    Note that tableware detection always detects a table although you can control whether that table will be 
    added to the collision map.  The table variable in this class is that detected table.
    
    **Attributes:**
    
        **pickup_goals (pickplace_definitions.PickupGoal):** Contains all of the detection information.  If you want 
        to pick up a detected object, you must fill in the arm_name field for which arm you want to use and 
        then the pickup_goal can be passed directly to pickup.
        
        **table (tabletop_object_detector.msg.Table):** The detected table.

        **table_name (string):** The collision map ID of the table if it was added to the collision map.
    '''

    def __init__(self, pickup_goals, table, table_name):
        '''
        Constructor for TablewareDetectionResult.

        **Args:**
        
            **pickup_goals (pickplace_definitions.PickupGoal):** Contains all of the detection information.  If you 
            want  to pick up a detected object, you must fill in the arm_name field for which arm you want to use and 
            then the pickup_goal can be passed directly to pickup.
        
            **table (tabletop_object_detector.msg.Table):** The detected table.
            
            **table_name (string):** The collision map ID of the table if it was added to the collision map.
        '''

        self.pickup_goals = pickup_goals
        self.table = table
        self.table_name = table_name

class TablewareDetection:
    '''
    Calls tabletop detection allowing the user a lot of (optional) control over when and how objects are added
    to the collision map.  The usual use of this class is to instantiate an instance and then use detect_objects
    to find objects on a table in front of the robot:
    detector = TablewareDetection()
    res = detector.detect_objects([optional arguments])
    
    This class is a wrapper around the tabletop_object_detector, which assumes that the robot is standing in 
    front of a table on which objects are positioned at least 3cm apart.  It first locates the principle 
    horizontal plane in the image and then segments clusters above that plane based on Euclidean distance.  
    It runs those clusters through the object recognition pipeline and returns them in the form of pickup goals.
    
    The attributes of this class can all be changed by passing in different defaults although the default settings 
    should work well in most cases.  However, most of the parameters pertinent to detection must be changed 
    service side.  Look at the tabletop_object_detector package for that.
    
    Many of the parameters in this class are for working with "vertical tables".
    The detector finds the "principle plane" in the image.  Unfortunately, this plane may not be horizontal.  
    In general, it is best if any detection that does not return a table that is approximately perpendicular to 
    the z axis in the world is considered as failed.  If a point for the head to look at is passed to 
    detect_objects, the robot will move its head trying to find a horizontal table.
    
    **Attributes:**

        **allow_vertical_tables (boolean):** If True will do segmentation and return even if a vertical table is found.
        
        **vertical_threshold (double):** The dot product of the normal to the table plane and the z axis must be greater
        than this for the table to be considered horizontal.
        
        **table_search_resolution (double):** If detect_objects is initially given a point for the head to look at, it 
        will search along the world x axis near that point until it founds a horizontal table.  This is the 
        resolution in meters of that search.  Resolution cannot be smaller than a centimeter.
        
        **table_search_max (double):** The maximum distance in meters from the point the search should go.  It will go 
        this far in both directions before failing.
        
        **table_thickness (double):** If the table is added to the map, it will be added with this thickness in meters.
        This is helpful for placing as it will occlude points that the robot sees below the table.
        
        **tableware_labels ([string]):** A set of database tags that count as "tableware".  If objects have 
        multiple tags, they are considered tableware if anyone of these tags matches.

        **object_padding (double):** If objects are added to the map, this is the padding in centimeters that they will 
        be added with.  Padding is used only for replacing voxels in the collision map.  We recommend setting 
        this high so that during pick there is no collision between the object and the voxels where it used to be.
        
        **get_model_description (rospy.ServiceProxy):** Service for getting the description of a model from the model 
        ID.  See the household_database_msgs documentation for more information.
    '''

    def __init__(self, allow_vertical_tables=False, vertical_threshold=DEFAULT_VERTICAL_THRESHOLD,
                 table_search_resolution=DEFAULT_TABLE_SEARCH_RESOLUTION, 
                 table_search_max=DEFAULT_TABLE_SEARCH_MAX, table_thickness=DEFAULT_TABLE_THICKNESS,
                 tableware_labels=None, object_padding=DEFAULT_OBJECT_PADDING):

        '''
        Constructor for TablewareDetection.
        
        **Args:**

            *allow_vertical_tables (boolean):* If True will do segmentation and return even if a vertical table is 
            found.
        
            *vertical_threshold (double):* The dot product of the normal to the table plane and the z axis must be 
            greater than this for the table to be considered horizontal.
        
            *table_search_resolution (double):* If detect_objects is initially given a point for the head to look at, 
            it will search along the world x axis near that point until it founds a horizontal table.  This is the 
            resolution in meters of that search.  Resolution cannot be smaller than a centimeter.
        
            *table_search_max (double):* The maximum distance in meters from the point the search should go.  It will 
            go this far in both directions before failing.
        
            *table_thickness (double):* If the table is added to the map, it will be added with this thickness in 
            meters.  This is helpful for placing as it will occlude points that the robot sees below the table.
        
            *tableware_labels ([string]):* A set of database tags that count as "tableware".  If objects have 
            multiple tags, they are considered tableware if anyone of these tags matches.

            *object_padding (double):* If objects are added to the map, this is the padding in centimeters that they 
            will be added with.  Padding is used only for replacing voxels in the collision map.  We recommend setting 
            this high so that during pick there is no collision between the object and the voxels where it used to be.
        '''

        self.allow_vertical_tables = allow_vertical_tables
        self.vertical_threshold = vertical_threshold
        self.table_search_resolution = table_search_resolution
        self.table_search_max = table_search_max
        self.table_thickness = table_thickness
        self.tableware_labels = tableware_labels
        if not tableware_labels:
            self.tableware_labels = copy.copy(DEFAULT_TABLEWARE_LABELS)
        self.object_padding = object_padding

        self._wi = WorldInterface()
        self._head = Head()

        self._detect_srv = rospy.ServiceProxy(OBJECT_DETECTION_SERVICE, TabletopDetection)
        rospy.loginfo('Waiting for object detection service')
        self._detect_srv.wait_for_service()
        self._bounding_box_srv = rospy.ServiceProxy(FIND_CLUSTER_BOUNDING_BOX_SERVICE, FindClusterBoundingBox)
        rospy.loginfo('Waiting for bounding box service to fill pickup goal.')
        self._bounding_box_srv.wait_for_service()
        rospy.loginfo('Waiting for object segmentation service')
        self.segmentation_service = rospy.ServiceProxy(SEGMENTATION_SERVICE,TabletopSegmentation)
        rospy.loginfo('Waiting for object recognition service')
        self.recognition_service = rospy.ServiceProxy(RECOGNITION_SERVICE,TabletopObjectRecognition) 

        self._get_model_mesh_srv = rospy.ServiceProxy(GET_MODEL_MESH_SERVICE, GetModelMesh)
        self.get_model_description = rospy.ServiceProxy(GET_MODEL_DESCRIPTION_SERVICE, GetModelDescription)
        try:
            rospy.loginfo('Waiting for get model mesh service')
            self._get_model_mesh_srv.wait_for_service(5.0)   
            rospy.loginfo('Waiting for get model description service')
            self.get_model_description.wait_for_service(5.0)
        except rospy.ROSException:
            rospy.logwarn('Did not find database services.  Likely will not be able to return recognition results')
            self._get_model_mesh_srv = None
            self.get_model_description = None
        
        self._object_id = 0


        rospy.loginfo('Ready to do object detections!')
        
    def detect_objects(self, add_objects_to_map='all', add_table_to_map=True,
                       add_objects_as_mesh='all', reset_collision_objects=True,
                       table_name='', point_head_at=None, reset_collider=True, 
                       labels_callback=None):
        '''
        This is the function you should call to get detections of objects in the scene.  By default, it will 
        give you back a TablewareResult, remove all collision objects (although not attached objects) currently in 
        the collision map and add the objects detected and the table to the map.  This returns first pickup goals for 
        objects recognized as tableware, then for objects recognized but not as tableware and then for objects not
        recognized at all.  In order to use the pickup goals returned, you must fill in the arm_name field.  
        
        The object labels returned in pickup goal reflect the recognition result.  Labels are:
        For tableware: One of the keys in the tableware_labels list.
        For recognized objects that are not tableware: The first tag in the database
        For unrecognized objects: graspable

        **Args:**
        
            *add_objects_to_map (string or False):* This argument controls whether and which objects are added to the
            collision map during detection.  Passing 'all' will add all detected objects to map, 'recognized' 
            will add only recognized objects to map, 'tableware' will add only objects recognized as tableware,  
            and False or 'None' or 'none' will add nothing to the map.
            
            *add_table_to_map (boolean):* If true, this will add the detected table to the map.  If you do not already
            have a representation of the table in the map and you wish to do pick or place it is a good idea to
            have the detection add it so there is a defined region of the map in which collision checking for the
            object can be disabled.  If the table is added to the map, the collision_support_surface_id field of
            the pickup goals will be set to its collision id.  Tables are usually added as meshes although
            if only the bounding box for the table is returned they will be added as boxes.
            
            *add_objects_as_mesh ('all', False, or [string]):* This argument controls which objects are added
            to the map as mesh rather than as a bounding box.  'all' will add all recognized objects as mesh while
            False will add no recognized objects as mesh.  For finer control, you can pass a list of labels that
            you want to add as mesh.
            
            *reset_collision_objects (boolean):* If true, this will remove all collision objects from the map before
            adding new ones.  The detection does no correlation with what is currently in the map so if you have
            done a detection on this table before, you will get repeat objects added to the map if you do not
            clear them first.
            
            *table_name (string):* The name of the table on which the objects are sitting.  If passed in, this will be 
            added to the pickup goals to avoid extraneous collisions caused by the object contacting the table
            (and will also be the table_name in the result).  If the table is added to the collision
            map and this is passed in, the table with have this name in the collision map.  If the table is added
            to the collision map and this is not passed in, it will have the name 'current_table', which is
            returned by the TablewareResult.
            
            *point_head_at (geometry_msgs.msg.PointStamped):* A point at which the head should look for the detection.
            If passed in, the robot will first look at this point, but if it cannot find a table or finds a
            vertical table, it will search along the x axis for a table.  For this reason, it is always a good
            idea to pass in this argument.  If left at None, the detection will not move the head.

            *reset_collider (boolean):* True if you want the collider to be reset before detection.  This will not
            wait for repopulate before the detection but will delay the return from the detection until after the
            repopulate is done if the detection is quick.

            *labels_callback (function: string f(pr2_tasks.pickplace_definitions.PickupGoal, 
            tabletop_object_detector.msgs.Table, string)):* After do_detection is finished assigning a label to a 
            cluster, this function (if not None) will be called to do more post-processing on the cluster.  For 
            example, this function is used to identify utensils by checking the height and pose of any 
            unidentified cluster.  It should return the label of the cluster.


        **Returns:**
            A TablewareDetectionResult.  The pickup goals (see pickplace_definitions.py) in this result have all of the 
            detection information, including recognized models and point cloud information, in the GraspableObject
            field.  The pose at which the object was detected and the label are also returned as fields in the 
            pickup goals.
            
         **Raises:**
         
             **exceptions.DetectionError:** if a table is not found or only vertical tables cannot be found.  If only
             vertical tables were found the error code will be OTHER_ERROR.
         '''

        if reset_collision_objects:
            self._wi.reset_collision_objects()
        if reset_collider:
            #this function takes so long that we'll have repopulated by the end of it
            self._wi.reset_collider_node(repopulate=False)
            reset_time = rospy.Time.now()
        collision_objects = self._wi.collision_objects()
        attached_objects = self._wi.attached_collision_objects()
        used_names = []
        for o in collision_objects:
            used_names.append(o.id)
        for ao in attached_objects:
            used_names.append(ao.object.id)
        res = self._do_detection(point_head_at)
        if add_table_to_map:
            table_name = self._add_table_to_map(res.detection.table, table_name)
        #we want to return first the tableware objects we might want to pick up
        tableware_pgs = []
        #recognized means we recognized them but don't think
        #they are anything we should pick up
        recognized_pgs = []
        #graspable means we could pick them up but we don't
        #know what they are
        graspable_pgs = []
        
        for c in range(len(res.detection.clusters)):
            #Compute the bounding box
            try:
                bres = self._bounding_box_srv(res.detection.clusters[c])
            except rospy.ServiceException, e:
                rospy.logerr('Unable to call cluster bounding box service.  Exception was '+str(e)+
                             '.  Ignoring cluster.')
                continue
            if bres.error_code != bres.SUCCESS:
                rospy.logerr('Cluster bounding box service returned error '+str(bres.error_code)+ 
                             '.  Ignoring cluster')
                continue
            go = GraspableObject()
            go.reference_frame_id = res.detection.clusters[c].header.frame_id
            go.cluster = res.detection.clusters[c]
            pg = PickupGoal('', go, object_pose_stamped=bres.pose,
                            collision_support_surface_name=table_name, allow_gripper_support_collision=True)
            co = CollisionObject()
            label = 'graspable'
            if self.get_model_description and self.get_mesh_from_database and\
                    len(res.detection.models[c].model_list) > 0 and\
                    res.detection.models[c].model_list[0].confidence < 0.005:
                #we recognized this - figure out what it is
                model_id = res.detection.models[c].model_list[0].model_id
                go.potential_models = res.detection.models[c].model_list
                rospy.logdebug('Potential models are: '+ str([m.model_id for m in res.detection.models[c].model_list]))
                try:
                    descr = self.get_model_description(model_id)
                    if descr.return_code.code != descr.return_code.SUCCESS:
                        rospy.logwarn('Get model description returned error '+
                                      str(descr.return_code.code))
                    for t in descr.tags:
                        if t in self.tableware_labels:
                            label = t
                            break
                    if label == 'graspable' and descr.tags:
                        label = descr.tags[0]
                    rospy.logdebug('Name of model is '+descr.name)
                except rospy.ServiceException, e:
                    rospy.logerr('Call to get description threw exception '+str(e))
                shape = None
                if add_objects_as_mesh and (add_objects_as_mesh == 'all' or label in add_objects_as_mesh):
                    try:
                        shape = self.get_mesh_from_database(res.detection.models[c].model_list[0].model_id)
                        if label == 'graspable': label = 'recognized'
                        co.header = res.detection.models[c].model_list[0].pose.header
                        co.shapes.append(shape)
                        co.poses.append(res.detection.models[c].model_list[0].pose.pose)
                        pg.object_pose_stamped = tl.transform_pose_stamped(pg.target.reference_frame_id,
                                                                           res.detection.models[c].model_list[0].pose)
                    except DetectionError:
                        shape = None
                if not shape:
                    co = self._get_bounding_box_collision_object(bres)
                    pg.object_pose_stamped = tl.transform_pose_stamped(pg.target.reference_frame_id, bres.pose)
            else:
                co = self._get_bounding_box_collision_object(bres)
                pg.object_pose_stamped = tl.transform_pose_stamped(pg.target.reference_frame_id, bres.pose)
            co.padding = self.object_padding
            if labels_callback:
                #more logic for assigning labels if you want it
                label = labels_callback(pg, res.detection.table, label)
            rospy.loginfo('Cluster '+str(c)+': '+label)
            self._object_id += 1
            co.id = label+'_'+str(self._object_id)
            while co.id in used_names:
                co.id = label+'_'+str(self._object_id)
                self._object_id += 1
            pg.collision_object_name = co.id
            go.collision_name = co.id
            pg.label = label
            if label in self.tableware_labels:
                tableware_pgs.append(pg)
                if add_objects_to_map == 'all' or add_objects_to_map == 'recognized' or\
                        add_objects_to_map == 'tableware':
                    self._wi.add_object(co)
            elif label == 'graspable':
                graspable_pgs.append(pg)
                if add_objects_to_map == 'all':
                    self._wi.add_object(co)
            else:
                recognized_pgs.append(pg)
                if add_objects_to_map == 'all' or add_objects_to_map == 'recognized':
                    self._wi.add_object(co)
        if reset_collider:
            current_time = rospy.Time.now()
            if current_time - reset_time < rospy.Duration(5.0):
                rospy.loginfo('Sleeping to allow scan to finish')
                rospy.sleep(rospy.Duration(5.0) - (current_time - reset_time))
        return TablewareDetectionResult(tableware_pgs+recognized_pgs+graspable_pgs, res.detection.table, table_name)

    def get_mesh_from_database(self, model_id):
        '''
        Calls the database to get the mesh for an object.

        **Args:**
            **model_id (int):** The id for the model for which you want to get a mesh

        **Returns:**
            An arm_navigation_msgs.msg.Shape corresponding to the mesh.

        **Raises:**
        
             **execeptions.DetectionError:** If the database call fails.  The error code will be the error code the call
             returned with.
        '''
        mesh = self._get_model_mesh_srv(model_id)
        if mesh.return_code.code != mesh.return_code.SUCCESS:
            raise DetectionError('Unable to get mesh from database', error_code=mesh.return_code.code)
        return mesh.mesh

    def get_segmentation_results(self):
        req = TabletopSegmentationRequest()
        resp = self.segmentation_service(req)
        if resp.result != resp.SUCCESS:
            raise DetectionError('Unable to carry out segmentation')
        if len(resp.clusters) < 1:
            raise DetectionError('Unable to carry out segmentation')
        return resp

    def recognize_objects(self, segmentation_result):
        req = TabletopObjectRecognitionRequest()
        req.table = segmentation_result.table
        req.clusters = segmentation_result.clusters
        req.num_models = 1
        req.perform_fit_merge = True

        resp = self.recognition_service(req)    

        tabletop_snapshot = TabletopSnapshot()
        tabletop_snapshot.table = segmentation_result.table
        tabletop_snapshot.timestamp = rospy.Time.now()

        for c in range(len(segmentation_result.clusters)):
            o = Object()
            modellist = resp.models[resp.cluster_model_indices[c]]
            o.pointcloud = segmentation_result.clusters[c]
            if not modellist.model_list:
                rospy.logwarn('Could not recognize any models')
                continue
            o.model_id = modellist.model_list[0].model_id
            o.pose = modellist.model_list[0].pose
            tabletop_snapshot.objects.append(o)
            
        if not tabletop_snapshot.objects:
           raise DetectionError('Unable to recognize any objects')
        return tabletop_snapshot

    def _do_detection(self, point_head_at):
        req = TabletopDetectionRequest()
        req.return_clusters = True
        req.return_models = True
        req.num_models = 1

        if point_head_at:
            point_head_at = tl.transform_point_stamped(self._wi.world_frame, point_head_at)
            zrng = range(0, int(100*self.table_search_max), int(100*self.table_search_resolution))
            sgnrng = [-1.0, 1.0]
            rospy.logdebug('For table finding, trying z offsets for head of:' +str(zrng))
        if not point_head_at or not zrng:
            zrng = [0]
            sgnrng = [1.0]
        first_pass=True

        for z in zrng:
            for sgn in sgnrng:
                if point_head_at:
                    new_point = copy.copy(point_head_at.point)
                    new_point.z += sgn*z/100.0
                    self._head._point_head([new_point.x, new_point.y, new_point.z], point_head_at.header.frame_id)
                    rospy.sleep(0.4) #let the head bobbing settle out
                res = self._detect_srv(req)
                if res.detection.result != res.detection.SUCCESS:
                    rospy.logerr('Unable to find table.  Error was: '+ str(res.detection.result))
                    if first_pass: 
                        exc = DetectionError('Unable to find table', error_code=res.detection.result)
                        first_pass = False
                    continue
                if self.allow_vertical_tables:
                    return res
                table = res.detection.table
                #is the table somewhat horizontal?
                table_pose_bl = tl.transform_pose_stamped(self._wi.robot_frame, table.pose)
                vert = Point()
                vert.z = 1.0
                oripose = Pose()
                oripose.orientation = table_pose_bl.pose.orientation
                table_vertical = gt.transform_point(vert, oripose)
                if table_vertical.z < self.vertical_threshold:
                    rospy.logerr('Table vertical is [%f, %f, %f].  This is not a horizontal table',
                                 table_vertical.x, table_vertical.y, table_vertical.z)
                    res.detection.result = res.detection.OTHER_ERROR
                    if first_pass:
                        exc = DetectionError('Unable to find horizontal table', error_code=res.detection.result)
                        first_pass = False
                    continue
                rospy.loginfo('Found horizontal table. Vertical is [%f, %f, %f]', 
                              table_vertical.x, table_vertical.y, table_vertical.z)
                return res
        raise exc

    def _get_bounding_box_collision_object(self, box):
        #we don't try to extend the box down to the table...
        #so hopefully we detected all the way down
        co = CollisionObject()
        co.header = box.pose.header
        shape = Shape()
        shape.type = shape.BOX
        shape.dimensions.append(box.box_dims.x)
        shape.dimensions.append(box.box_dims.y)
        shape.dimensions.append(box.box_dims.z)
        co.shapes.append(shape)
        co.poses.append(box.pose.pose)
        return co
        
    def _add_table_to_map(self, table, table_name):
        co = CollisionObject()
        co.id = table_name
        if not co.id:
            co.id = 'current_table'
        co.header = table.pose.header
        #we do NOT want a padded table
        co.padding = -1
        if len(table.convex_hull.vertices) > 0:
            if self.table_thickness > 0:
                table_z = range(0, int(1000*self.table_thickness/2.0), int(1000*TABLE_RESOLUTION))
                table_z.append(1000.0*self.table_thickness/2.0)
                sgnrng = [1.0, -1.0]
            else:
                table_z = [0]
                sgnrng = [1.0]
            for z in table_z:
                for sgn in sgnrng:
                    co.shapes.append(table.convex_hull)
                    ps = tl.transform_pose_stamped(self._wi.world_frame, table.pose)
                    ps.pose.position.z += sgn*z/1000.0
                    co.poses.append(tl.transform_pose(table.pose.header.frame_id, ps.header.frame_id, ps.pose))
            rospy.logdebug('Adding table as convex hull')
        else:
            bbox = Shape()
            bbox.type = bbox.BOX
            bbox.dimensions = [abs(table.x_max - table.x_min), abs(table.y_max - table.y_min), self.table_thickness]
            co.shapes.append(bbox)
            co.poses.append(table.pose.pose)
            rospy.logdebug('Adding table as bounding box')
        self._wi.add_object(co)
        return co.id
