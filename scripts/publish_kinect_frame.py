#!/usr/bin/env python
from argparse import ArgumentParser
import roslib
roslib.load_manifest('myBot')
import rospy
import tf

def publish():
    parser = ArgumentParser()
    parser.add_argument(dest = 'x', type = float, help = 'x coordinate')
    parser.add_argument(dest = 'y', type = float, help = 'y coordinate')
    parser.add_argument(dest = 'z', type = float, help = 'z coordinate')
    parser.add_argument(dest = 'r', type = float, help = 'roll coordinate')
    parser.add_argument(dest = 'p', type = float, help = 'pitch coordinate')
    parser.add_argument(dest = 'w', type = float, help = 'yaw coordinate')

    br = tf.TransformBroadcaster()
    args = parser.parse_args()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        br.sendTransform(
            (args.x, args.y, args.z),
            tf.transformations.quaternion_from_euler(args.r, args.p, args.w),
            rospy.Time.now(),
            'camera_link',
            'fourth_level_link')
        rate.sleep()

rospy.init_node('kinect_frame_broadcaster')
publish()
