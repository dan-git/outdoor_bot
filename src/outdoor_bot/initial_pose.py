#!/usr/bin/env/python

import roslib; roslib.load_manifest('myBot')
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def main():
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped)
    rospy.sleep(0.3)
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = '/map'
    pose.pose.pose.position.x = 1.0
    pose.pose.pose.position.y = 1.0
    pose.pose.pose.orientation.w = 1.0
    rospy.loginfo('Publishing:\n' + str(pose))
    pub.publish(pose)

if __name__ == '__main__':
    rospy.init_node('stupid_initial_pose_node')
    main()
