#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

def callback(data):
    if len(data.markers) > 0:
        marker = data.markers[0]
        print("AR Tag ID:", marker.id)
        print("Position:", marker.pose.pose.position)
        print("Orientation:", marker.pose.pose.orientation)

def listener():
    rospy.init_node('ar_tag_listener', anonymous=True)
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()