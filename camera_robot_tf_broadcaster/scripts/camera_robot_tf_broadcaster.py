#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import math

def publish_transform():
    rospy.init_node('link6_to_camera_tf_publisher')
    
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "Link_6"
    static_transformStamped.child_frame_id = "camera_color_frame"

    # 设置平移
    static_transformStamped.transform.translation.x = 0.591
    static_transformStamped.transform.translation.y = 0.347
    static_transformStamped.transform.translation.z = -0.212

    # 设置旋转 (四元数)
    static_transformStamped.transform.rotation.x = 0.013
    static_transformStamped.transform.rotation.y = -0.382
    static_transformStamped.transform.rotation.z = -0.619
    static_transformStamped.transform.rotation.w = 0.686

    broadcaster.sendTransform(static_transformStamped)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        publish_transform()
    except rospy.ROSInterruptException:
        pass
