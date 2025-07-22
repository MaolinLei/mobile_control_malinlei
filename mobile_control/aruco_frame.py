#!/usr/bin/env python3

import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
import math

class ManualFramePublisher:
    def __init__(self):
        rospy.init_node('manual_frame_publisher', anonymous=True)

        self.br = tf2_ros.TransformBroadcaster()

        # Timer to publish frame periodically
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_manual_frame)  # 10 Hz

    def publish_manual_frame(self, event):
        # First frame: custom_frame relative to base_link
        t1 = TransformStamped()
        t1.header.stamp = rospy.Time.now()
        t1.header.frame_id = "base_link"
        t1.child_frame_id = "aruco_frame_first"

        t1.transform.translation.x = -0.043
        t1.transform.translation.y = 0.106
        t1.transform.translation.z = -0.155

        t1.transform.rotation.x = 0.446349
        t1.transform.rotation.y = -0.493811
        t1.transform.rotation.z = -0.520067
        t1.transform.rotation.w = 0.535214

        t1.transform.rotation.x = 0.5
        t1.transform.rotation.y = -0.5
        t1.transform.rotation.z = -0.5
        t1.transform.rotation.w = 0.5

     #    # Second frame: sub_frame relative to custom_frame
        t2 = TransformStamped()
        t2.header.stamp = t1.header.stamp
        t2.header.frame_id = "aruco_frame_first"
        t2.child_frame_id = "aruco_frame_second"

        t2.transform.translation.x = -1.17
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = -0.11

        quat = tf.transformations.quaternion_from_euler(math.pi/2, 0,  -math.pi/2)
        t2.transform.rotation.x = 0
        t2.transform.rotation.y =  -0.7071068
        t2.transform.rotation.z =  0
        t2.transform.rotation.w = 0.7071068


        t3 = TransformStamped()
        t3.header.stamp = t1.header.stamp
        t3.header.frame_id = "aruco_frame_first"
        t3.child_frame_id = "aruco_frame_third"

        t3.transform.translation.x = -1.5
        t3.transform.translation.y = 0.0
        t3.transform.translation.z = -0.11
        t3.transform.rotation.x = 0
        t3.transform.rotation.y =  0.258819
        t3.transform.rotation.z =  0
        t3.transform.rotation.w = 0.9659258





        self.br.sendTransform([t1,t2,t3])
if __name__ == '__main__':
    try:
        publisher = ManualFramePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass