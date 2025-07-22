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
        t1.header.frame_id = "mobile_base"
        t1.child_frame_id = "second_desire"

        t1.transform.translation.x = 0.82389
        t1.transform.translation.y = -0.13365
        t1.transform.translation.z = 0.135


        t1.transform.rotation.x = 0.5
        t1.transform.rotation.y = -0.5
        t1.transform.rotation.z = -0.5
        t1.transform.rotation.w = 0.5

     #    # Second frame: sub_frame relative to custom_frame
        t2 = TransformStamped()
        t2.header.stamp = t1.header.stamp
        t2.header.frame_id = "mobile_base"
        t2.child_frame_id = "first_desire"

        t2.transform.translation.x = 0.8768
        t2.transform.translation.y = -1.01630
        t2.transform.translation.z = 1.35

        t2.transform.rotation.x = -0.5
        t2.transform.rotation.y = -0.5
        t2.transform.rotation.z = -0.5
        t2.transform.rotation.w = -0.5


        t3 = TransformStamped()
        t3.header.stamp = t1.header.stamp
        t3.header.frame_id = "mobile_base"
        t3.child_frame_id = "third_desire"

        t3.transform.translation.x = 0.6825
        t3.transform.translation.y = 1.0
        t3.transform.translation.z = 0.135

        t3.transform.rotation.x =  0.242
        t3.transform.rotation.y =  -0.66423
        t3.transform.rotation.z =  -0.66423
        t3.transform.rotation.w =  0.242465


        self.br.sendTransform([t1,t2,t3])
if __name__ == '__main__':
    try:
        publisher = ManualFramePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
