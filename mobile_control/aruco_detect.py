#!/usr/bin/env python3

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped

class ArucoDetector:
    def __init__(self):
        rospy.init_node('aruco_detector_node', anonymous=True)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size=1, buff_size=2**24)
        self.br = tf2_ros.TransformBroadcaster()

        # Define ArUco dictionary and parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()

        # Example camera calibration parameters (replace with actual values)
        self.camera_matrix = np.array([[615.0, 0.0, 320.0],
                                       [0.0, 615.0, 240.0],
                                       [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.zeros((5, 1))  # Assuming no distortion

        # Set correct parent frame based on your system
        self.camera_frame = "camera_color_optical_frame"  # <- Change this based on your TF tree

    def image_callback(self, data):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"CV Bridge error: {e}")

    def run(self):
        rate = rospy.Rate(2)  # Run at 2 Hz (adjust as needed)
        while not rospy.is_shutdown():
            if self.latest_image is None:
                rate.sleep()
                continue

            cv_image = self.latest_image.copy()
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            if ids is not None:
                aruco.drawDetectedMarkers(cv_image, corners, ids)
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.10, self.camera_matrix, self.dist_coeffs)

                for i in range(len(ids)):
                    if ids[i][0] not in [0, 5]:
                        continue

                    rvec, tvec = rvecs[i][0], tvecs[i][0]
                    aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)
                    rotation_matrix, _ = cv2.Rodrigues(rvec)
                    quat = tf.transformations.quaternion_from_matrix(
                        np.vstack((np.hstack((rotation_matrix, np.array([[0], [0], [0]]))), [0, 0, 0, 1]))
                    )

                    t = TransformStamped()
                    t.header.stamp = rospy.get_rostime()
                    t.header.frame_id = self.camera_frame
                    t.child_frame_id = f"aruco_marker_{ids[i][0]}"
                    t.transform.translation.x = tvec[0]
                    t.transform.translation.y = tvec[1]
                    t.transform.translation.z = tvec[2]
                    t.transform.rotation.x = quat[0]
                    t.transform.rotation.y = quat[1]
                    t.transform.rotation.z = quat[2]
                    t.transform.rotation.w = quat[3]
                    self.br.sendTransform(t)

                    rospy.loginfo(f"[aruco_marker_{ids[i][0]}] Position: {tvec}")
                    rospy.loginfo(f"[aruco_marker_{ids[i][0]}] Rotation matrix:\n{rotation_matrix}")

            cv2.imshow("Aruco Detection", cv_image)
            cv2.waitKey(1)
            rate.sleep()

if __name__ == '__main__':
    detector = ArucoDetector()
    try:
        detector.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
