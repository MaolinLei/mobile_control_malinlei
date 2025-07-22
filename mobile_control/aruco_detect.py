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
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            aruco.drawDetectedMarkers(cv_image, corners, ids)

            # Marker size is 0.10m (10cm)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.10, self.camera_matrix, self.dist_coeffs)

            for i in range(len(ids)):
                if ids[i][0] not in [0, 5]:
                    continue

                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                # Draw axis
                aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)

                # Convert rotation vector to quaternion
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                transformation_matrix = np.eye(4)
                transformation_matrix[:3, :3] = rotation_matrix
                quat = tf.transformations.quaternion_from_matrix(
                    np.vstack((np.hstack((rotation_matrix, np.array([[0],[0],[0]]))), [0,0,0,1]))
                )

                # Publish TF frame for RViz visualization
                t = TransformStamped()
                t.header.stamp = rospy.get_rostime()  # More robust timestamp
                t.header.frame_id = self.camera_frame  # Use correct camera frame
                t.child_frame_id = f"aruco_marker_{ids[i][0]}"
                t.transform.translation.x = tvec[0]
                t.transform.translation.y = tvec[1]
                t.transform.translation.z = tvec[2]
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]
                self.br.sendTransform(t)

                # Print marker pose (position + rotation matrix)
                rospy.loginfo(f"[aruco_marker_{ids[i][0]}] Position (x,y,z): {tvec[0]:.3f}, {tvec[1]:.3f}, {tvec[2]:.3f}")
                rospy.loginfo(f"[aruco_marker_{ids[i][0]}] Rotation matrix:\n{rotation_matrix}")

        cv2.imshow("Aruco Detection", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    detector = ArucoDetector()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()