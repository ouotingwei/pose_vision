#!/usr/bin/env python
import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

DIM = [848, 800]
K_1 = np.array([[286.38861083984375, 0.0, 421.372314453125], [0.0, 286.45220947265625, 390.54730224609375], [0.0, 0.0, 1.0]])
D_1 = np.array([[-0.012872150167822838], [0.05464962124824524], [-0.05153217166662216], [0.010507550090551376]])

K_2 = np.array([[286.74090576171875, 0.0, 420.4461975097656], [0.0, 286.7449035644531, 393.5614929199219], [0.0, 0.0, 1.0]])
D_2 = np.array([[-0.008860611356794834], [0.044272929430007935], [-0.04117792844772339], [0.0071413880214095116]])

def undistort(img, K, D):
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    # Crop to the middle 400x400 region
    crop_y = (undistorted_img.shape[0] - 400) // 2
    crop_x = (undistorted_img.shape[1] - 400) // 2
    undistorted_img = undistorted_img[crop_y:crop_y+400, crop_x:crop_x+400]

    return undistorted_img

def image_callback_cam1(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    undist_cam1 = undistort(cv_image, K_1, D_1)

    undist_cam1_msg = bridge.cv2_to_imgmsg(undist_cam1, encoding="bgr8")
    pub_cam1.publish(undist_cam1_msg)

def image_callback_cam2(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    undist_cam2 = undistort(cv_image, K_2, D_2)

    undist_cam2_msg = bridge.cv2_to_imgmsg(undist_cam2, encoding="bgr8")
    pub_cam2.publish(undist_cam2_msg)

def main():
    rospy.init_node('fisheye_calibration')

    global pub_cam1, pub_cam2
    pub_cam1 = rospy.Publisher("/camera/fisheye1/image_undistorted", Image, queue_size=10)
    pub_cam2 = rospy.Publisher("/camera/fisheye2/image_undistorted", Image, queue_size=10)

    rospy.Subscriber("/camera/fisheye1/image_raw", Image, image_callback_cam1)
    rospy.Subscriber("/camera/fisheye2/image_raw", Image, image_callback_cam2)

    rospy.spin()

if __name__ == "__main__":
    main()
