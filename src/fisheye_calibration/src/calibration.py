#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv

class StereoVision:
    def __init__(self):
        rospy.init_node('stereo_vision_node', anonymous=True)
        self.bridge = CvBridge()
        self.imgL = None
        self.imgR = None
        self.depth_pub = rospy.Publisher('/depth_image', Image, queue_size=10)
        self.subL = rospy.Subscriber('/camera/fisheye1/image_undistorted', Image, self.left_callback)
        self.subR = rospy.Subscriber('/camera/fisheye2/image_undistorted', Image, self.right_callback)

    def left_callback(self, data):
        self.imgL = cv.cvtColor(self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8"), cv.COLOR_BGR2GRAY)

        self.process_images()

    def right_callback(self, data):
        self.imgR = cv.cvtColor(self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8"), cv.COLOR_BGR2GRAY)

        self.process_images()

    def process_images(self):
        if self.imgL is not None and self.imgR is not None:
            stereo = cv.StereoBM_create(numDisparities=16, blockSize=11)
            disparity = stereo.compute(self.imgL, self.imgR)
            disparity_normalized = cv.normalize(disparity, None, alpha=0, beta=255, norm_type=cv.NORM_MINMAX, dtype=cv.CV_8U)
            disparity_img_msg = self.bridge.cv2_to_imgmsg(disparity_normalized, encoding="mono8")
            self.depth_pub.publish(disparity_img_msg)


def main():
    try:
        stereo_vision = StereoVision()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
