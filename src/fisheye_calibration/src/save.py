#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber:
    def __init__(self):
        rospy.init_node('image_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/fisheye1/image_raw', Image, self.image_callback)
        self.save_count = 0

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # 保存图像
            self.save_image(cv_image)
        except Exception as e:
            rospy.logerr(e)

    def save_image(self, image):
        try:
            # 定义图像文件名
            filename = 'fisheyes.png'
            # 保存图像
            cv2.imwrite(filename, image)
            rospy.loginfo("Image saved as {}".format(filename))
            self.save_count += 1
        except Exception as e:
            rospy.logerr(e)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        image_subscriber = ImageSubscriber()
        image_subscriber.run()
    except rospy.ROSInterruptException:
        pass
