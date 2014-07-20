
import rospy
# OpenCV
from cv_bridge import CvBridge
import cv

from sensor_msgs.msg import Image


class Camera(object):

    def __init__(self,  robotName):
        self.rospy = rospy
        # Sensed value by the camera
        self.sensed_value = 0
        # How many white pixels in the left
        self.sensed_left = 0
        # How many white pixels in the right
        self.sensed_right = 0

        topic_name = "/" + robotName + "/front_cam/camera/image"
        image_sub = rospy.Subscriber(topic_name, Image, self.img_callback, queue_size=1)


    def img_callback(self, img):
        """
        Callback for camera sensor.
        Receive images from local camera.
        :param img: received image from local camera
        :return:
        """
        sensor_enabled = self.rospy.get_param('/sensor_enabled', True)

        if not sensor_enabled:
            return

        #OpenCV matrix
        mat = CvBridge().imgmsg_to_cv(img, "rgb8")
        # Extract red channel
        red_channel = cv.CreateImage(cv.GetSize(mat), 8, 1)
        green_channel = cv.CreateImage(cv.GetSize(mat), 8, 1)
        blue_channel = cv.CreateImage(cv.GetSize(mat), 8, 1)
        cv.Split(mat, red_channel, green_channel, blue_channel, None)


        # How many white pixels in the left
        pl = 0
        # How many white pixels in the right
        pr = 0

        threshold_value = 160
        threshold_other = 160

        # Count the pixels in the middle row of the image.
        for i in [int(mat.rows / 2)]:
            for j in xrange(mat.cols / 2):
                if red_channel[i, j] > threshold_value and green_channel[i, j] < threshold_other and blue_channel[
                    i, j] < threshold_other:
                    pl = j

            for j in xrange(int(mat.cols / 2) + 1, mat.cols):
                if red_channel[i, j] > threshold_value and green_channel[i, j] < threshold_other and blue_channel[
                    i, j] < threshold_other:
                    pr = j - int(mat.cols / 2)

        total = mat.rows * mat.cols * 1.0
        self.sensed_value = (pl + pr) / total
        self.sensed_left = (pl * 2.0) / mat.rows
        self.sensed_right = (pr * 2.0) / mat.rows

