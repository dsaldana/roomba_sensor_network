import rospy
# OpenCV
from cv_bridge import CvBridge
import cv2 as cv

from sensor_msgs.msg import Image


class Camera(object):
    def __init__(self, robotName):
        self.rospy = rospy
        # Sensed value by the camera. How many pixes are
        # in the row divided by the pixes in the line.
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
        # sensor_enabled = self.rospy.get_param('/sensor_enabled', True)
        sensor_enabled = True
        if not sensor_enabled:
            return

        # OpenCV matrix
        mat = CvBridge().imgmsg_to_cv2(img, "rgb8")

        # Extract red channel
        split_image = cv.split(mat)
        red = split_image[0]  # Red image
        green = split_image[1]  # Green image
        blue = split_image[2]  # Blue image

        # How many white pixels in the left and right
        pl, pr = 0, 0

        threshold_value = 160
        threshold_other = 160

        # Count the pixels in the middle row of the image.
        # for i in [int(mat.rows / 2)]:
        # for j in xrange(mat.cols / 2):
        # if red_channel[i, j] > threshold_value and green_channel[i, j] < threshold_other and blue_channel[
        # i, j] < threshold_other:
        #             pl = j
        #
        #     for j in xrange(int(mat.cols / 2) + 1, mat.cols):
        #         if red_channel[i, j] > threshold_value and green_channel[i, j] < threshold_other and blue_channel[
        #             i, j] < threshold_other:
        #             pr = j - int(mat.cols / 2)

        # Take just a row/line of the image
        line_index = -1
        red_line = red[line_index]
        green_line = green[line_index]
        blue_line = blue[line_index]

        # Take the left half of the line
        half = int(len(red_line) / 2)

        f = (red_line > threshold_value) * (green_line < threshold_other) * (blue_line < threshold_other)

        # normalized values
        self.sensed_value = sum(f) / (1.0 * len(f))
        self.sensed_left = sum(f[:half]) / (len(f[:half]))
        self.sensed_right = sum(f[half + 1:]) / (len(f[half + 1:]))

