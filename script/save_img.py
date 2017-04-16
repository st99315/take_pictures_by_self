"""Get realsense image from realsense node, and save image."""

#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, SetBoolResponse

import os
import sys

_NAME_REPEAT = 1000
_TIME_DIFF = 5.0


class ImageConverter:
    """Get realsense image and convert to cv image type."""

    def __init__(self, topic='/camera/color/image_raw', type=Image):
        """Initial cv_bridge, subscriber, and image."""
        self.__bridge = CvBridge()
        self.__img_sub = rospy.Subscriber(
            topic,
            type,
            self.__callback,
            queue_size=1)

        self.__cv_img = None
        self.img_stamp = None

    def __callback(self, data):
        try:
            self.__cv_img = self.__bridge.imgmsg_to_cv2(data, "bgr8")
            self.img_stamp = data.header.stamp
        except CvBridgeError as e:
            print 'ImageConverter_callback:', e

        # image size, type
        (rows, cols, channels) = self.__cv_img.shape

    @property
    def cv_img(self):
        """Image getter: cv_image."""
        return self.__cv_img

    def imshow(self, event=None):
        """Show cv image."""
        if self.__cv_img is not None:
            cv2.imshow("Image", self.__cv_img)
            cv2.waitKey(10)


class ImageSaver:
    """Saving image and setting its directory."""

    def __init__(self, img_cvt, tar_dir, sub_dir='1', base_name='', extension='jpg'):
        self.__main_dir = tar_dir
        self.__sub_dir = int(sub_dir)
        self.__set_saveDirectory()

        self.__base_name = base_name if base_name == '' else base_name+'-'
        self.__save_count = 0
        self.__extension = '.'+extension

        self.__img_cvt = img_cvt
        self.__srv = rospy.Service(
            '/save_img', SetBool, self.__saveImg_callback)

    def __set_saveDirectory(self):
        """Setting directory of pictures to save."""
        self.__update_dir()
        os.makedirs(self.__directory)

    def __update_dir(self):
        while True:
            directory = os.path.join(self.__main_dir, '{:03d}'.format(self.__sub_dir))
            # convert ~/ to abs_path
            self.__directory = os.path.expanduser(directory)
            if os.path.exists(self.__directory):
                self.__sub_dir += 1
            else:
                break

    def __check_name(self):
        while os.path.isfile(self.__file_path):
            self.__update_filePath()

    def __update_filePath(self):
        """Update file path of image to save."""
        self.__save_count += 1
        self.__file_name = (self.__base_name +
                            '{:05d}'.format(self.__save_count) +
                            self.__extension)
        self.__file_path = os.path.join(self.__directory, self.__file_name)

    def __saveImg_callback(self, req):
        """Service server request callback."""
        res = SetBoolResponse()

        if req.data:
            result = self.save()
            res.success = result
            res.message = self.__file_name if result else 'image not ready'
        else:
            """Data of requset is false then change name of directory."""
            self.__set_saveDirectory()
            res.success = True
            res.message = self.__directory
            print 'Change directory:', self.__directory
            self.__save_count = 0

        return res

    def save(self):
        """Save image to desire directory."""
        if self.__img_cvt.cv_img is not None:
            # is image refreshed
            req_time = rospy.Time.now().to_sec()
            if abs(req_time - self.__img_cvt.img_stamp.to_sec()) > _TIME_DIFF:
                return False

            self.__update_filePath()
            self.__check_name()
            cv2.imwrite(self.__file_path, self.__img_cvt.cv_img)
            print 'save image:', self.__file_path
            return True

        return False


if __name__ == '__main__':

    rospy.init_node('image_save', anonymous=True)

    # param for file path
    directory = rospy.get_param('directory', '~/test')
    sub_dir   = rospy.get_param('sub_dir',   '001')
    item_name = rospy.get_param('item_name', 'unknow')

    # instance of image converter and saver
    img_cvt = ImageConverter()
    img_sav = ImageSaver(img_cvt, directory, sub_dir, item_name)

    rospy.on_shutdown(cv2.destroyAllWindows)
    rospy.loginfo('Image saver is running.')

    # timer for show image 20hz
    # comment the line below if using ssh connect 
    rospy.Timer(rospy.Duration(.05), img_cvt.imshow)
    rospy.spin()
