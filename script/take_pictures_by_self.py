#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import os
import sys

class ImageConverter:

    def __init__(self, topic='/camera/color/image_raw', type=Image):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            topic,
            type,
            self.callback,
            queue_size = 1)

        self.cv_image = None

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        # image size, type
        (rows, cols, channels) = self.cv_image.shape

    @property
    def cv_img(self):
        ''' image getter: cv_image '''
        return self.cv_image

class ImageSaver:

    def __init__(self, tar_dir, base_name='', extension='jpg'):
        self.set_saveDirectory(tar_dir)

        self.base_name = base_name
        self.save_count = 0
        self.extension = '.'+extension
        self.update_filePath()

        self.name_repeat = 0

    def set_saveDirectory(self, directory):
        ''' set directory of pictures to save '''
        # convert ~/ to abs_path
        self.directory = os.path.expanduser(directory)
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)
            self.name_repeat = None

    def check_name(self):
        while (os.path.isfile(self.file_path) and
            isinstance(self.name_repeat, int)):

            self.update_filePath()
            self.name_repeat += 1
            if 10 == self.name_repeat:
                print('*** Name repeat 10 times, please check!\n')
                sys.exit(1)
        # claer counter
        if self.name_repeat is int:
            self.name_repeat = 0
    
    def update_filePath(self):
        self.save_count += 1
        self.file_name = (self.base_name +
            '{:05d}'.format(self.save_count) +
            self.extension)
        self.file_path = os.path.join(self.directory, self.file_name)

    def save(self, image):
        ''' save image to desire directory '''
        self.check_name()
        print 'save image:', self.file_path

        cv2.imwrite(self.file_path, image)
        self.update_filePath()

if __name__ == '__main__':

    rospy.init_node('image_converter', anonymous=True)

    # instance of image converter and saver
    img_cvt = ImageConverter()
    img_sav = ImageSaver('~/test1')

    num = 0
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        img = img_cvt.cv_img

        if not img is None:
            cv2.imshow("Image window", img)
            cv2.waitKey(10)

            img_sav.save(img)
            num+=1
            if num == 100:
                break

        rate.sleep()

    cv2.destroyAllWindows()
