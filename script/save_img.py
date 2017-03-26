#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, SetBoolResponse

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
            print 'ImageConverter_callback:', e

        # image size, type
        (rows, cols, channels) = self.cv_image.shape

    @property
    def cv_img(self):
        ''' image getter: cv_image '''
        return self.cv_image

    def imshow(self):
        if not self.cv_image is None:
            cv2.imshow("Image", self.cv_image)
            cv2.waitKey(10)

class ImageSaver:

    def __init__(self, img_cvt, tar_dir, base_name='', extension='jpg'):
        self.set_saveDirectory(tar_dir)
        self.base_name = base_name if base_name == '' else base_name+'-'
        self.save_count = 0
        self.extension = '.'+extension
        self.update_filePath()
        self.name_repeat = 0

        self.img_cvt = img_cvt
        self.srv = rospy.Service('/save_img', SetBool, self.saveImg_callback)

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
            if 1000 == self.name_repeat:
                print('*** Name repeat 1000 times, please check! ***\n')
                sys.exit(1)

        # claer counter
        if isinstance(self.name_repeat, int):
            self.name_repeat = 0
    
    def update_filePath(self):
        self.save_count += 1
        self.file_name = (self.base_name +
            '{:05d}'.format(self.save_count) +
            self.extension)
        self.file_path = os.path.join(self.directory, self.file_name)

    def save(self):
        ''' save image to desire directory '''
        if not self.img_cvt.cv_img is None:
            self.check_name()
            cv2.imwrite(self.file_path, self.img_cvt.cv_img)
            print 'save image:', self.file_path
            self.update_filePath()
            return True

        return False
    
    def saveImg_callback(self, req):
        res = SetBoolResponse()
        
        if req.data:
            result = self.save()
            res.success = result
            res.message = '' if result else 'image not ready'
        else:
            print req.data
            res.success = False

        return res

if __name__ == '__main__':

    rospy.init_node('image_save', anonymous=True)

    # instance of image converter and saver
    img_cvt = ImageConverter()
    img_sav = ImageSaver(img_cvt, '~/test3', '5')

    rospy.loginfo('save image running')

    pic_num = 0
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        img_cvt.imshow()

        # if pic_num < 100:
        #     if img_sav.save():
        #         pic_num += 1

        rate.sleep()

    cv2.destroyAllWindows()
