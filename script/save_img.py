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
        self.__bridge = CvBridge()
        self.__img_sub = rospy.Subscriber(
            topic,
            type,
            self.__callback,
            queue_size = 1)

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
        ''' image getter: cv_image '''
        return self.__cv_img

    def imshow(self):
        if not self.__cv_img is None:
            cv2.imshow("Image", self.__cv_img)
            cv2.waitKey(10)

class ImageSaver:

    def __init__(self, img_cvt, tar_dir, base_name='', extension='jpg'):
        self.__set_saveDirectory(tar_dir)
        self.__base_name = base_name if base_name == '' else base_name+'-'
        self.__save_count = 0
        self.__extension = '.'+extension
        self.__update_filePath()
        self.__name_repeat = 0

        self.__img_cvt = img_cvt
        self.__srv = rospy.Service('/save_img', SetBool, self.__saveImg_callback)

    def __set_saveDirectory(self, directory):
        ''' set directory of pictures to save '''
        # convert ~/ to abs_path
        self.__directory = os.path.expanduser(directory)
        if not os.path.exists(self.__directory):
            os.makedirs(self.__directory)
            self.__name_repeat = None

    def __check_name(self):
        while (os.path.isfile(self.__file_path) and
            isinstance(self.__name_repeat, int)):

            self.__update_filePath()
            self.__name_repeat += 1
            if 1000 == self.__name_repeat:
                print('*** Name repeat 1000 times, please check! ***\n')
                sys.exit(1)

        # claer counter
        if isinstance(self.__name_repeat, int):
            self.__name_repeat = 0
    
    def __update_filePath(self):
        self.__save_count += 1
        self.__file_name = (self.__base_name +
            '{:05d}'.format(self.__save_count) +
            self.__extension)
        self.__file_path = os.path.join(self.__directory, self.__file_name)

    def __saveImg_callback(self, req):
        res = SetBoolResponse()
        
        if req.data:
            result = self.save()
            res.success = result
            res.message = '' if result else 'image not ready'
        else:
            print req.data
            res.success = False

        return res

    def save(self):
        ''' save image to desire directory '''
        if not self.__img_cvt.cv_img is None:
            # is image refreshed
            req_time = rospy.Time.now().to_sec()
            if  abs(req_time - self.__img_cvt.img_stamp.to_sec()) > 5.0:
                return False
            
            self.__check_name()
            cv2.imwrite(self.__file_path, self.__img_cvt.cv_img)
            print 'save image:', self.__file_path
            self.__update_filePath()
            return True

        return False

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

        if pic_num < 100:
            if img_sav.save():
                pic_num += 1

        rate.sleep()

    cv2.destroyAllWindows()
