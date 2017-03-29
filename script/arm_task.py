"""Use to generate arm task and run."""

#!/usr/bin/env python

import rospy

from std_msgs.msg import String, Float64
from robotis_controller_msgs.msg import StatusMsg
from manipulator_h_base_module_msgs.msg import IK_Cmd

import sys
from math import radians, degrees

_ROLL_MAX, _ROLL_MIN = 30, -30
_PITCH_MAX, _PITCH_MIN = -10, -80
_YAW_MAX, _YAW_MIN = 40, -40
_STEP = 5

_POS = (0, .6, .1)  # x, y, z
_ORI = (-20, 0, 0)  # pitch, roll, yaw


class ArmTask:
    """Running arm task class."""

    def __init__(self):
        """Inital object."""
        self.__set_pubSub()
        rospy.on_shutdown(self.stop_task)
        self.__set_mode_pub.publish('set')
        self.__generator = self.gen_nextEuler()

    def __set_pubSub(self):
        self.__set_mode_pub = rospy.Publisher(
            '/robotis/base/set_mode_msg',
            String,
            latch=True,
            queue_size=1)

        self.__set_endlink_pub = rospy.Publisher(
            '/robotis/base/set_endlink',
            Float64,
            latch=True,
            queue_size=1)

        self.__ptp_pub = rospy.Publisher(
            '/robotis/base/JointP2P_msg',
            IK_Cmd,
            latch=True,
            queue_size=1)

        self.__cmd_pub = rospy.Publisher(
            '/robotis/base/TaskP2P_msg',
            IK_Cmd,
            latch=True,
            queue_size=1)

        self.__status_sub = rospy.Subscriber(
            '/robotis/status',
            StatusMsg,
            self.__status_callback,
            queue_size=10)

    def __status_callback(self, msg):
        if 'IK Failed' in msg.status_msg:
            rospy.logwarn('ik fail')
            self.stop_task()

        elif 'End Trajectory' in msg.status_msg:
            self.__is_busy = False

    def pub_ikCmd(self, mode='line', pos=_POS, euler=_ORI):
        """Publish ik cmd msg to manager node."""
        cmd = []

        for p in pos:
            cmd.append(p)
        for e in euler:
            cmd.append(e)

        if 'line' == mode:
            self.__cmd_pub.publish(cmd)
        elif 'ptp' == mode:
            self.__ptp_pub.publish(cmd)

        self.__is_busy = True

    def stop_task(self):
        """Stop task running."""
        self.__set_mode_pub.publish('')

    def set_endlink(self, dis_m):
        """TODO: change endlink."""
        self.__set_endlink_pub.publish(dis_m)

    def gen_nextEuler(self):
        """Generator euler angle."""
        p, y = _PITCH_MAX, _YAW_MAX

        for p in range(_PITCH_MAX, _PITCH_MIN - _STEP, -_STEP):
            if p % 2 == 0:
                for y in range(_YAW_MAX, _YAW_MIN - _STEP, -_STEP):
                    yield (p, y)
            else:
                for y in range(_YAW_MIN, _YAW_MAX + _STEP, _STEP):
                    yield (p, y)

    def run(self):
        """Get euler angle and run task."""
        if self.__is_busy:
            return
        else:
            (p, y) = next(self.__generator)
            self.pub_ikCmd('ptp', euler=(p, 0, y))


if __name__ == '__main__':

    rospy.init_node('robot_arm_task', anonymous=True)
    rospy.loginfo('robot arm task running')
    dis_m = rospy.get_param('distance', 0.1)

    task = ArmTask()
    task.set_endlink(dis_m)
    rospy.sleep(1)
    task.pub_ikCmd('ptp')

    try:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            task.run()
            rate.sleep()

    except rospy.exceptions.ROSInterruptException as e:
        print 'close'
