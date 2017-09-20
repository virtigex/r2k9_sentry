#!/usr/bin/env python

import os
import signal
import sys
import rospy
from shutil import copyfile
import six.moves.urllib as urllib
import tarfile
import tensorflow as tf
import numpy as np
from enum import Enum

from std_msgs.msg import String

from sensor_msgs.msg import Image
from r2k9_sentry.msg import ObjectDetection, DetectedObject

class R2k9Memory:
    def __init__(self):
        self.states = Enum('vacant', 'stranger')
        self.state = self.states.vacant
        self.memory_limit = 100
        rospy.Subscriber('/camera/color/image_raw', Image, self.handle_image)
        rospy.Subscriber('/r2k9/detect', ObjectDetection, self.handle_detection)
        self.images = []

    def terminate(self):
        rospy.loginfo('memory shut down')

    def change_state(self, new_state):
        rospy.loginfo('changing from {} to {}'.format(self.state, new_state))
        if new_state == self.states.vacant:
            rospy.loginfo('vacant'):
        elif new_state == self.states.stanger:
            rospy.loginfo('stranger detected')
        self.state = new_state

    def handle_image(self, image):
        self.images.append(image)
        rospy.loginfo('got image {}'.format(image.header.seq))
        while (len(self.images) > self.memory_limit):
            img = self.images.pop(0)
            rospy.loginfo('discarding image {}'.format(img.header.seq))

    def handle_detection(self, det):
        seq = det.image_seq
        rospy.loginfo('handle_detection {}'.format(seq))
        while len(self.images) > 0 and self.images[0].header.seq < seq:
            self.images.pop(0)
        if len(self.images) > 0 and self.images[0].header.seq == seq:
            img = self.images.pop(0)
        else:
            rospy.loginfo('no image found for {}'.format(seq))

def int_handler(signal, frame):
    print('shutting down')
    rospy.signal_shutdown('interrupted') 

if __name__ == '__main__':
    rospy.init_node('memory')
    signal.signal(signal.SIGINT, int_handler)
    memory = R2k9Memory()
    rospy.spin()
    memory.terminate()
    rospy.loginfo('terminated')
