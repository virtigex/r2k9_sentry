#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import os
import signal
import sys
import rospy
#import tensorflow
from shutil import copyfile
from std_msgs.msg import String
from r2k9.msg import TrainingReady
from r2k9.msg import TrainingUpdate

rospy.init_node('r2k9_trainer')
pubAccouncer = rospy.Publisher('/r2k9/alert', TrainingUpdate, queue_size=10)

def trainer(training):
  rospy.loginfo('training ' + training.location + ' ip ' + training.host)
  file = os.path.split(training.location)[-1]
  copyfile(training.location, file)
  os.remove(training.location)
  rospy.loginfo('processed ' + file)

def int_handler(signal, frame):
  print('shutting down')
  rospy.signal_shutdown('interrupted') 

if __name__ == '__main__':
  signal.signal(signal.SIGINT, int_handler)
  rospy.Subscriber('/r2k9/training', TrainingReady, trainer)
  rospy.spin()
  print('terminated')
