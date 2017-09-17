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
from cv_bridge import CvBridge, CvBridgeError
import cv2

from std_msgs.msg import String

from sensor_msgs.msg import Image
from r2k9_sentry.msg import ObjectDetection, DetectedObject
import visualization_utils as vis_util
import label_map_util

rospy.init_node('r2k9_tracker')
pubAccouncer = rospy.Publisher('/r2k9/detect', ObjectDetection, queue_size=10)
bridge = CvBridge()

HEADLESS = rospy.get_param('~headless', True)
MAGNIFY = rospy.get_param('~magnify', 1)

# What model to download.
MODEL_NAME = 'ssd_mobilenet_v1_coco_11_06_2017'
MODEL_FILE = MODEL_NAME + '.tar.gz'
DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'

# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

# List of the strings that is used to add correct label for each box.
DOWNLOAD_LABELS='https://raw.githubusercontent.com/tensorflow/models/master/object_detection/data/mscoco_label_map.pbtxt'
#PATH_TO_LABELS = os.path.join('data', 'mscoco_label_map.pbtxt')
PATH_TO_LABELS = os.path.join('mscoco_label_map.pbtxt')

NUM_CLASSES = 90

# Download Model
if not os.path.isfile(MODEL_FILE):
    print('download model')
    opener = urllib.request.URLopener()
    opener.retrieve(DOWNLOAD_BASE + MODEL_FILE, MODEL_FILE)
    tar_file = tarfile.open(MODEL_FILE)
    for file in tar_file.getmembers():
        file_name = os.path.basename(file.name)
        if 'frozen_inference_graph.pb' in file_name:
            tar_file.extract(file, os.getcwd())
else:
    print('using cached model')

# download labels
if not os.path.isfile(PATH_TO_LABELS):
    print('download labels')
    opener = urllib.request.URLopener()
    opener.retrieve(DOWNLOAD_LABELS, PATH_TO_LABELS)

# ## Load a (frozen) Tensorflow model into memory.
print('loading model')
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        print('importing graph')
        tf.import_graph_def(od_graph_def, name='')
        print('done')

# ## Loading label map
# Label maps map indices to category names, so that when our convolution network predicts `5`, we know that this corresponds to `airplane`.  Here we use internal utility functions, but anything that returns a dictionary mapping integers to appropriate string labels would be fine

label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES,
                                                            use_display_name=True)
category_index = label_map_util.create_category_index(categories)
print(category_index)

def_graph = detection_graph.as_default()
sess = tf.Session(graph=detection_graph)

def got_image(image):
    det = ObjectDetection()
    det.header = image.header
    # TODO - fix image encoding
    # desired_encoding="passthrough" gave me blue skin
    image_np = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
    height, width, channels = image_np.shape
    det.height = height
    det.width = width

    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
    image_np_expanded = np.expand_dims(image_np, axis=0)
    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
    # Each box represents a part of the image where a particular object was detected.
    boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
    # Each score represent how level of confidence for each of the objects.
    # Score is shown on the result image, together with the class label.
    scores = detection_graph.get_tensor_by_name('detection_scores:0')
    classes = detection_graph.get_tensor_by_name('detection_classes:0')
    num_detections = detection_graph.get_tensor_by_name('num_detections:0')
    # Actual detection.
    (boxes, scores, classes, num_detections) = sess.run(
        [boxes, scores, classes, num_detections],
        feed_dict={image_tensor: image_np_expanded})

    print('width', width, 'height', height)
    min_score_thresh = .4
    labels = []
    s = np.squeeze(scores)
    b = np.squeeze(boxes)
    c = np.squeeze(classes).astype(np.int32)
    for i in range(b.shape[0]):
        if s is None or s[i] > min_score_thresh:
            obj = DetectedObject()
            object_name = category_index[c[i]]['name']
            ypmin, xpmin, ypmax, xpmax = tuple(b[i].tolist())

            obj.name = object_name
            obj.xmin = xpmin
            obj.xmax = xpmax
            obj.ymin = ypmin
            obj.ymax = ypmax
            labels.append(object_name)
            det.objects.append(obj)

    if not HEADLESS:
        # Visualization of the results of a detection.
        display_img = np.copy(image_np)
        height, width, channels = display_img.shape
        vis_util.visualize_boxes_and_labels_on_image_array(
            display_img,
            b, c, s,
            category_index,
            use_normalized_coordinates=True,
            line_thickness=2)
        mag_size = ( MAGNIFY * width, MAGNIFY * height)
        cv2.imshow("R2K9 vision", cv2.resize(display_img, mag_size))
        cv2.waitKey(3)
    rospy.loginfo(' '.join(labels))
    pubAccouncer.publish(det)

def int_handler(signal, frame):
    print('shutting down')
    rospy.signal_shutdown('interrupted') 

if __name__ == '__main__':
    signal.signal(signal.SIGINT, int_handler)
    rospy.Subscriber('/camera/color/image_raw', Image, got_image)
    rospy.spin()
    sess.close()
    cv2.destroyAllWindows()
    print('terminated')
