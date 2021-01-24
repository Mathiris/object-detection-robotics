#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ros_numpy import numpify, msgify
import rospkg

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as viz_utils

import numpy as np
import tensorflow as tf
import os
import time


class Vision:
    def __init__(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path('mr_vision') # path is the path to the package mr_vision

        path_to_model_dir = "../RobotIA/robotics_cc/src/mr_vision/src/mr_vision/ssd_mobilenet_v2_fpnlite_640"
        path_to_saved_model = path_to_model_dir + "/saved_model"
        path_to_labels = "../RobotIA/robotics_cc/src/mr_vision/src/mr_vision/Label/label_map.pbtxt"

        print('Loading model...', end='')
        start_time = time.time()

        # Load saved model and build the detection function
        self.detect_fn = tf.saved_model.load(path_to_saved_model)

        end_time = time.time()
        elapsed_time = end_time - start_time
        print('Done! Took {} seconds'.format(elapsed_time))

        self.category_index = label_map_util.create_category_index_from_labelmap(path_to_labels,
                                                                                 use_display_name=True)

        self.image_sub = rospy.Subscriber('/rrbot/camera1/image_raw', Image, self.img_cb)
        self.image_pub = rospy.Publisher('/stop_sign', Image)


    def img_cb(self, img):
        rospy.loginfo('received an image')
        image_np = numpify(img)

        input_tensor = tf.convert_to_tensor(image_np)[tf.newaxis, ...]

        start_detection_time = time.time()

        # Perform detection on new image
        detections = self.detect_fn(input_tensor)

        end_detection_time = time.time()
        elapsed_time = end_detection_time - start_detection_time
        print('Done! Took {} seconds'.format(elapsed_time))

        # Format and add bounding boxes and labels to image
        num_detections = int(detections.pop('num_detections'))
        detections = {key: value[0, :num_detections].numpy()
                    for key, value in detections.items()}
        detections['num_detections'] = num_detections

        detections['detection_classes'] = detections['detection_classes'].astype(np.int64)

        image_np_with_detections = image_np.copy()

        viz_utils.visualize_boxes_and_labels_on_image_array(
            image_np_with_detections,
            detections['detection_boxes'],
            detections['detection_classes'],
            detections['detection_scores'],
            self.category_index,
            use_normalized_coordinates=True,
            max_boxes_to_draw=200,
            min_score_thresh=.75,
            agnostic_mode=False
        )

        self.image_pub.publish(msgify(Image, image_np_with_detections, encoding='rgb8'))


if __name__ == '__main__':
    rospy.init_node('vision', anonymous=True)
    vision = Vision()
    rospy.spin()
