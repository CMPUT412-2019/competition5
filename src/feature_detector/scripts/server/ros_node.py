#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
from predict_server import PredictServer
import cv2
import json

rospy.init_node('shape_detect')

bridge = CvBridge()
predict_server = PredictServer('fastai_trained.onnx')
id_to_classname = ['circle', 'square', 'triangle']
prediction_publisher = rospy.Publisher('/pred/result', String)


def on_image(image):
    image = bridge.imgmsg_to_cv2(image, 'bgr8')  # type: np.ndarray
    boxes, labels, colours = predict_server.predict(image, show_mask=False)
    prediction_publisher.publish(json.dumps([boxes, labels, colours]))
    # for (x, y, w, h), label in zip(boxes, labels):
    #     cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 0))
    #     cv2.putText(image, id_to_classname[label], (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0))
    # cv2.imshow('labeled image', image)
    # cv2.waitKey(1)


image_subscriber = rospy.Subscriber('/pred/request', Image, on_image)
rospy.spin()
