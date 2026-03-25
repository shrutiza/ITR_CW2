#!/usr/bin/env python3
# coding=utf-8

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from yolov4 import Detector
from second_coursework.msg import YOLODetection
from std_msgs.msg import String


class YOLOv4ROSITR:

    def __init__(self):
        self.cv_image = None
        self.bridge = CvBridge()

        self.cam_subs = rospy.Subscriber(
            "/usb_cam/image_raw",
            Image,
            self.img_callback
        )

        self.detection_pub = rospy.Publisher(
            '/yolo/detections',
            YOLODetection,
            queue_size=10
        )

        self.detector = Detector(
            gpu_id=0,
            config_path='/opt/darknet/cfg/yolov4.cfg',
            weights_path='/opt/darknet/yolov4.weights',
            lib_darknet_path='/opt/darknet/libdarknet.so',
            meta_path='/home/k25043271/ros_ws/src/second_coursework/cfg/coco.data'
        )

        rospy.loginfo("YOLO node ready.")
        self.run()

    def img_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.cv_image is None:
                rate.sleep()
                continue

            img_arr = cv2.resize(
                self.cv_image,
                (self.detector.network_width(), self.detector.network_height())
            )
            detections = self.detector.perform_detect(
                image_path_or_buf=img_arr,
                show_image=False
            )

            for detection in detections:
                d = YOLODetection(
                    detection.class_name,
                    detection.class_confidence,
                    detection.left_x,
                    detection.top_y,
                    detection.width,
                    detection.height
                )
                self.detection_pub.publish(d)

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('yolo_ros_itr')
    yolo_ros = YOLOv4ROSITR()
    rospy.spin()