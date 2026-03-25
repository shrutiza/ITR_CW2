#!/usr/bin/env python3
# coding=utf-8

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from yolov4 import Detector
from second_coursework.srv import YOLOLastFrame, YOLOLastFrameResponse
from second_coursework.msg import YOLODetection


class YOLOv4ROSITR:

    def __init__(self):
        # Initialise image field to None
        self.cv_image = None
        self.bridge = CvBridge()

        # Subscribe to the webcam
        self.cam_subs = rospy.Subscriber(
            "/usb_cam/image_raw",
            Image,
            self.img_callback
        )

        # Declare the service
        self.yolo_srv = rospy.Service(
            '/detect_frame',
            YOLOLastFrame,
            self.yolo_service
        )

        self.detector = Detector(
            gpu_id=0,
            config_path='/opt/darknet/cfg/yolov4.cfg',
            weights_path='/opt/darknet/yolov4.weights',
            lib_darknet_path='/opt/darknet/libdarknet.so',
            meta_path='/home/k25043271/ros_ws/src/second_coursework/cfg/coco.data'
        )

        rospy.loginfo("YOLO node ready.")

    # Camera callback
    def img_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough'
        )

    # Service callback
    def yolo_service(self, request):
        res = YOLOLastFrameResponse()

        if self.cv_image is None:
            rospy.logwarn("Have not yet received any image.")
            return res

        # Resize and run detection
        img_arr = cv2.resize(
            self.cv_image,
            (self.detector.network_width(), self.detector.network_height())
        )
        detections = self.detector.perform_detect(
            image_path_or_buf=img_arr,
            show_image=True
        )

        # Build response
        for detection in detections:
            box = detection.left_x, detection.top_y, \
                  detection.width, detection.height
            print(
                f'{detection.class_name.ljust(10)} | '
                f'{detection.class_confidence * 100:.1f} % | {box}'
            )
            d = YOLODetection(
                detection.class_name,
                detection.class_confidence,
                detection.left_x,
                detection.top_y,
                detection.width,
                detection.height
            )
            res.detections.append(d)

        return res


if __name__ == '__main__':
    rospy.init_node('yolo_ros_itr')
    yolo_ros = YOLOv4ROSITR()
    rospy.spin()