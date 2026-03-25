#!/usr/bin/env python3
# coding=utf-8

import rospy
from cv_bridge import CvBridge  # Note: normally this would be from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from second_coursework.srv import YOLOLastFrame, YOLOLastFrameResponse
from second_coursework.msg import YOLODetection


# MAP GOAL COORDINATES - FIXXX
GOALS = {
    'book':     {'x':  1.8439291715621948,  'y':  8.396284103393555,  'w': 1.0},   #top left
    'cell phone': {'x':  10.471418380737305,  'y':  8.27509880065918,  'w': 1.0},   #top right
    'bottle':       {'x': 6.25718879699707,  'y':  3.661388874053955,  'w': 1.0},   #bottom middle
}

STOP_OBJECT = 'apple'


class YOLONavigationNode:

    def __init__(self):
        # Initialise the image field to None (as taught in lecture)
        self.cv_image = None

        # CvBridge to convert ROS image messages to OpenCV images
        self.bridge = CvBridge()

        # Subscribe to the webcam topic (as taught in lecture)
        self.cam_subs = rospy.Subscriber(
            "/usb_cam/image_raw",
            Image,
            self.img_callback
        )

        # Initialise the move_base action client for navigation
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server()
        rospy.loginfo("move_base action server ready.")

        rospy.loginfo("YOLONavigationNode initialised. Starting detection loop...")
        self.run()

    # ------------------------------------------------------------------
    # Camera callback — stores the latest frame (exactly as in lecture)
    # ------------------------------------------------------------------
    def img_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # ------------------------------------------------------------------
    # Call the YOLO service and return the list of detections
    # (mirrors the service-call pattern shown in lecture)
    # ------------------------------------------------------------------
    def call_yolo_service(self):
        rospy.wait_for_service('/detect_frame')
        try:
            detect = rospy.ServiceProxy('/detect_frame', YOLOLastFrame)
            response = detect()
            return response.detections
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return []

    # ------------------------------------------------------------------
    # Send a navigation goal to move_base and wait for the result
    # ------------------------------------------------------------------
    def navigate_to(self, x, y, w):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = w

        rospy.loginfo(f"Navigating to x={x}, y={y}")
        self.move_base.send_goal(goal)
        self.move_base.wait_for_result()

        state = self.move_base.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Navigation succeeded.")
        else:
            rospy.logwarn(f"Navigation finished with state: {state}")

    # ------------------------------------------------------------------
    # Main detection loop
    # ------------------------------------------------------------------
    def run(self):
        rate = rospy.Rate(1)  # Check once per second

        while not rospy.is_shutdown():

            # Wait until we have received at least one image
            if self.cv_image is None:
                rospy.loginfo("Waiting for camera image...")
                rate.sleep()
                continue

            # Call YOLO service to get detections on the latest frame
            detections = self.call_yolo_service()

            if not detections:
                rospy.loginfo("No objects detected.")
                rate.sleep()
                continue

            # Print all detections (mirrors lecture service callback output)
            for detection in detections:
                box = (detection.bbox_x, detection.bbox_y,
                       detection.width, detection.height)
                rospy.loginfo(
                    f"{detection.name.ljust(10)} | "
                    f"{detection.confidence * 100:.1f}% | {box}"
                )

            # Collect the names of detected objects
            detected_names = [d.name for d in detections]

            # ---- Stop condition: apple seen ----
            if STOP_OBJECT in detected_names:
                rospy.loginfo("Apple detected — ending program.")
                break

            # ---- Navigation triggers ----
            # Priority order: book > cell phone > bottle
            # (only the first matching trigger is acted upon per cycle)
            navigated = False
            for obj, coords in GOALS.items():
                if obj in detected_names:
                    rospy.loginfo(f"'{obj}' detected — navigating to goal.")
                    self.navigate_to(coords['x'], coords['y'], coords['w'])
                    navigated = True
                    break

            if not navigated:
                rospy.loginfo("No target objects detected this cycle.")

            rate.sleep()

        rospy.loginfo("Node shutting down.")


# ----------------------------------------------------------------------
# Entry point (exactly as in lecture node skeleton)
# ----------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('yolo_navigation_itr')
    node = YOLONavigationNode()
    rospy.spin()