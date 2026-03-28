#!/usr/bin/env python3
import rospy
import smach
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from second_coursework.msg import YOLODetection

#import coordinates
from constants import ROOM_COORDS

class NavigateToRoom(smach.State):
    def __init__(self, tts_pub=None):
        smach.State.__init__(self, outcomes=['arrived', 'failed'],
                             input_keys=['target_room', 'current_room'],
                             output_keys=[])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.detection = None
        self.last_spoken = 0
        self.tts_pub = tts_pub
        #subscribe to YOLO detections to handle person detection during navigation
        self.sub = rospy.Subscriber('/yolo/detections', YOLODetection, self.detection_cb)

    def detection_cb(self, msg):
        self.detection = msg

    def execute(self, userdata):
        rospy.loginfo(f"Navigating to {userdata.target_room}")
        x, y = ROOM_COORDS[userdata.target_room]
        #build and send navigation goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal)

        #during navigation - check for person detections
        while not self.client.wait_for_result(rospy.Duration(0.5)):
            if self.detection is not None:
                if self.detection.name == 'person' and self.tts_pub:
                    now = rospy.Time.now().to_sec()
                    if now - self.last_spoken > 10:   #cooldown to avoid repetition
                        self.tts_pub.publish(f"I am currently tidying the {userdata.current_room}")
                        self.last_spoken = now
                self.detection = None

        rospy.loginfo(f"Arrived at {userdata.target_room}")
        return 'arrived'