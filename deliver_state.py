#!/usr/bin/env python3
import rospy
import smach
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from second_coursework.msg import YOLODetection

#import coordinates
from constants import ROOM_COORDS

class DeliverObject(smach.State):
    def __init__(self, tts_pub=None):
        smach.State.__init__(self, outcomes=['delivered', 'done'],
                             input_keys=['found_object', 'destination_room', 'current_room', 'objects_tidied', 'rooms_tidied'],
                             output_keys=['objects_tidied', 'rooms_tidied'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.detection = None
        self.last_spoken = 0
        self.tts_pub = tts_pub
        #subscribe to YOLO detections to handle person detection during delivery
        self.sub = rospy.Subscriber('/yolo/detections', YOLODetection, self.detection_cb)

    def detection_cb(self, msg):
        self.detection = msg

    def execute(self, userdata):
        rospy.loginfo(f"Delivering {userdata.found_object} to {userdata.destination_room}")

        #build and send navigation goal to destination room
        x, y = ROOM_COORDS[userdata.destination_room]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal)

        #while delivering - check for person detections
        while not self.client.wait_for_result(rospy.Duration(0.5)):
            if self.detection is not None:
                if self.detection.name == 'person':
                    now = rospy.Time.now().to_sec()
                    if now - self.last_spoken > 10:
                        self.tts_pub.publish(f"I am taking the {userdata.found_object} to {userdata.destination_room}")
                        self.last_spoken = now
                self.detection = None

        #update tidied lists
        userdata.objects_tidied.append(userdata.found_object)
        userdata.rooms_tidied.append(userdata.destination_room)
        rospy.loginfo(f"Delivered! Objects tidied so far: {userdata.objects_tidied}")

        #stop after 2 objects delivered
        if len(userdata.objects_tidied) >= 2:
            return 'done'
        return 'delivered'