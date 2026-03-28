#!/usr/bin/env python3
import rospy
import smach
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from second_coursework.msg import YOLODetection, TidyUpFeedback

#import room details
from constants import ROOM_OBJECTS, ROOM_WAYPOINTS

class SearchRoom(smach.State):
    def __init__(self, server=None, tts_pub=None):
        smach.State.__init__(self, outcomes=['object_found', 'done'],
                             input_keys=['current_room', 'objects_tidied', 'rooms_tidied'],
                             output_keys=['found_object', 'destination_room', 'objects_tidied', 'rooms_tidied'])
        self.detection = None
        self.server = server
        self.tts_pub = tts_pub
        self.last_spoken = 0
        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #subscribe to YOLO detections topic
        self.sub = rospy.Subscriber('/yolo/detections', YOLODetection, self.detection_cb)

    def detection_cb(self, msg):
        self.detection = msg

    def find_destination(self, object_name):
        #find which room the object belongs to
        for room, objects in ROOM_OBJECTS.items():
            if object_name in objects:
                return room
        return None

    def execute(self, userdata):
        rospy.loginfo(f"Searching {userdata.current_room}")
        #check if already tidied 2 objects - no need to search
        if len(userdata.objects_tidied) >= 2:
            return 'done'

        self.detection = None

        #keep looping through waypoints until a misplaced object is found
        while not rospy.is_shutdown():
            for x, y in ROOM_WAYPOINTS[userdata.current_room]:
                #send robot to each waypoint
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = x
                goal.target_pose.pose.position.y = y
                goal.target_pose.pose.orientation.w = 1.0
                self.nav_client.send_goal(goal)

                #while moving to waypoint - check YOLO detections
                while not self.nav_client.wait_for_result(rospy.Duration(0.5)):
                    if self.detection is not None:
                        name = self.detection.name
                        self.detection = None
                        if name == 'person':
                            #say current tidying activity
                            now = rospy.Time.now().to_sec()
                            if now - self.last_spoken > 10:
                                self.tts_pub.publish(f"I am currently tidying the {userdata.current_room}")
                                self.last_spoken = now
                        elif name not in ROOM_OBJECTS[userdata.current_room]:
                            #object doesn't belong here - find where it should go
                            destination = self.find_destination(name)
                            if destination and destination != userdata.current_room:
                                #publish feedback with object and destination
                                if self.server:
                                    feedback = TidyUpFeedback()
                                    feedback.object = name
                                    feedback.room = destination
                                    self.server.publish_feedback(feedback)
                                userdata.found_object = name
                                userdata.destination_room = destination
                                self.nav_client.cancel_goal()
                                return 'object_found'

            if len(userdata.objects_tidied) >= 2:
                return 'done'

        return 'done'