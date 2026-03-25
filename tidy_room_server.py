#!/usr/bin/env python3
import rospy
import smach
import smach_ros
import actionlib
from second_coursework.msg import TidyUpAction, TidyUpResult, TidyUpFeedback
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from second_coursework.msg import YOLODetection

# Room coordinates (we'll fill these in later from RViz)
ROOM_COORDS = {
    'kitchen': (1.89, 8.53),
    'garage': (6.02, 8.73),
    'bedroom-1': (10.5, 8.71),
    'bedroom-2': (1.88, 3.48),
    'living-room': (6.23, 3.96),
    'bathroom': (10.7, 2.78)
}

# Objects belonging to each room
ROOM_OBJECTS = {
    'kitchen': ['microwave', 'refrigerator', 'bottle', 'sink', 'fork', 'banana', 'sandwich', 'chair'],
    'garage': ['bicycle', 'car', 'sports ball', 'skateboard', 'motorbike'],
    'bedroom-1': ['bottle', 'laptop', 'suitcase', 'tv monitor', 'sports ball', 'skateboard', 'chair', 'bed'],
    'bedroom-2': ['bottle', 'laptop', 'handbag', 'tv monitor', 'sports ball', 'skateboard', 'chair', 'bed'],
    'living-room': ['tv monitor', 'remote', 'book', 'clock', 'chair'],
    'bathroom': ['toothbrush', 'toilet', 'sink']
}

ROOM_WAYPOINTS = {
    'kitchen': [(1.17, 9.0), (2.8, 9.01), (2.58, 7.62)],
    'garage': [],
    'bedroom-1': [],
    'bedroom-2': [],
    'living-room': [],
    'bathroom': []
}

tts_pub = None

class NavigateToRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived', 'failed'],
                             input_keys=['target_room'],
                             output_keys=[])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        rospy.loginfo(f"Navigating to {userdata.target_room}")
        x, y = ROOM_COORDS[userdata.target_room]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo(f"Arrived at {userdata.target_room}")
        tts_pub.publish(f"I have arrived at the {userdata.target_room}")
        return 'arrived'

class SearchRoom(smach.State):
    def __init__(self, server=None):
        smach.State.__init__(self, outcomes=['object_found', 'done'],
                             input_keys=['current_room', 'objects_tidied', 'rooms_tidied'],
                             output_keys=['found_object', 'destination_room', 'objects_tidied', 'rooms_tidied'])
        self.detection = None
        self.server = server
        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.sub = rospy.Subscriber('/yolo/detections', YOLODetection, self.detection_cb)

    def detection_cb(self, msg):
        self.detection = msg

    def find_destination(self, object_name):
        for room, objects in ROOM_OBJECTS.items():
            if object_name in objects:
                return room
        return None

    def execute(self, userdata):
        rospy.loginfo(f"Searching {userdata.current_room}")
        self.detection = None
        rate = rospy.Rate(1)

        for x, y in ROOM_WAYPOINTS[userdata.current_room]:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation.w = 1.0
            self.nav_client.send_goal(goal)

            while not self.nav_client.wait_for_result(rospy.Duration(0.5)):
                if self.detection is not None:
                    name = self.detection.name
                    if name == 'person':
                        tts_pub.publish(f"I am currently tidying the {userdata.current_room}")
                    elif name not in ROOM_OBJECTS[userdata.current_room]:
                        destination = self.find_destination(name)
                        if destination and destination != userdata.current_room:
                            if self.server:
                                feedback = TidyUpFeedback()
                                feedback.object = name
                                feedback.room = destination
                                self.server.publish_feedback(feedback)
                            userdata.found_object = name
                            userdata.destination_room = destination
                            self.nav_client.cancel_goal()
                            return 'object_found'
                    self.detection = None
                rate.sleep()

        if len(userdata.objects_tidied) >= 2:
            return 'done'
        return 'done'

class DeliverObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['delivered'],
                             input_keys=['found_object', 'destination_room', 'current_room', 'objects_tidied', 'rooms_tidied'],
                             output_keys=['objects_tidied', 'rooms_tidied'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, userdata):
        rospy.loginfo(f"Delivering {userdata.found_object} to {userdata.destination_room}")
        tts_pub.publish(f"I am taking the {userdata.found_object} to {userdata.destination_room}")

        x, y = ROOM_COORDS[userdata.destination_room]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal)
        self.client.wait_for_result()

        userdata.objects_tidied.append(userdata.found_object)
        userdata.rooms_tidied.append(userdata.destination_room)

        rospy.loginfo(f"Delivered! Objects tidied so far: {userdata.objects_tidied}")
        return 'delivered'

def main():
    rospy.init_node('tidy_room_server')
    #tts
    global tts_pub
    tts_pub = rospy.Publisher('/tts/phrase', String, queue_size=1)
    rospy.sleep(3)

    def execute_cb(goal):
        rospy.loginfo(f"Received goal: tidy {goal.room}")

        # Set up state machine
        sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
        sm.userdata.current_room = goal.room
        sm.userdata.objects_tidied = []
        sm.userdata.rooms_tidied = []
        sm.userdata.found_object = ''
        sm.userdata.destination_room = ''

        with sm:
            smach.StateMachine.add('NAVIGATE_TO_ROOM', NavigateToRoom(),
                transitions={'arrived': 'SEARCH_ROOM', 'failed': 'failed'},
                remapping={'target_room': 'current_room'})

            smach.StateMachine.add('SEARCH_ROOM', SearchRoom(server),
                transitions={'object_found': 'DELIVER_OBJECT', 'done': 'succeeded'})

            smach.StateMachine.add('DELIVER_OBJECT', DeliverObject(),
                transitions={'delivered': 'NAVIGATE_TO_ROOM'})

        outcome = sm.execute()

        result = TidyUpResult()
        result.objects = sm.userdata.objects_tidied
        result.rooms = sm.userdata.rooms_tidied
        server.set_succeeded(result)

    server = actionlib.SimpleActionServer('/tidy_room', TidyUpAction, execute_cb, False)
    server.start()
    rospy.loginfo("Tidy room server started!")
    rospy.spin()

if __name__ == '__main__':
    main()
