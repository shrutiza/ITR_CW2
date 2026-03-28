#!/usr/bin/env python3
import rospy
import smach
import actionlib
from std_msgs.msg import String
from second_coursework.msg import TidyUpAction, TidyUpResult
from navigate_state import NavigateToRoom
from search_state import SearchRoom
from deliver_state import DeliverObject

def main():
    rospy.init_node('tidy_room_server')

    global tts_pub
    tts_pub = rospy.Publisher('/tts/phrase', String, queue_size=1)
    rospy.sleep(3)

    def execute_cb(goal):
        rospy.loginfo(f"Received goal: tidy {goal.room}")

        #initialise state machine with userdata
        sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
        sm.userdata.current_room = goal.room
        sm.userdata.objects_tidied = []
        sm.userdata.rooms_tidied = []
        sm.userdata.found_object = ''
        sm.userdata.destination_room = ''
        sm.userdata.first_arrival = True

        #CBState to announce arrival only on first visit
        @smach.cb_interface(outcomes=['done'], input_keys=['current_room', 'first_arrival'], output_keys=['first_arrival'])
        def announce_arrival(userdata):
            if userdata.first_arrival:
                tts_pub.publish(f"I have arrived at the {userdata.current_room}")
                userdata.first_arrival = False
            return 'done'

        with sm:
            #navigate to the target room
            smach.StateMachine.add('NAVIGATE_TO_ROOM', NavigateToRoom(tts_pub),
                transitions={'arrived': 'ANNOUNCE_ARRIVAL', 'failed': 'failed'},
                remapping={'target_room': 'current_room'})
            #announce arrival using CBState (only on first visit)
            smach.StateMachine.add('ANNOUNCE_ARRIVAL', smach.CBState(announce_arrival),
                transitions={'done': 'SEARCH_ROOM'})
            #search the room for misplaced objects using YOLO
            smach.StateMachine.add('SEARCH_ROOM', SearchRoom(server, tts_pub),
                transitions={'object_found': 'DELIVER_OBJECT', 'done': 'FINAL_RETURN'}) #misplaced object detected
            #deliver the misplaced object to its correct room
            smach.StateMachine.add('DELIVER_OBJECT', DeliverObject(tts_pub),
                transitions={'delivered': 'NAVIGATE_TO_ROOM', 'done': 'FINAL_RETURN'}) #2 objects tidied - finish
            #return to original room after all tidying is complete
            smach.StateMachine.add('FINAL_RETURN', NavigateToRoom(tts_pub),
                transitions={'arrived': 'succeeded', 'failed': 'succeeded'},
                remapping={'target_room': 'current_room'})

        sm.execute()

        #return result with all tidied objects and their rooms
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