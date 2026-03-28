#!/usr/bin/env python3
import rospy
import smach
import smach_ros
import random
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyResponse

dirt_detected    = False
battery_low      = False
obstacle_detected = False

#callbacks:
def dirt_callback(msg):
    global dirt_detected
    if msg.data:
        rospy.loginfo("[EVENT] Dirt detected")
        dirt_detected = True


def battery_low_callback(req):
    global battery_low
    rospy.loginfo("[EVENT] Battery low")
    battery_low = True
    return EmptyResponse()


def scan_callback(msg):
    global obstacle_detected
    valid = [r for r in msg.ranges if not math.isnan(r) and not math.isinf(r)]
    if valid and min(valid) < 0.5:
        obstacle_detected = True


#helpers:
def publish_twist(pub, linear_x=0.0, angular_z=0.0, duration=0.1):
    msg = Twist()
    msg.linear.x  = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)
    rospy.sleep(duration)


def stop_robot(pub):
    publish_twist(pub, 0.0, 0.0, 0.1)


#1) moving state
class Moving(smach.State):
    """
    Move the robot in a random forward+turn pattern.
    Transitions:
      obstacle - turning, dirt - spiralling, low_battery - NavigatingToCharger
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['obstacle', 'dirt', 'low_battery'])
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def execute(self, userdata):
        global obstacle_detected, dirt_detected, battery_low

        rospy.loginfo("State: MOVING")

        # move in short bursts, checking flags each iteration
        rate = rospy.Rate(10)
        for _ in range(20):
            if rospy.is_shutdown():
                break

            linear  = 0.2
            angular = random.uniform(-0.5, 0.5)
            publish_twist(self.cmd_pub, linear, angular, 0.0)
            rate.sleep()

            if obstacle_detected:
                stop_robot(self.cmd_pub)
                obstacle_detected = False
                return 'obstacle'
            if dirt_detected:
                stop_robot(self.cmd_pub)
                dirt_detected = False
                return 'dirt'
            if battery_low:
                stop_robot(self.cmd_pub)
                battery_low = False
                return 'low_battery'

        stop_robot(self.cmd_pub)
        return 'obstacle'  # fallback: check for obstacle after movement burst


#2) turning state
class Turning(smach.State):
    """
    Turn in place until the obstacle is no longer detected.
    Transitions: obstacle - Turning, no_obstacle - Moving
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['obstacle', 'no_obstacle'])
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def execute(self, userdata):
        global obstacle_detected

        rospy.loginfo("State: TURNING")

        #turn once and recheck
        rate = rospy.Rate(0.5)
        #back up before turning
        for _ in range(10):
            publish_twist(self.cmd_pub, -0.1, 0.0, 0.0)
            rate.sleep()

        for _ in range(15):
            publish_twist(self.cmd_pub, 0.0, 0.5, 0.0)
            rate.sleep()

        stop_robot(self.cmd_pub)
        rospy.sleep(0.2)

        if obstacle_detected:
            obstacle_detected = False   # reset
            return 'obstacle'           # self-loop - keep turning
        else:
            return 'no_obstacle'


#3) spiralling state

class Spiralling(smach.State):
    """
    Spiral outward to cover the dirty area.
    Self-loop while dirt is still being detected; return to Moving when clear.
    Transitions: dirt - Spiralling  (self-loop), no_dirt - Moving
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['dirt', 'no_dirt'])
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.spiral_radius = 0.1   #starting radius

    def execute(self, userdata):
        global dirt_detected

        rospy.loginfo("State: SPIRALLING (radius={:.2f})".format(self.spiral_radius))

        rate = rospy.Rate(10)
        steps = 30

        for i in range(steps):
            #gradua;ly increase linear speed to increase the spiral
            linear  = self.spiral_radius + (i / float(steps)) * 0.1
            angular = 0.8   #constant angular velocity
            publish_twist(self.cmd_pub, linear, angular, 0.0)
            rate.sleep()

        stop_robot(self.cmd_pub)

        #grow radius for next pass
        self.spiral_radius = min(self.spiral_radius + 0.05, 0.4)

        if dirt_detected:
            dirt_detected = False
            return 'dirt'    #self-loop - more dirt detected during cleaning
        else:
            self.spiral_radius = 0.1   #reset for next time
            return 'no_dirt'


#4) charging state

class Charging(smach.State):
    """
    Sit at the charging station until fully charged.
    Transitions: charged - Moving
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['charged'])

    def execute(self, userdata):
        rospy.loginfo("State: CHARGING — waiting 10 s to simulate full charge")
        rospy.sleep(10)
        rospy.loginfo("State: CHARGING — battery full, resuming")
        return 'charged'


#main method
def main():
    rospy.init_node('vacuum_state_machine')

    #subscribers & service
    rospy.Subscriber('/dirt_detected', Bool,      dirt_callback)
    rospy.Subscriber('/scan',          LaserScan,  scan_callback)
    rospy.Service('/battery_low',      Empty,      battery_low_callback)

    #charging station goal for move_base
    charge_goal = MoveBaseGoal()
    charge_goal.target_pose.header.frame_id    = "map"
    charge_goal.target_pose.pose.position.x    = 0.0
    charge_goal.target_pose.pose.position.y    = 0.0
    charge_goal.target_pose.pose.orientation.w = 1.0

    #state Machine
    sm = smach.StateMachine(outcomes=['finished'])

    with sm:

        # 1. MOVING — random exploration
        smach.StateMachine.add(
            'MOVING',
            Moving(),
            transitions={
                'obstacle':    'TURNING',
                'dirt':        'SPIRALLING',
                'low_battery': 'GO_CHARGE'
            }
        )

        # 2. TURNING — self-loop until obstacle clears
        smach.StateMachine.add(
            'TURNING',
            Turning(),
            transitions={
                'obstacle':    'TURNING',   # self-loop
                'no_obstacle': 'MOVING'
            }
        )

        # 3. SPIRALLING — self-loop while dirt is present
        smach.StateMachine.add(
            'SPIRALLING',
            Spiralling(),
            transitions={
                'dirt':    'SPIRALLING',   # self-loop
                'no_dirt': 'MOVING'
            }
        )

        # 4. GO_CHARGE — SimpleActionState using move_base (as required by brief)
        smach.StateMachine.add(
            'GO_CHARGE',
            smach_ros.SimpleActionState(
                'move_base',
                MoveBaseAction,
                goal=charge_goal
            ),
            transitions={
                'succeeded': 'CHARGING',
                'aborted':   'MOVING',     # if navigation fails, resume cleaning
                'preempted': 'MOVING'
            }
        )

        # 5. CHARGING — wait until full, then back to Moving
        smach.StateMachine.add(
            'CHARGING',
            Charging(),
            transitions={
                'charged': 'MOVING'
            }
        )

    #visualise with smach_viewer
    sis = smach_ros.IntrospectionServer('vacuum_smach', sm, '/VACUUM_SM')
    sis.start()

    outcome = sm.execute()
    rospy.loginfo("State machine finished with outcome: {}".format(outcome))

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
