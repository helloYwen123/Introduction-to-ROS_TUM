#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from perception_msgs.msg import LightState
from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import Twist

# init variables
signal_value = False
is_init = True
v = Float64()
omega = Float64()

class WaitingState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["drive"])
    def execute(self, userdata):
        global is_init
        rospy.loginfo("Waiting for launch command...")
        rate = rospy.Rate(100)
        while is_init and not rospy.is_shutdown():
            v = Float64()
            v.data = 0.0  # Some value to publish in the stop state
            target_v_pub.publish(v)
            omega = Float64()
            omega.data = 0.0
            target_omega_pub.publish(omega)
            rate.sleep()
            rospy.loginfo("Waiting for launch command...")
            init_done = rospy.get_param("/state_machine/init_done", False)
            if is_init and init_done:
                rospy.loginfo("Launching...")
                is_init=False
        return "drive"

class DriveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["stop"])

    def execute(self, userdata):
        rospy.loginfo("Traffic light signal false, start...")
        rate = rospy.Rate(100)  # 100 Hz
        while not signal_value and not rospy.is_shutdown() and not is_init:
            state_pub.publish(Bool(data=False))  # is_stop: false
            target_v_pub.publish(v)
            target_omega_pub.publish(omega)
            rate.sleep()

        return "stop"


class StopState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["drive"])

    def execute(self, userdata):
        rospy.loginfo("Traffic light signal True, stop...")
        rate = rospy.Rate(100)  # 100 Hz

        while signal_value and not rospy.is_shutdown() and not is_init:
            state_pub.publish(Bool(data=True))  # is_stop: true
            v = Float64()
            v.data = 0.0  # Some value to publish in the stop state
            target_v_pub.publish(v)
            omega = Float64()
            omega.data = 0.0
            target_omega_pub.publish(omega)
            rate.sleep()

        return "drive"


def target_twist_callback(target_twist_msg):
    global v, omega
    v.data = target_twist_msg.linear.x
    omega.data = target_twist_msg.angular.z


class TrafficLight:
    def __init__(self) -> None:
        self.signal_sub = rospy.Subscriber("traffic_light_state", LightState, self.signal_callback)
        self.last_signal_value = False
        self.counter = 0

    def signal_callback(self, LightState):
        global signal_value
        if self.last_signal_value == LightState.is_stop:
            self.counter += 1
        else:
            self.last_signal_value = LightState.is_stop
            self.counter = 0
        if self.counter > 10:
            signal_value = LightState.is_stop
            self.counter = 0


def main():
    rospy.init_node("smach")

    global signal_value, state_pub, target_v_pub, target_omega_pub, is_init

    is_init=True
    signal_value = False
    traffic_light = TrafficLight()
    state_pub = rospy.Publisher("/drive_state", Bool, queue_size=10)

    twist_sub = rospy.Subscriber("target_twist", Twist, target_twist_callback)

    target_v_pub = rospy.Publisher("/target_linear_velocity", Float64, queue_size=10)
    target_omega_pub = rospy.Publisher("/target_angular_velocity", Float64, queue_size=10)
    sm = smach.StateMachine(outcomes=[])

    with sm:
        smach.StateMachine.add('WAIT', WaitingState(), transitions={"drive": "DRIVE"})
        smach.StateMachine.add('DRIVE', DriveState(), transitions={"stop": "STOP"})
        smach.StateMachine.add('STOP', StopState(), transitions={"drive": "DRIVE"})


    sm.set_initial_state(['WAIT'])
    outcome = sm.execute()

    rospy.spin()


if __name__ == "__main__":
    main()
