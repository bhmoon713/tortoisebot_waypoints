#! /usr/bin/env python
import rospy
import time
import actionlib

from tortoisebot_waypoints.msg import WaypointActionFeedback, WaypointActionResult, WaypointActionAction
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import math

class WaypointActionClass(object):

    # create messages that are used to publish feedback/result
    _feedback = WaypointActionFeedback()
    _result = WaypointActionResult()

    # topics
    _pub_cmd_vel = None
    _sub_odom = None

    # go to point vars
    # robot state variables
    _position = Point()
    _yaw = 0
    # machine state
    _state = 'idle'
    # goal
    _des_pos = Point()
    # parameters
    _yaw_precision = math.pi / 90 # +/- 2 degree allowed
    _dist_precision = 0.05

    def __init__(self):
        # creates the action server
        self._as = actionlib.SimpleActionServer("tortoisebot_as", WaypointActionAction, self.goal_callback, False)
        self._as.start()

        # define a loop rate
        self._rate = rospy.Rate(25)

        # topics
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._sub_odom = rospy.Subscriber('/odom', Odometry, self._clbk_odom)
        rospy.loginfo("Action server started")

    def _clbk_odom(self, msg):
        # position
        self._position = msg.pose.pose.position

        # yaw
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self._yaw = euler[2]

    def goal_callback(self, goal):
        rospy.loginfo("goal %s received" % str(goal))

        # helper variables
        success = True

        # define desired position and errors
        self._des_pos = goal.position
        desired_yaw = math.atan2(self._des_pos.y - self._position.y, self._des_pos.x - self._position.x)
        err_pos = math.sqrt(pow(self._des_pos.y - self._position.y, 2) + pow(self._des_pos.x - self._position.x, 2))
        err_yaw = desired_yaw - self._yaw

        # perform task
        while err_pos > self._dist_precision and success:
            # update vars
            desired_yaw = math.atan2(self._des_pos.y - self._position.y, self._des_pos.x - self._position.x)
            err_yaw = desired_yaw - self._yaw
            err_pos = math.sqrt(pow(self._des_pos.y - self._position.y, 2) + pow(self._des_pos.x - self._position.x, 2))
            rospy.loginfo("Current Yaw: %s" % str(self._yaw))
            rospy.loginfo("Desired Yaw: %s" % str(desired_yaw))
            rospy.loginfo("Error Yaw: %s" % str(err_yaw))
            # --- parameters (put these near __init__ as class members if you want) ---
            yaw_enter = math.radians(8.0)    # enter "fix yaw" if |err_yaw| > 8°
            yaw_leave = math.radians(3.0)    # leave "fix yaw" when |err_yaw| < 3°
            k_ang     = 1.0                  # P gain for angular speed
            w_max     = 0.3                  # max |angular.z|
            w_min     = 0.05                 # minimum non-zero turn to overcome stiction
            v_max     = 0.2                  # max forward speed
            steer_k   = 0.05                 # small steering while moving

            # --- always normalize yaw error first ---
            err_yaw = math.atan2(math.sin(err_yaw), math.cos(err_yaw))  # [-pi, pi]

            # helper: saturated angular speed with small non-zero minimum
            def sat_turn(w):
                if abs(w) < 1e-6:
                    return 0.0
                w = max(-w_max, min(w_max, w))
                if abs(w) < w_min:
                    w = w_min if w > 0 else -w_min
                return w

            # state machine with hysteresis + proportional control
            twist_msg = Twist()
            if err_pos > self._dist_precision:
                if abs(err_yaw) > yaw_enter or self._state == 'fix yaw' and abs(err_yaw) > yaw_leave:
                    # FIX YAW
                    self._state = 'fix yaw'
                    w = sat_turn(k_ang * err_yaw)
                    twist_msg.angular.z = w
                else:
                    # GO TO POINT
                    self._state = 'go to point'
                    # scale forward speed down when not well aligned (smooths transitions)
                    align = max(0.0, 1.0 - (abs(err_yaw) / yaw_enter))  # 0..1
                    twist_msg.linear.x  = max(0.15, v_max * align)
                    twist_msg.angular.z = sat_turn(steer_k * err_yaw)   # gentle steering while moving
            else:
                # reached
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0

            self._pub_cmd_vel.publish(twist_msg)


            # send feedback
            self._feedback.position = self._position
            self._feedback.state = self._state
            self._as.publish_feedback(self._feedback)

            # loop rate
            self._rate.sleep()

        # stop
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        self._pub_cmd_vel.publish(twist_msg)

        # return success
        if success:
            self._result.success = True
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('tortoisebot_as')
    WaypointActionClass()
    rospy.spin()