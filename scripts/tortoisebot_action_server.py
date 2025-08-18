#!/usr/bin/env python3
import math
import rospy
import actionlib
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations

from tortoisebot_waypoints.msg import (
    WaypointActionAction,
    WaypointActionFeedback,
    WaypointActionResult
)

def normalize_angle(z):
    a = math.atan2(math.sin(z), math.cos(z))
    return a

class WaypointActionClass(object):
    _feedback = WaypointActionFeedback()
    _result = WaypointActionResult()

    def __init__(self):
        self._position = Point()
        self._yaw = 0.0
        self._state = "idle"
        self._des_pos = Point()

        self._yaw_precision = math.pi / 90.0  # ~2 deg
        self._dist_precision = 0.05           # 5 cm

        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._sub_odom = rospy.Subscriber('/odom', Odometry, self._clbk_odom)

        self._as = actionlib.SimpleActionServer(
            "tortoisebot_as", WaypointActionAction, self.goal_callback, False
        )
        self._rate = rospy.Rate(25)
        self._as.start()
        rospy.loginfo("tortoisebot_as action server started")

    def _clbk_odom(self, msg):
        self._position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        euler = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._yaw = normalize_angle(euler[2])

    def goal_callback(self, goal):
        rospy.loginfo(f"Goal received: {goal.position}")
        success = True

        self._des_pos = goal.position
        time_limit = rospy.get_param("~time_limit", 120.0)  # safety timeout
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            # timeout?
            if (rospy.Time.now() - start_time).to_sec() > time_limit:
                rospy.logwarn("Timeout reached; preempting.")
                self._as.set_preempted()
                success = False
                break

            # allow cancel
            if self._as.is_preempt_requested():
                rospy.loginfo("Preempt requested; cancelling goal.")
                self._as.set_preempted()
                success = False
                break

            # recompute errors
            desired_yaw = math.atan2(
                self._des_pos.y - self._position.y,
                self._des_pos.x - self._position.x
            )
            err_yaw = normalize_angle(desired_yaw - self._yaw)
            err_pos = math.hypot(self._des_pos.x - self._position.x,
                                 self._des_pos.y - self._position.y)

            twist = Twist()
            if err_pos > self._dist_precision:
                if abs(err_yaw) > self._yaw_precision:
                    self._state = "fix yaw"
                    twist.angular.z = 0.65 if err_yaw > 0.0 else -0.65
                else:
                    self._state = "go to point"
                    twist.linear.x = 0.6
            else:
                # reached
                break

            self._pub_cmd_vel.publish(twist)

            self._feedback.position = self._position
            self._feedback.state = self._state
            self._as.publish_feedback(self._feedback)

            self._rate.sleep()

        # stop the robot
        self._pub_cmd_vel.publish(Twist())

        if success:
            self._result.success = True
            self._as.set_succeeded(self._result)

if __name__ == "__main__":
    rospy.init_node("tortoisebot_as")
    WaypointActionClass()
    rospy.spin()
