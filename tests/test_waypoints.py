#!/usr/bin/env python3
import math
import time
import unittest
import rospy
import actionlib
import rostest

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from tortoisebot_waypoints.msg import WaypointActionAction, WaypointActionGoal

def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))

class OdomTracker:
    def __init__(self):
        self.x = None
        self.y = None
        self.yaw = None
        self._sub = rospy.Subscriber('/odom', Odometry, self._cb)

    def _cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.yaw = normalize_angle(yaw)

class TestWaypoints(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('waypoints_test_node', anonymous=True)
        cls.odom = OdomTracker()
        cls.client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
        assert cls.client.wait_for_server(rospy.Duration(15.0)), "Action server not available"

    def _wait_for_odom(self, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout and not rospy.is_shutdown():
            if self.odom.x is not None and self.odom.y is not None and self.odom.yaw is not None:
                return True
            rospy.sleep(0.05)
        return False

    def test_final_position(self):
        self.assertTrue(self._wait_for_odom(), "No /odom received")

        goal_x = rospy.get_param('~goal_x', -0.2)
        goal_y = rospy.get_param('~goal_y', -0.2)
        action_timeout = rospy.get_param('~action_timeout', 60.0)
        settle_time = rospy.get_param('~settle_time', 1.0)
        pos_tol = rospy.get_param('~pos_tol', 0.15)

        # send goal
        goal = WaypointActionGoal()
        goal.position = Point(goal_x, goal_y, 0.0)
        self.client.send_goal(goal)
        self.assertTrue(self.client.wait_for_result(rospy.Duration(action_timeout)), "Action timed out")

        # small settle to ensure odom last update
        rospy.sleep(settle_time)

        # check final position
        exp_x = rospy.get_param('~expected_x', goal_x)
        exp_y = rospy.get_param('~expected_y', goal_y)
        dx = (self.odom.x - exp_x)
        dy = (self.odom.y - exp_y)
        dist = math.hypot(dx, dy)

        rospy.loginfo(f"[TEST] final pos=({self.odom.x:.3f},{self.odom.y:.3f}), expected=({exp_x:.3f},{exp_y:.3f}), err={dist:.3f}")
        self.assertLessEqual(dist, pos_tol, f"Position error {dist:.3f} > tol {pos_tol:.3f}")

    def _snapshot_pose(self):
        # wait until we have a pose
        self.assertTrue(self._wait_for_odom(), "No /odom received")
        return (self.odom.x, self.odom.y, self.odom.yaw)

    def test_final_yaw(self):
        # --- params ---
        goal_x = rospy.get_param('~goal_x', -0.2)
        goal_y = rospy.get_param('~goal_y', -0.2)
        action_timeout = rospy.get_param('~action_timeout', 60.0)
        settle_time = rospy.get_param('~settle_time', 1.0)
        yaw_tol_deg = rospy.get_param('~yaw_tol_deg', 10.0)

        # Optional override: if you want a specific final heading
        expected_yaw_deg_param = rospy.get_param('~expected_yaw_deg', None)

        # 1) Snapshot start pose BEFORE sending the goal
        start_x, start_y, _ = self._snapshot_pose()

        # 2) Send goal and wait
        goal = WaypointActionGoal()
        goal.position = Point(goal_x, goal_y, 0.0)
        self.client.send_goal(goal)
        self.assertTrue(self.client.wait_for_result(rospy.Duration(action_timeout)), "Action timed out")
        rospy.sleep(settle_time)

        # 3) Compute expected yaw
        if expected_yaw_deg_param is not None:
            expected_yaw = math.radians(expected_yaw_deg_param)
            source = "param"
        else:
            # face from START -> GOAL (stable even after arrival)
            expected_yaw = math.atan2(goal_y - start_y, goal_x - start_x)
            source = "start->goal"

        # 4) Compare final yaw to expected
        err = abs(normalize_angle(self.odom.yaw - expected_yaw))
        err_deg = math.degrees(err)
        rospy.loginfo(f"[TEST] yaw_final={math.degrees(self.odom.yaw):.1f} deg, "
                    f"yaw_expected({source})={math.degrees(expected_yaw):.1f} deg, "
                    f"err={err_deg:.1f} deg (tol={yaw_tol_deg:.1f})")
        self.assertLessEqual(err_deg, yaw_tol_deg, f"Yaw error {err_deg:.1f} > tol {yaw_tol_deg:.1f}")


if __name__ == '__main__':
    rostest.rosrun('tortoisebot_waypoints', 'waypoints_tests', TestWaypoints)
