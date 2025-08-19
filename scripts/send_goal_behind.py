#!/usr/bin/env python3
import math
import rospy
import actionlib
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Import the ROS1 action types generated from WaypointAction.action
from tortoisebot_waypoints.msg import WaypointActionAction, WaypointActionGoal

def get_pose():
    msg = rospy.wait_for_message('/odom', Odometry, timeout=5.0)
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    return x, y, yaw

def main():
    rospy.init_node('send_goal_behind')
    # distance behind (meters) — tweak if you need more clearance
    d = rospy.get_param('~back_distance', 0.1)

    x, y, yaw = get_pose()
    # goal behind robot in world frame
    gx = x - d * math.cos(yaw)
    gy = y - d * math.sin(yaw)

    client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
    rospy.loginfo("Waiting for tortoisebot_as...")
    client.wait_for_server()

    goal = WaypointActionGoal()
    goal.position = Point(gx, gy, 0.0)

    rospy.loginfo("Sending goal behind: (%.2f, %.2f) from pose (%.2f, %.2f, yaw=%.1f°)",
                  gx, gy, x, y, math.degrees(yaw))
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(90.0))
    rospy.loginfo("Done. Result: %s", str(client.get_result()))

if __name__ == '__main__':
    main()
