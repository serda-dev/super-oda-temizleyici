#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

class WaypointNavigatorNode:
    def __init__(self):
        rospy.init_node('waypoint_navigator_node', anonymous=True)
        
        # Klasik move_base işlemini oynatıyor.
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server")

        # Ana Mission Manager ile iletişim kurar.
        self.goal_sub = rospy.Subscriber("/mission/goal", PoseStamped, self.goal_callback)
        self.result_pub = rospy.Publisher("/mission/result", String, queue_size=10)

    def goal_callback(self, msg):
        rospy.loginfo(f"Received goal: x={msg.pose.position.x}, y={msg.pose.position.y}")
        
        goal = MoveBaseGoal()
        goal.target_pose = msg
        goal.target_pose.header.stamp = rospy.Time.now()


        self.client.send_goal(goal)
        
        finished = self.client.wait_for_result(rospy.Duration(60.0))

        if finished:
            state = self.client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Navigation Succeeded")
                self.result_pub.publish("SUCCESS")
            else:
                rospy.loginfo(f"Navigation Failed with state: {state}")
                self.result_pub.publish("FAIL")
        else:
            rospy.loginfo("Navigation Timed Out")
            self.client.cancel_goal()
            self.result_pub.publish("FAIL")

if __name__ == '__main__':
    try:
        node = WaypointNavigatorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
