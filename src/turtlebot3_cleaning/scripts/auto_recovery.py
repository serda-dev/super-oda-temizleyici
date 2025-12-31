#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
import rosservice

class AutoRecovery:
    def __init__(self):
        rospy.init_node('auto_recovery_monitor')
        
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        
        # Service client for clearing costmaps
        self.clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        
        self.last_cmd_time = rospy.Time.now()
        self.is_stuck = False
        self.stuck_timeout = rospy.Duration(10.0) # 10 seconds without movement command
        self.check_rate = rospy.Rate(1.0) # 1 Hz
        
        rospy.loginfo("Auto Recovery Monitor Started")

    def cmd_vel_callback(self, msg):
        # If we receive non-zero velocity commands, we are not stuck
        if abs(msg.linear.x) > 0.0 or abs(msg.angular.z) > 0.0:
            self.last_cmd_time = rospy.Time.now()

    def status_callback(self, msg):
        if not msg.status_list:
            return

        # Check the latest status
        latest_status = msg.status_list[-1].status
        
        # If aborted (4) or rejected (5), trigger recovery
        if latest_status == GoalStatus.ABORTED or latest_status == GoalStatus.REJECTED:
             rospy.logwarn("MoveBase ABORTED/REJECTED. Triggering Clear Costmaps...")
             self.clear_costmaps()

    def clear_costmaps(self):
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=2.0)
            self.clear_costmaps_srv()
            rospy.loginfo("Costmaps cleared successfully.")
            # Reset timer to avoid spamming
            self.last_cmd_time = rospy.Time.now()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        except rospy.ROSException:
             rospy.logerr("Clear Costmaps service not available. Available services:")
             try:
                 services = rosservice.get_service_list()
                 for s in services:
                     if 'costmap' in s:
                         rospy.logerr(f" - {s}")
             except:
                 pass

    def run(self):
        while not rospy.is_shutdown():
            # Check for silent freeze (no cmd_vel for X seconds while active)
            # Note: This is simplistic. A better way uses odometry, but checking command starvation works for 'planner failed' freezes.
            if rospy.Time.now() - self.last_cmd_time > self.stuck_timeout:
                rospy.logwarn(f"No velocity commands for {self.stuck_timeout.secs}s. Clearing Costmaps...")
                self.clear_costmaps()
            
            self.check_rate.sleep()

if __name__ == '__main__':
    try:
        monitor = AutoRecovery()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
