#!/usr/bin/env python3

import rospy
import yaml
import time
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

# BEKLENEN ÖZELLİKLER
# - Oda girişine git vs.
INIT = "INIT"
GO_TO_ROOM_ENTRY = "GO_TO_ROOM_ENTRY"
QR_VERIFY = "QR_VERIFY"
EXECUTE_CLEANING = "EXECUTE_CLEANING"
NEXT_ROOM = "NEXT_ROOM"
FINISH = "FINISH"

class MissionManagerNode:
    def __init__(self):
        rospy.init_node('mission_manager_node', anonymous=True)
        
        # TASK parametreleri.
        self.rooms = rospy.get_param("/mission_config/rooms", [])
        if not self.rooms:
            rospy.logerr("No rooms found in parameters! Check launch file.")
            
        self.goal_pub = rospy.Publisher("/mission/goal", PoseStamped, queue_size=10)
        self.nav_result_sub = rospy.Subscriber("/mission/result", String, self.nav_result_callback)
        self.qr_sub = rospy.Subscriber("/qr_code_result", String, self.qr_callback)
        
        self.nav_status = None
        self.detected_qr = None
        
        self.current_room_idx = 0
        self.report = [] # Odaların adlarını ve durumlarını tutar.

    def nav_result_callback(self, msg):
        self.nav_status = msg.data

    def qr_callback(self, msg):
        self.detected_qr = msg.data

    def send_goal(self, x, y, yaw):
        self.nav_status = None # Reset
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        
        q = quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation = Quaternion(*q)
        
        self.goal_pub.publish(goal)
        
    def wait_for_nav(self, timeout=60):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.nav_status:
                return self.nav_status
            time.sleep(0.1)
        return "TIMEOUT"

    def run(self):
        rospy.loginfo("Mission Manager Started")
        time.sleep(2)
        
        state = INIT
        current_room = None
        
        while not rospy.is_shutdown():
            if state == INIT:
                if self.current_room_idx < len(self.rooms):
                    current_room = self.rooms[self.current_room_idx]
                    rospy.loginfo(f"Starting Room: {current_room['name']}")
                    state = GO_TO_ROOM_ENTRY
                else:
                    state = FINISH
            
            elif state == GO_TO_ROOM_ENTRY:
                attempts = 0
                success = False
                while attempts < 2 and not success:  # 2 deneme alır. Çünkü yavaş attempt atıyor.
                    entry = current_room['entry_goal']
                    rospy.loginfo(f"Navigating to Entry (Attempt {attempts+1})...")
                    self.send_goal(entry['x'], entry['y'], entry['yaw'])
                    
                    res = self.wait_for_nav(timeout=90)
                    if res == "SUCCESS":
                        success = True
                    else:
                        attempts += 1
                        rospy.logwarn(f"Navigation Failed: {res}")
                
                if success:
                    state = QR_VERIFY
                else:
                    rospy.logerr(f"Failed to enter room {current_room['name']}")
                    self.report.append(f"{current_room['name']}: FAILED (Entry)")
                    state = NEXT_ROOM

            elif state == QR_VERIFY:
                rospy.loginfo("Verifying QR Code...")
                self.detected_qr = None
                
                start = time.time()
                verified = False
                qr_attempts = 0
                
                # 5 sn boyunca QR ararız. 2 attempt hakkı var.
                
                while qr_attempts < 3 and not verified:
                    rospy.loginfo(f"Looking for QR (Attempt {qr_attempts+1})...")
                    sub_start = time.time()
                    while time.time() - sub_start < 5.0:
                        if self.detected_qr and current_room['qr_expected'] in self.detected_qr:
                            verified = True
                            break
                        if self.detected_qr:
                             # QR bulunur, ama eşleşmezse buraya gelir. Ama bizim sistemde böyle bir şey yok.
                             pass
                        time.sleep(0.1)
                    
                    if verified:
                        break
                    qr_attempts += 1
                
                if verified:
                    rospy.loginfo("Room Verified!")
                    state = EXECUTE_CLEANING
                else:
                    rospy.logwarn("QR Verification Failed or Skipped")
                    self.report.append(f"{current_room['name']}: SKIPPED (QR Fail)")
                    state = NEXT_ROOM

            elif state == EXECUTE_CLEANING:
                rospy.loginfo("Starting Cleaning Routine...")
                goals = current_room.get('cleaning_goals', [])
                all_clean = True
                
                for i, g in enumerate(goals):
                    rospy.loginfo(f"Cleaning Waypoint {i+1}/{len(goals)}")
                    self.send_goal(g['x'], g['y'], g['yaw'])
                    res = self.wait_for_nav(timeout=60)
                    if res != "SUCCESS":
                        rospy.logwarn("Failed to reach cleaning waypoint")
                        all_clean = False
                        # Sonraki noktaya mı geçecek? Yoksa direkt odayı mı iptal edecek?
                        
                status = "CLEANED" if all_clean else "PARTIALLY_CLEANED"
                self.report.append(f"{current_room['name']}: {status}")
                state = NEXT_ROOM

            elif state == NEXT_ROOM:
                self.current_room_idx += 1
                state = INIT
            
            elif state == FINISH:
                rospy.loginfo("Mission Finished!")
                print("\n================ CLEANING REPORT ================")
                for line in self.report:
                    print(line)
                print("=================================================")
                break
                
            time.sleep(0.1)

if __name__ == '__main__':
    try:
        node = MissionManagerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
