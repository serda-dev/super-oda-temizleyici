#!/usr/bin/env python3

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class QRDetectorNode:
    def __init__(self):
        rospy.init_node('qr_detector_node', anonymous=True)
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.qr_pub = rospy.Publisher("/qr_code_result", String, queue_size=10)
        
        # Helper for QR detection
        self.detector = cv2.QRCodeDetector()
        rospy.loginfo("QR Detector Node Initialized")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # Simple detection
        data, bbox, _ = self.detector.detectAndDecode(cv_image)
        
        if data:
            rospy.loginfo(f"QR Code Detected: {data}")
            self.qr_pub.publish(data)
            

if __name__ == '__main__':
    try:
        node = QRDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
