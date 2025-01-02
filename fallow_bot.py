from audioop import error
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist


class robot_camera():
    def __init__(self):
        rospy.init_node("fallow_bot_node")
        rospy.loginfo("fallow_bot_node started")
        rospy.Subscriber("camera/rgb/image_raw", Image, self.camera_person)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.speed = Twist()
        self.bridge = CvBridge()
        self.person_found = False
        self.person_position = None
        self.searching = True 
        self.close_distance = 50  
        rospy.spin()

    def camera_person(self, msg):
        self.cap = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.cap = cv2.resize(self.cap, (640, 480))
        cv2.imshow("frame", self.cap)
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        (regions, _) = (hog.detectMultiScale(self.cap,
                                             winStride=(4, 4),
                                             padding=(4, 4),
                                             scale=1.05))
        count = len(regions)
        self.person_found = count > 0
        if self.person_found:
            (x, y, w, h) = regions[0]
            self.person_position = (x + w // 2, y + h // 2)  
            cv2.rectangle(self.cap, (x, y),
                          (x + w, y + h),
                          (0, 255, 0), 2)
            self.searching = False  
        else:
            self.person_position = None

        self.control_robot()
        cv2.imshow("frame", self.cap)
        cv2.waitKey(1)

    def control_robot(self):
        if self.person_found and self.person_position:
            frame_center = self.cap.shape[1] // 2
            x_error = self.person_position[0] - frame_center
            self.speed.angular.z = -0.002 * x_error
            self.speed.linear.x = 0.6
            rospy.loginfo("Finding Person, moving towards them.")
        elif self.searching:
            rospy.loginfo("Searching for a person, spinning.")
            self.speed.linear.x = 0
            self.speed.angular.z = 0.5
        else:   
            rospy.loginfo("finding person")
            self.speed.linear.x = 0
            self.speed.angular.z = 0

        self.pub.publish(self.speed)

if __name__ == "__main__":
    obj = robot_camera()
