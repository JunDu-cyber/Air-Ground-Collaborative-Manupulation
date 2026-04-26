#!/usr/bin/env python3

import rospy
import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def test_image_publisher():
    # 1. Initialize the ROS Node
    rospy.init_node('test_image_publisher', anonymous=True)
    
    # 2. Create the Publisher (Topic must match your YOLO node exactly)
    pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
    
    # Publish at 10 frames per second
    rate = rospy.Rate(10)
    bridge = CvBridge()

    # 3. Define the path to a test image (Change this to a real photo of a cup/apple!)
    image_path = os.path.expanduser("~/learning_ws/bus.jpg")

    # 4. Load the image, or create a fake one if it doesn't exist
    if os.path.exists(image_path):
        cv_image = cv2.imread(image_path)
        rospy.loginfo(f"Successfully loaded image from: {image_path}")
    else:
        rospy.logwarn(f"Could not find {image_path}. Generating a fake test pattern...")
        # Create a black 640x480 image
        cv_image = np.zeros((480, 640, 3), dtype=np.uint8)
        # Draw a white box to act as a fake object
        cv2.rectangle(cv_image, (200, 150), (440, 330), (255, 255, 255), -1)
        cv2.putText(cv_image, "NO IMAGE FOUND", (210, 240), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    rospy.loginfo("Broadcasting test image on /camera/color/image_raw...")

    # 5. Continuous Publishing Loop
    while not rospy.is_shutdown():
        try:
            # Convert the OpenCV image matrix into a ROS Image message
            ros_msg = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            
            # Broadcast it to the ROS network
            pub.publish(ros_msg)
            
        except Exception as e:
            rospy.logerr(f"CV Bridge Error: {e}")

        rate.sleep()

if __name__ == '__main__':
    try:
        test_image_publisher()
    except rospy.ROSInterruptException:
        pass