#!/usr/bin/env python3
"""
Test client for the /yolo/detect service.
Loads an image, sends it once, and prints detection results.

Usage:
  rosrun mobile_manipulator test_yolo_client.py
  rosrun mobile_manipulator test_yolo_client.py --image /path/to/image.jpg
"""
import rospy
import cv2
import sys
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from mobile_manipulator.srv import DetectObjects, DetectObjectsRequest


def main():
    parser = argparse.ArgumentParser(description="YOLO detection service test client")
    parser.add_argument("--image", default="~/learning_ws/bus.jpg",
                        help="Path to test image")
    args, _ = parser.parse_known_args()

    rospy.init_node("test_yolo_client", anonymous=True)
    bridge = CvBridge()

    # Load the image
    import os
    image_path = os.path.expanduser(args.image)
    cv_image = cv2.imread(image_path)
    if cv_image is None:
        rospy.logerr(f"Cannot load image: {image_path}")
        return

    rospy.loginfo(f"Loaded image: {image_path} ({cv_image.shape[1]}x{cv_image.shape[0]})")

    # Wait for the service to be available
    rospy.loginfo("Waiting for /yolo/detect service...")
    rospy.wait_for_service("/yolo/detect")
    detect = rospy.ServiceProxy("/yolo/detect", DetectObjects)

    # Build request
    req = DetectObjectsRequest()
    req.image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

    # Call the service
    rospy.loginfo("Calling /yolo/detect...")
    res = detect(req)

    # Print results
    rospy.loginfo(f"Inference time: {res.inference_time_ms:.1f} ms")
    rospy.loginfo(f"Detected {len(res.detections)} objects:")
    for det in res.detections:
        rospy.loginfo(f"  [{det.class_name}] conf={det.confidence:.2f} "
                      f"box=({det.x},{det.y},{det.width},{det.height}) "
                      f"center=({det.center_u},{det.center_v})")


if __name__ == "__main__":
    main()
