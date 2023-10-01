#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()

        # Create the video capture object
        self.cap = cv2.VideoCapture(0)
        #self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 300)
        #self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 201)

        if not self.cap.isOpened():
            self.get_logger().error('Cannot open video capture')
            exit(1)

        # Set up ArUco dictionary and parameters
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        self.parameters = cv2.aruco.DetectorParameters_create()

        # Create the publisher
        self.publisher_ = self.create_publisher(Image, 'detected_aruco_markers', 10)

        # Set the timer to capture and process video frames
        self.timer = self.create_timer(0.1, self.process_video_frame)

    def process_video_frame(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().error('Error capturing video frame')
            exit(1)

        # Detect ArUco markers in the frame
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, self.dictionary, parameters=self.parameters)

        if ids is not None:
            # Draw detected markers
            detected_markers = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            #detected_markers = aruco_display(corners, ids, rejected, gray_frame)

            '''
            cv2.imshow("Image", detected_markers)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                exit(0)
            '''
        # Convert the frame to an Image message and publish it
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)


    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
