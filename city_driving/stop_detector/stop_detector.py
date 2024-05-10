import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped
from detector import StopSignDetector

class SignDetector(Node):
    def __init__(self):
        super().__init__("stop_detector")
        self.detector = StopSignDetector() # can edit confidence threshold here if needed
        self.publisher_ = self.create_publisher(Float32, 'stop_area', 10)
        self.subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.callback, 1)
        self.bridge = CvBridge()
        self.stopped = False
        self.curr = None
        # self.stop_time = 10 # in seconds

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.callback)
        self.get_logger().info("Stop Detector Initialized")

    def callback(self, img_msg):
        # Process image with CV Bridge
        image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        # use built-in detector to look for stop sign
        is_stop, bbox = self.detector.predict(image)
        # thresh = 10 # adjust based on area at stopping distance
        area = abs((bbox[2] - bbox[0])*(bbox[3] - bbox[1]))
        # # determine distance threshold for stop sign detection
        # # TODO: extrapolate location from bounding box

        # # if first time detecting a stop sign, stop for 10 seconds
        # if is_stop and area > thresh and not self.stopped:
        #     self.curr = self.get_clock().now().to_msg()
        #     self.temp_stop(self.stop_time)
        #     self.stopped = True
        # # if no longer detecting a stop sign, unpause detection
        # if not is_stop and self.stopped:
        #     self.stopped = False

        msg = Float32()
        msg.data = area
        if is_stop:
            self.publisher_.publish(msg)
        
    # def temp_stop(self, dur):
    #     now = self.get_clock().now().to_msg()
    #     while now.sec < self.curr.sec + dur:
    #         stopper = AckermannDriveStamped()
    #         stopper.header.stamp = now
    #         stopper.header.frame_id = "base_link" #'map'
    #         self.publisher.publish(stopper)
    #         now = self.get_clock().now().to_msg()

def main(args=None):
    rclpy.init(args=args)
    detector = SignDetector()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()