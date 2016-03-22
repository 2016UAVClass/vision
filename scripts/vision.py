from __future__ import print_function
import sys
import rospy
import cv2
import math
import PID
import roslib
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String


roslib.load_manifest('vision')


def nothing(x):
    return


class ImageConverter:

    def __init__(self):
        # self.image_pub = rospy.Publisher("cv2_processed_image", Image,queue_size=0)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/uav_camera_down/image_raw",Image,self.callback)
        self.blob_detect_config = cv2.SimpleBlobDetector_Params()
        self.blob_detect_config.filterByArea = True
        self.blob_detect_config.minArea = 200
        self.blob_detect_config.maxArea = 1000000

        # Filter by Circularity
        self.blob_detect_config.filterByCircularity = True
        self.blob_detect_config.minCircularity = 0.3

        # Filter by Convexity
        self.blob_detect_config.filterByConvexity = False
        self.blob_detect_config.minConvexity = 0.87

        # Filter by Inertia
        self.blob_detect_config.filterByInertia = False
        self.blob_detect_config.minInertiaRatio = 0.01

        self.blob_detect = cv2.SimpleBlobDetector(self.blob_detect_config)

        self.pid_controller = PID.PID()
        self.pid_controller.set_kp(0)
        self.pid_controller.set_ki(0)
        self.pid_controller.set_accumulated_error_max(1000)
        self.pid_controller.set_kd(1)

        cv2.namedWindow('frame')
        cv2.createTrackbar('Thresh_Min', 'frame', 0, 255, nothing)

    def callback(self,data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        # Take each frame
        height, width, _ = frame.shape
        # Convert BGR to HSV
        frame_ycrcb = cv2.cvtColor(frame, cv2.COLOR_BGR2YCR_CB)
        frame_cr = cv2.extractChannel(frame_ycrcb, 1)

        _, frame_cr_highlights = cv2.threshold(frame_cr,cv2.getTrackbarPos('Thresh_Min', 'frame'),255, cv2.THRESH_BINARY)
        # frame_cr_highlights = cv2.adaptiveThreshold(frame_cr,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,11,2)
        # Threshold the HSV image to get only blue colors
        # mask = cv2.inRange(hsv, lower_green, upper_green)

        keypoints = self.blob_detect.detect(frame_cr_highlights)

        im_with_keypoints = cv2.drawKeypoints(frame_cr_highlights, keypoints, np.array([]), (0, 255, 0),
                                              cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        if len(keypoints) > 0:
            center = (width/2, height/2)
            target = (int(keypoints[0].pt[0]), int(keypoints[0].pt[1]))
            distance = math.sqrt((center[0] - target[0])**2 + (center[1] - target[1])**2)

            # print(self.pid_controller.out(distance))

            cv2.line(im_with_keypoints, center, target, (0, 255-distance, distance), 5)

        cv2.imshow('frame', im_with_keypoints)
        cv2.waitKey(1)


def main(args):
    rospy.init_node('vision', anonymous=True)
    ic = ImageConverter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

