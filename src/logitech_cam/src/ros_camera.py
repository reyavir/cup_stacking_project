#!/usr/bin/env python

import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo # For camera intrinsic parameters
from logitech_cam.srv import PositionSrv 
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import os
import time
import tf
from geometry_msgs.msg import Point, PoseArray, PointStamped, Pose, Quaternion
from std_msgs.msg import Header
import tf2_ros
import imutils


PLOTS_DIR = os.path.join(os.getcwd(), 'plots')

class ObjectDetector:

   def __init__(self):
      rospy.init_node('object_detector', anonymous=True)
      self.poses = []

      self.bridge = CvBridge()

      self.cv_color_image = None
      self.cv_depth_image = None

      print("attempting to subscribe to usb_cam")
      self.color_image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.color_image_callback)

      self.tf_listener = tf.TransformListener()  # Create a TransformListener object

      # self.point_pub = rospy.Publisher("cup_locations", PoseArray, queue_size=10)
      self.point_service = rospy.Service("cup_locations", PositionSrv, self.point_srv_callback)
      self.image_pub = rospy.Publisher('detected_cups', Image, queue_size=10)

      self.sawyer_bl = [0.871, 0.046]
      self.sawyer_tr = [0.444, 0.737]
      self.sawyer_tl = [0.487, 0.046]
      self.sawyer_x = self.sawyer_bl[0] - self.sawyer_tl[0]
      self.sawyer_y = self.sawyer_tr[1] - self.sawyer_bl[1]
      self.sawyer_z = -0.099
      # self.sawyer_tr = [0.471, 0.644]

      rospy.spin()

   def point_srv_callback(self, request):
      return self.poses

   def color_image_callback(self, msg):
      try:
         print("entering color_image_callback")
         # Convert the ROS Image message to an OpenCV image (BGR8 format)
         self.cv_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
         self.process_images("homography.jpg")

      except Exception as e:
         print("Error:", e)


   def process_images(self, filename):
      print("attempting to process image")
      # img = self.cv_color_image
      img = cv.imread(filename)
      img = img.astype(np.uint8)
      og_image = img
      cups = []
      cimg = cv.cvtColor(img,cv.COLOR_RGB2BGR)

      img_gray = cv.cvtColor(og_image, cv.COLOR_BGR2GRAY) 
      img_hsv = cv.cvtColor(og_image, cv.COLOR_BGR2HSV)
      lower_hsv = np.array([100, 150, 100])
      upper_hsv = np.array([200, 255, 255])

      mask = cv.inRange(img_hsv, lower_hsv, upper_hsv)
      y, x = np.nonzero(mask)

      cont, _ = cv.findCountours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
      for c in cont:
         m = cv.moments(c)
         if m["m00"] != 0:
            c_x = int(m["m10"] / m["m00"])
            c_y = int(m["m01"] / m["m00"])
            cv.circle(cimg, (c_x, c_y), 5, (255, 0, 0), -1)

      # Circle Detection
      img = cv.medianBlur(img,5)
      rows = img.shape[0]
      # circles = cv.HoughCircles(img,cv.HOUGH_GRADIENT,1,20,
      #                             param1=50,param2=30,minRadius=0,maxRadius=0)
      print("calc hough circles")
      circles = cv.HoughCircles(img_gray,cv.HOUGH_GRADIENT,1,rows/8,
                                 param1=100,param2=30,minRadius=10,maxRadius=50)
      print("circles detected ")
      # Center Detection
      if circles is not None:
         circles = np.uint16(np.around(circles))
         for i in circles[0,:]:
            center_x = round(i[0])
            center_y = round(i[1])
            # note that this is in pixels
            # color filtering would go here
            cups.append([center_x, center_y])
            # draw the outer circle
            cv.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
         print(str(len(cups)) + " circles detected")

      # # If there are no detected points, exit
      # if len(x_coords) == 0 or len(y_coords) == 0:
      #       print("no detected points")
      #       return

      # updated Edge Detection
      median = cv.medianBlur(img_gray,5)
      img_blur = cv.GaussianBlur(median, (5,5), 0) 
      edges = cv.Canny(image=img_blur, threshold1=100, threshold2=200) 

      cnts = cv.findContours(edges.copy(), cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
      cnts2 = imutils.grab_contours(cnts)
      c = max(cnts2, key = cv.contourArea)
      print("contour")
      print(len(c))
      r = cv.minAreaRect(c)
      hull = np.array(cv.convexHull(c))

      box = cv.boxPoints(r)
      box = np.int0(box)
      print(box)

      first = hull[:, 0, 0]
      sec = hull[:, 0, 1]
      cv.drawContours(cimg, [box], 0, (255, 0, 0), 2)
      max_x, min_x = max(first), min(first)
      max_y, min_y = max(sec), min(sec)
      # cv.circle(cimg,(max_x,min_y),2,(255,255,255),3)
      cv.circle(cimg,(min_x,max_y),2,(255,255,255),3)

      # print("hull")
      # print(hull)
      # print("hull")
      # IMPORTANT: Box starts at bottom most point and goes clockwise
      #TODO: for EX IM, starts at bottom right but may need to adjust later

      br = box[0]
      bl = box[1]
      tl = box[2] # if this is not accurate everything is thrown off

      x_pix = 1173 # abs(bl[0] - tl[0])
      y_pix = 720 # abs(br[1] - bl[1])

      # find Sawyer coordinates of cups
      quat = [0.0, 1.0, 0.0, 0.0]
      pose_arr = PoseArray()
      for c in cups:
         u, v = c
         # set tl to be offset origin & br to be largest values
         # x_diff = self.sawyer_x * ((u - tl[0]) / x_pix)
         # y_diff = self.sawyer_y * ((v - tl[1]) / y_pix)
         x_diff = self.sawyer_x * (v / y_pix) # sawyer x is vertical
         y_diff = self.sawyer_y * (u / x_pix) # sawyer x is horizontal
         pose = Pose(Point(self.sawyer_tl[0] + x_diff, self.sawyer_tl[1] + y_diff, self.sawyer_z),
                      Quaternion(quat[0], quat[1], quat[2], quat[3]))
         pose_arr.poses.append(pose)
      self.poses = pose_arr.poses
      print("PoseArray: ", pose_arr)

      # Convert the (X, Y, Z) coordinates from camera frame to odom frame
      try:
            r = rospy.Rate(10)
            # Publish the transformed point
            while not rospy.is_shutdown():
               #Publish the transformed point
               # self.point_pub.publish(pose_arr)
               self.poses = pose_arr
               
               # Convert to ROS Image message and publish
               ros_image = self.bridge.cv2_to_imgmsg(cimg, "bgr8")
               self.image_pub.publish(ros_image)

               r.sleep()
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("TF Error")
            print(e)
            return

if __name__ == '__main__':
   cam = ObjectDetector()

   # cam.process_images()