#!/usr/bin/env python

import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo # For camera intrinsic parameters
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import os
import time
import tf
from geometry_msgs.msg import Point, PointArray, PointStamped
from std_msgs.msg import Header
import tf2_ros
import imutils


PLOTS_DIR = os.path.join(os.getcwd(), 'plots')

class ObjectDetector:
   def __init__(self):
      rospy.init_node('object_detector', anonymous=True)

      self.bridge = CvBridge()

      self.cv_color_image = None
      self.cv_depth_image = None

      self.color_image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_image_callback)

      self.tf_listener = tf.TransformListener()  # Create a TransformListener object

      self.point_pub = rospy.Publisher("cup_locations", PointArray, queue_size=10)
      self.image_pub = rospy.Publisher('detected_cups', Image, queue_size=10)

      self.sawyer_bl = [0.905, -0.005]
      self.sawyer_br = [0.888, 0.647]
      self.sawyer_tl = [0.488, -0.005]
      self.sawyer_x = self.sawyer_bl[0] - self.sawyer_tl[0]
      self.sawyer_y = self.sawyer_br[1] - self.sawyer_bl[1]
      self.sawyer_z = -0.099
      # self.sawyer_tr = [0.471, 0.644]

      rospy.spin()

   def color_image_callback(self, msg):
      try:
         # Convert the ROS Image message to an OpenCV image (BGR8 format)
         self.cv_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
         self.process_images()

      except Exception as e:
         print("Error:", e)


   def process_images(self, filename):
      img = cv.imread(filename)
      og_image = img
      cups = []

      # Circle Detection
      cimg = cv.cvtColor(img,cv.COLOR_RGB2BGR)
      #TODO: should this be plain BGR given ending rosbridge conversion??
      img = cv.medianBlur(img,5)
      rows = img.shape[0]
      # circles = cv.HoughCircles(img,cv.HOUGH_GRADIENT,1,20,
      #                             param1=50,param2=30,minRadius=0,maxRadius=0)
      circles = cv.HoughCircles(img,cv.HOUGH_GRADIENT,1,rows/8,
                                 param1=100,param2=30,minRadius=10,maxRadius=50)
      
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

      # Testing
      cv.imshow('detected circles',cimg)
      cv.waitKey(0)
      cv.destroyAllWindows()


      # # If there are no detected points, exit
      # if len(x_coords) == 0 or len(y_coords) == 0:
      #       print("no detected points")
      #       return

      # updated Edge Detection
      img_gray = cv.cvtColor(og_image, cv.COLOR_BGR2GRAY) 
      median = cv.medianBlur(img_gray,5)
      img_blur = cv.GaussianBlur(median, (5,5), 0) 
      edges = cv.Canny(image=img_blur, threshold1=100, threshold2=200) 

      cnts = cv.findContours(edges.copy(), cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
      cnts2 = imutils.grab_contours(cnts)
      c = max(cnts2, key = cv.contourArea)
      r = cv.minAreaRect(c)

      box = cv.boxPoints(r)
      box = np.int0(box)
      print(box)
      # IMPORTANT: Box starts at bottom most point and goes clockwise
      #TODO: for EX IM, starts at bottom right but may need to adjust later

      br = box[0]
      bl = box[1]
      tl = box[2] # if this is not accurate everything is thrown off

      x_pix = abs(bl[0] - tl[0])
      y_pix = abs(br[1] - bl[1])

      # find Sawyer coordinates of cups
      points = []
      for c in cups:
         u, v = c
         # set tl to be offset origin & br to be largest values
         x_diff = self.sawyer_x * ((u - tl[0]) / x_pix)
         y_diff = self.sawyer_y * ((v - tl[1]) / y_pix)
         points.append(Point(tl[0] + x_diff, tl[1] + y_diff, self.sawyer_z))

      # Convert the (X, Y, Z) coordinates from camera frame to odom frame
      try:
            # Publish the transformed point
            self.point_pub.publish(points)
            
            # Convert to ROS Image message and publish
            ros_image = self.bridge.cv2_to_imgmsg(cimg, "bgr8")
            self.image_pub.publish(ros_image)
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("TF Error")
            print(e)
            return

if __name__ == '__main__':
   cam = ObjectDetector()

   cam.process_images("_frame0000.jpg")