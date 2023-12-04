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
from geometry_msgs.msg import Point, PointStamped, PoseArray, Quaternion, Pose
from std_msgs.msg import Header
import tf2_ros
# from msg import PointArray


PLOTS_DIR = os.path.join(os.getcwd(), 'plots')

# class ObjectDetector:
#    def __init__(self):
#       rospy.init_node('object_detector', anonymous=True)

#       self.bridge = CvBridge()

#       self.cv_color_image = None
#       self.cv_depth_image = None

#       self.color_image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_image_callback)
#       self.depth_image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)

#       self.fx = None
#       self.fy = None
#       self.cx = None
#       self.cy = None

#       self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)

#       self.tf_listener = tf.TransformListener()  # Create a TransformListener object

#       self.point_pub = rospy.Publisher("goal_point", Point, queue_size=10)
#       # self.image_pub = rospy.Publisher("detected_cup", Image, queue_size=10)
#       self.image_pub = rospy.Publisher("detected_cup", Point, queue_size=10)
#       self.image_pub.publish([Point(0.49, 0.624, -0.099)])

#       rospy.spin()

#    def camera_info_callback(self, msg):
#       # TODO: Extract the intrinsic parameters from the CameraInfo message
#       K = msg.K
#       self.fx = K[0]
#       self.fy = K[4]
#       self.cx = K[2]
#       self.cy = K[5]

#    def lookup_tag(tag_number):
#       """
#       Given an AR tag number, this returns the position of the AR tag in the robot's base frame.
#       You can use either this function or try starting the scripts/tag_pub.py script.  More info
#       about that script is in that file.  

#       Parameters
#       ----------
#       tag_number : int

#       Returns
#       -------
#       3x' :obj:`numpy.ndarray`
#          tag position
#       """
#       tfBuffer = tf2_ros.Buffer()
#       tfListener = tf2_ros.TransformListener(tfBuffer) 

#       try:
#          source = 'base'
#          goal = f'ar_marker_{tag_number}'
#          trans = tfBuffer.lookup_transform(source, goal, rospy.Time(0), rospy.Duration(10.0))
#       except Exception as e:
#          print(e)
#          print("Retrying ...")

#       tag_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
#       return np.array(tag_pos)

#    def pixel_to_point(self, u, v, depth):
#       # TODO: Use the camera intrinsics to convert pixel coordinates to real-world coordinates
#       X = (u - self.cx) * depth / self.fx
#       Y = (v - self.cy) * depth / self.fy
#       Z = depth
#       return X, Y, Z
   
#    def pixel_to_table(self, u, v):
#       X = ...
#       Y = ...

#    def color_image_callback(self, msg):
#       try:
#          # Convert the ROS Image message to an OpenCV image (BGR8 format)
#          self.cv_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

#          # If we have both color and depth images, process them
#          if self.cv_depth_image is not None:
#                self.process_images()

#       except Exception as e:
#          print("Error:", e)

#    def depth_image_callback(self, msg):
#       try:
#          # Convert the ROS Image message to an OpenCV image (16UC1 format)
#          self.cv_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

#       except Exception as e:
#          print("Error:", e)

#    def process_images(self, ar_tag_locations, im):
#       test_img = 'test/cups.jpg'
#       pickup_length = 32.5 
#       pyramid_length = 26.5
#       pickup_width = 17.5

#       # TO-DO: find locations of AR tags
#       cups = []
#       # need 4 point correspondences --> feed in ar tag markers --> square coordinates in pixel dimensions
#       output = [[0,0], [], [], []] # define based on relative ar tag locations - ORDER MATTERS HERE
#       maxWidth = int(pickup_width * 36)
#       maxHeight = int(pickup_length * 36) # define based on relative length of table region
#       H = cv.getPerspectiveTransform(ar_tag_locations, output) 
#       rectified_img = cv.warpPerspective(im, H, (maxWidth, maxHeight), flags=cv.INTER_LINEAR)

#       # Q: what does this do
#       img = rectified_img # find circles based on straightened out image
#       # cap = cv.VideoCapture(0)

#       img = cv.medianBlur(img,5)
#       circles = cv.HoughCircles(img,cv.HOUGH_GRADIENT,1,20,
#                                  param1=50,param2=30,minRadius=0,maxRadius=0)
#       circles = np.uint16(np.around(circles))
#       for i in circles[0,:]:
#          center_x = i[0]
#          center_y = i[1]
#          # note that this is in pixels
#          # color filtering would go here
#          cups.append([center_x, center_y])
      
#       # # Convert the color image to HSV color space
#       # hsv = cv.cvtColor(self.cv_color_image, cv.COLOR_BGR2HSV)
#       # print(hsv[(len(hsv) // 2)])
#       # # TODO: Define range for cup color in RGB
#       # # NOTE: You can visualize how this is performing by viewing the result of the segmentation in rviz
#       # # 140 50 50 160 255 255
#       # # Convert RGB thresholds to HSV
#       # lower_hsv = np.array([80, 20, 120])
#       # upper_hsv = np.array([180, 120, 255])
#       # # TODO: Threshold the image to get only cup colors
#       # # HINT: Lookup cv.inRange() or np.where()
#       # mask = cv.inRange(self.cv_color_image, lower_hsv, upper_hsv)
#       # # TODO: Get the coordinates of the cup points on the mask
#       # # HINT: Lookup np.where() or np.nonzero()
#       # y_coords, x_coords = np.nonzero(mask)
#       # # If there are no detected points, exit
#       # if len(x_coords) == 0 or len(y_coords) == 0:
#       #       print("no detected points")
#       #       return
#       # # Calculate the center of the detected region by 
#       # center_x = int(np.mean(x_coords))
#       # center_y = int(np.mean(y_coords))

#       # Fetch the depth value at the center
#       depth = self.cv_depth_image[center_y, center_x]

#       if self.fx and self.fy and self.cx and self.cy:
#          camera_x, camera_y, camera_z = self.pixel_to_point(center_x, center_y, depth)
#          camera_link_x, camera_link_y, camera_link_z = camera_z, -camera_x, -camera_y
#          # Convert from mm to m
#          camera_link_x /= 1000
#          camera_link_y /= 1000
#          camera_link_z /= 1000

#          # Convert the (X, Y, Z) coordinates from camera frame to odom frame
#          try:
#                self.tf_listener.waitForTransform("/odom", "/camera_link", rospy.Time(), rospy.Duration(10.0))
#                point_odom = self.tf_listener.transformPoint("/odom", PointStamped(header=Header(stamp=rospy.Time(), frame_id="/camera_link"), point=Point(camera_link_x, camera_link_y, camera_link_z)))
#                X_odom, Y_odom, Z_odom = point_odom.point.x, point_odom.point.y, point_odom.point.z
#                print("Real-world coordinates in odom frame: (X, Y, Z) = ({:.2f}m, {:.2f}m, {:.2f}m)".format(X_odom, Y_odom, Z_odom))

#                if X_odom < 0.001 and X_odom > -0.001:
#                   print("Erroneous goal point, not publishing")
#                else:
#                   print("Publishing goal point: ", X_odom, Y_odom, Z_odom)
#                   # Publish the transformed point
#                   self.point_pub.publish(Point(X_odom, Y_odom, Z_odom))

#                   # Overlay cup points on color image for visualization
#                   cup_img = self.cv_color_image.copy()
#                   cup_img[y_coords, x_coords] = [0, 0, 255]  # Highlight cup points in red
#                   cv.circle(cup_img, (center_x, center_y), 5, [0, 255, 0], -1)  # Draw green circle at center
                  
#                   # Convert to ROS Image message and publish
#                   ros_image = self.bridge.cv2_to_imgmsg(cup_img, "bgr8")
#                   self.image_pub.publish(ros_image)
#          except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
#                print("TF Error")
#                print(e)
#                return

if __name__ == '__main__':
   # ObjectDetector()
   rospy.init_node('camera_node')

   point_pub = rospy.Publisher("cup_locations", PoseArray, queue_size=10)
   try:
      neg_z = -0.099
      quat = [0.0, 1.0, 0.0, 0.0]
      pose_arr = PoseArray()
      pose = Pose(Point(0.49, 0.624, neg_z), Quaternion(quat[0], quat[1], quat[2], quat[3]))
      pose_arr.poses = [pose]

      r = rospy.Rate(10)
      # Publish the transformed point
      while not rospy.is_shutdown():
         point_pub.publish(pose_arr)
         r.sleep()
         
   except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
      print(e)
