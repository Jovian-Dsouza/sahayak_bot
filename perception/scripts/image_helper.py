#!/usr/bin/env python

# helper module for image manipulation and visulization
# Defines a class ImagePub that is used for publishing to /detected_image topic
# Also defines a class image which was used to generate training data

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImagePub:

  def __init__(self):
    self.image_pub = rospy.Publisher("/detected_image",Image, queue_size=1)
    # self.image_sub = rospy.Subscriber('/camera/color/image_raw2', Image, self.capture_image)
    self.bridge = CvBridge()
    
  def capture_image(self):
    ros_image = rospy.wait_for_message('/camera/color/image_raw2', Image)
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)
  
  def publish_image(self):
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
    except CvBridgeError as e:
      rospy.logerr(e)
  
  def draw_rectangle(self, x1, y1, x2, y2, pad = 15):
    x1 ,y1 = x1 - pad, y1 - pad
    x2, y2 = x2 + pad, y2 + pad
    self.cv_image = cv2.rectangle(self.cv_image, (x1, y1) , (x2, y2), (255, 0, 0), 2)
    
  def draw_rectangle(self, rect, pad = 15):
    x1 = rect[0] - pad
    y1 = rect[1] - pad
    x2 = rect[2] + pad
    y2 = rect[3] + pad
    self.cv_image = cv2.rectangle(self.cv_image, (x1, y1) , (x2, y2), (255, 0, 0), 2)

  def draw_rectangle_with_label(self, rect, label, pad = 15):
    x1 = rect[0] - pad
    y1 = rect[1] - pad
    x2 = rect[2] + pad
    y2 = rect[3] + pad
    self.cv_image = cv2.rectangle(self.cv_image, (x1, y1) , (x2, y2), (255, 0, 0), 2)
    cv2.putText(self.cv_image, label, (x1, y1-10), cv2.FONT_HERSHEY_TRIPLEX \
      , 0.9, (36, 255, 12))

  def crop_save_img(self, filename, x1, y1, x2, y2, pad = 15):
    x1 ,y1 = x1 - pad, y1 - pad
    x2, y2 = x2 + pad, y2 + pad
    crop_img = self.cv_image[y1:y2, x1:x2]
    cv2.imwrite(filename, crop_img)

  def check_bounds(self, x1, y1 , x2, y2):
    self.rows, self.cols, self.channels = self.cv_image.shape
    
    if 0 <= y1 < self.rows and 0 <= y2 < self.rows \
      and 0 <= x1 < self.cols and 0 <= x2 < self.cols:
      return True

    rospy.logerr("Out of bounds")
    return False 


  def crop_save_img(self, filename, rect , pad = 15):
    x1 = rect[0] - pad
    y1 = rect[1] - pad
    x2 = rect[2] + pad
    y2 = rect[3] + pad

    if(self.check_bounds(x1, y1 , x2, y2) == False):
      return False

    crop_img = self.cv_image[y1:y2, x1:x2]
    cv2.imwrite(filename, crop_img)
    rospy.loginfo("Saved File : " + filename)
    return True

  def show_image(self):
    cv2.imshow("Image Window", self.cv_image)
    cv2.waitKey(3)
    #cv2.destroyAllWindows()

class image(ImagePub): #Class used only for training
  def __init__(self):
    self.image_pub = rospy.Publisher("/detected_image",Image, queue_size=1)
    self.image_sub = rospy.Subscriber('/camera/color/image_raw2', Image, self.capture_image)
    self.bridge = CvBridge()
    
  def capture_image(self, ros_image):
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)
