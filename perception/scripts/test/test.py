#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image:

  def __init__(self):
    self.image_pub = rospy.Publisher("/detected_image",Image, queue_size=1)
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

  def crop_save_img(self, filename, x1, y1, x2, y2, pad = 15):
    x1 ,y1 = x1 - pad, y1 - pad
    x2, y2 = x2 + pad, y2 + pad
    crop_img = self.cv_image[y1:y2, x1:x2]
    cv2.imwrite(filename, crop_img)

  def show_image(self):
    cv2.imshow("Image Window", self.cv_image)
    cv2.waitKey(3)
    #cv2.destroyAllWindows()

if __name__ == '__main__':
  
  rospy.init_node('image_converter', anonymous=True)
  rate = rospy.Rate(10)

  ic = image_converter()
  ic.capture_image()
  #ic.draw_rectangle(160, 141, 233, 275)
  ic.publish_image()
  ic.show_image()
  count = 2
  image_name = "glue/" + str(count) + ".jpg"
  ic.crop_save_img(image_name,160, 141, 233, 275 )

  # while not rospy.is_shutdown():
  #   ic.capture_image()
  #   ic.publish_image()
  #   rate.sleep()
 
