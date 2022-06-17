import numpy as np
import cv2

import sys

import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge

points = []
raw_image = None
show_image = None

published = False

def onMouse(event, x, y, flag, params):
  global points
  if event == cv2.EVENT_LBUTTONDOWN:
    points.append([x,y])
    print("add point")
    if len(points) > 2:
      points = []

def process_image(msg):
    global raw_image
    try:
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        img = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
        if(len(img.shape)<3):
          raw_image = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
          raw_image = img
        #cv2.imshow('image', img)
        #cv2.waitKey(1)
    except Exception as err:
        print(err)

rospy.init_node('img_proc')
rospy.loginfo('img_proc node started')

rospy.Subscriber("usb_cam/image_raw", Image, process_image)
pub = rospy.Publisher('cripped_image', Image, queue_size=10)

#raw_image = cv2.imread("capture.png")

wname = "MouseEvent"
cv2.namedWindow(wname)
cv2.setMouseCallback(wname, onMouse, [wname])

while not rospy.is_shutdown():

  if raw_image is not None:

    show_image = raw_image.copy()

    if len(points) == 1:
      cv2.circle(show_image, (points[0][0], points[0][1]), 2, (0, 0, 255), 3)

    if len(points) == 2:
      x1 = points[0][0]
      y1 = points[0][1]
      x2 = points[1][0]
      y2 = points[1][1]
      cv2.rectangle(show_image, (x1, y1), (x2, y2), (255, 0, 0), 3)
      
      if not (x1 == x2 or y1 == y2):
        crip_image = raw_image[y1:y2, x1:x2, :]
    #    cv2.imshow("crip image", crip_image)

        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(crip_image, encoding="bgr8")

        if published == False:
          pub.publish(msg)
          published = True
    
    else:
      if published == True:
        published = False;

    cv2.imshow(wname, show_image)
    cv2.waitKey(1)
