#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Point , Point32, Polygon
from cv_bridge import CvBridge
import cv2
import math

#point_x_ = 50
#point_y_ = 50
#angle_ = 0

points_ = []

edges_x_ = []
edges_y_ = []

def rotate(point, angle):
    rad = 2 * math.pi * angle / 360
    x = point[0]
    y = point[1]

    rx = math.cos(rad) * x - math.sin(rad) * y
    ry = math.sin(rad) * x + math.cos(rad) * y

    rp = [rx, ry]

    return rp

def process_image(msg):
    global point_x_, point_y_
    global edges_x, edges_y
    global points_
    #try:
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    #img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    points = points_
    print(len(points))
    for i in range(len(points)):
      point_x_ = points[i].x
      point_y_ = points[i].y
      angle_ = points[i].z
      cv2.drawMarker(
        img, (int(point_x_), int(point_y_)), color=(0, 0, 255), markerType=cv2.MARKER_CROSS, thickness=1
      )

      edges_x = edges_x_
      edges_y = edges_y_
      for j in range(len(edges_x)):

        edge = [ edges_x[j], edges_y[j] ]
        re = rotate(edge, -angle_)
        center = (int(re[0]) + int(point_x_), int(re[1]) + int(point_y_))
        radius = 3
        color = (0, 255, 0)
        cv2.circle(img, center, radius, color, thickness=1, lineType=cv2.LINE_8, shift=0)

    cv2.imshow('ros_image_viewer', img)
    cv2.waitKey(1)
    #except Exception as err:
    #    print(err)

def point_callback(data):
    global points_
    points_ = data.points

def edges_callback(msg):
    print("edges callback")
    global edges_x_, edges_y_
    edges_x_ = []
    edges_y_ = []
    points = msg.points
    for i in range(len(points)):
      edges_x_.append(points[i].x)
      edges_y_.append(points[i].y)
  
def start_node():
    rospy.init_node('img_proc')
    rospy.loginfo('img_proc node started')
    rospy.Subscriber("/usb_cam/image_raw", Image, process_image)
    rospy.Subscriber("/matching/result", Polygon, point_callback)
    rospy.Subscriber("/matching/edges", Polygon, edges_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
