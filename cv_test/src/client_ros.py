#!/usr/bin/env python3
import rospy
import socket
import cv2
import numpy as np
import time
from custom_socket import CustomSocket
import json
import cv_bridge
import sensor_msgs.msg
import std_msgs.msg
import rospkg
from rospy_message_converter import message_converter
from std_msgs.msg import String

rospy.init_node("test")
ip= rospy.Publisher("/test/img",sensor_msgs.msg.Image,queue_size=1)
sp= rospy.Publisher("/test/str",std_msgs.msg.String,queue_size=1)

print("wdwdwdw")
bridge = cv_bridge.CvBridge()
def list_available_cam(max_n):
    list_cam = []
    for n in range(max_n):
        cap = cv2.VideoCapture(n)
        ret, _ = cap.read()

        if ret:
            list_cam.append(n)
        cap.release()

    if len(list_cam) == 1:
        return list_cam[0]
    else:
        print(list_cam)
        return int(input("Cam index: "))


host = socket.gethostname()
port = 12305

c = CustomSocket(host, port)

c.clientConnect()


# cap = cv2.VideoCapture(list_available_cam(10))
cap = cv2.VideoCapture("/home/nitro20/walkie_ws/src/cv/cv_test/src/dof_test_2.mp4")
cap.set(4, 480)
cap.set(3, 640)
print("fsd")
while cap.isOpened():
    print("dwdwdjhajhfkjdsbh")

    ret, frame = cap.read()
    if not ret:
        print("Ignoring empty camera frame.")
        continue

    img = bridge.cv2_to_imgmsg(frame,"bgr8")

    # cv2.imshow('client_cam', frame)

    msg = c.req(frame)

    ip.publish(img)
    message = String(str(msg))
    # message = message_converter.convert_dictionary_to_ros_message('std_msgs/String', msg)
    sp.publish(message)

    
    

    print(msg)

    key = cv2.waitKey(1)
    # print(key)

    if key == ord("q"):
        cap.release()
    if key == ord('p'):
        cv2.waitKey()

cv2.destroyAllWindows()
rospy.spin()