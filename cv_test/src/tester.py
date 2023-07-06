#!/usr/bin/env python3

import rospy
from cv_connector.srv import CV_srv, CV_srvRequest
from cv_connector.msg import CV_type

if __name__ == "__main__":
    rospy.init_node('testerasgrhwetuilh')
    cv_client = rospy.ServiceProxy('/CV_connect/req_cv', CV_srv)
    
    while not rospy.is_shutdown():
        try:
            rospy.wait_for_service('/CV_connect/req_cv')
            req = CV_srvRequest()
            a = input("0-6 by CV_type")
            print("User input : "+str(a))
            if str(a) == "0":
                req.cv_type.type = CV_type.YoloV8_Tracker
            elif str(a) == "1":
                req.cv_type.type = CV_type.TargetTracker_Register
            elif str(a) == "2":
                req.cv_type.type = CV_type.TargetTracker_Detect
            elif str(a) == "3":
                req.cv_type.type = CV_type.PoseCaptioning
            elif str(a) == "4":
                req.cv_type.type = CV_type.BAE
            elif str(a) == "5":
                req.cv_type.type = CV_type.VQA
            elif str(a) == "6":
                req.cv_type.type = CV_type.HumanPoseEstimation
                
            b = input("request info = ")
            req.req_info = str(b)
            response = cv_client(req)
            print(response.result)

        except rospy.ServiceException as e:
            rospy.logerr("Failed to call service: %s", str(e))
            exit(1)
    rospy.spin()
        
# uint8 YoloV8_Tracker = 0
# uint8 TargetTracker_Register = 1
# uint8 TargetTracker_Detect = 2
# uint8 PoseCaptioning = 3
# uint8 BAE = 4
# uint8 VQA = 5
# uint8 HumanPoseEstimation = 6