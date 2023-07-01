import rospy

def call_service():
    rospy.wait_for_service('/CV_connect/req_cv')
    cv_service = rospy.ServiceProxy('/CV_connect/req_cv', 1)

    print(cv_service)

if __name__ == "__main__":
    call_service()