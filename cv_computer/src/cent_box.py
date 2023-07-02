import rospy
from cv_connector.srv import CV_srv, CV_srvRequest
from cv_connector.msg import CV_type


class box:

    def __init__(self):
        self.client = rospy.ServiceProxy('/CV_connect/req_cv', CV_srv)

    def compute_center_point(self, obj_shape:)
