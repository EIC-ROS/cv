import rospy
from cv_connector.srv import CV_srv, CV_srvRequest
from cv_connector.msg import CV_type
from geometry_msgs.msg import Vector3, Point, Quaternion
import math


class box:

    def __init__(self):
        self.client = rospy.ServiceProxy('/CV_connect/req_cv', CV_srv)

    def compute_center_point(self, obj_shape:Vector3, falt_center_point:Point, box_orientation:Quaternion):
        max_min_angle_z = math.atan2()
