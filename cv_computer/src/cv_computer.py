#!/usr/bin/env python3

import rospy
import struct
from sensor_msgs.msg import PointCloud2
from cv_connector.srv import CV_srv, CV_srvRequest
from cv_connector.msg import CV_type
from rospy_message_converter import message_converter
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from ast import literal_eval

class haiyaaaa:
    def __init__(self):
        self.client = rospy.ServiceProxy('/CV_connect/req_cv', CV_srv)
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)

    def markerpub(self, pX, pY, pZ):
        marker = Marker()

        marker.header.frame_id = "zed2i_base_link"
        marker.header.stamp = rospy.Time.now()

        marker.type = 2
        marker.id = 0

        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.position.x = pX
        marker.pose.position.y = pY
        marker.pose.position.z = pZ
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        self.marker_pub.publish(marker)

    def get_dict(self, target = "target"):
        try:
            rospy.wait_for_service('/CV_connect/req_cv')
            req = CV_srvRequest()
            req.cv_type.type = CV_type.TargetTracker_Detect
            response = self.client(req)
            my_dict = literal_eval(response.result)
            mypointcloud = response.pointcloud

        except rospy.ServiceException as e:
            rospy.logerr("Failed to call service: %s", str(e))
            exit(1)

        target_kill_comfirm = False

        for a, dick in my_dict.items():
            if dick['name'] == target:
                x = dick['center'][0]
                y = dick['center'][1]
                target_kill_comfirm = True
        
        if target_kill_comfirm:
            assert isinstance(mypointcloud, PointCloud2)
            index = (y * mypointcloud.row_step) + (x * mypointcloud.point_step)
            mypoint = struct.unpack_from('fff', mypointcloud.data, offset=index)
            x, y, z = mypoint
            self.marker_pub(x, y, z)
            return mypoint
        else:
            return None

if __name__ == "__main__":
    rospy.init_node("cv_comp_track")
    k = haiyaaaa()
    print(k.get_dict())
