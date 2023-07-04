#!/usr/bin/env python3

import rospy
import struct
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
from cv_connector.srv import CV_srv, CV_srvRequest
from cv_connector.msg import CV_type
from ast import literal_eval
import message_filters

class cv_computer:
    def __init__(self):
        self.marker_pub = rospy.Publisher('xyz_irl', Marker, queue_size=2)
        self.cv_client = rospy.ServiceProxy('/CV_connect/req_cv', CV_srv)
        self.save_mypoint = (0,0,0)
        
        
    def xypix_to_xyz(self, x, y, pointcloud):
        index = (y * pointcloud.row_step) + (x * pointcloud.point_step)
        point = struct.unpack_from('fff', pointcloud.data, offset=index)
        return point

    def pub_marker(self, x, y, z, frame, id:int=0):
        mark = Marker()
        mark.header.frame_id = frame
        mark.header.stamp = rospy.Time.now()

        mark.type = 2
        mark.id = id

        mark.scale.x = 0.03
        mark.scale.y = 0.03
        mark.scale.z = 0.03

        mark.color.r = 0.0
        mark.color.g = 1.0
        mark.color.b = 0.0
        mark.color.a = 1.0

        mark.pose.position.x = x
        mark.pose.position.y = y
        mark.pose.position.z = z
        mark.pose.orientation.x = 0.0
        mark.pose.orientation.y = 0.0
        mark.pose.orientation.z = 0.0
        mark.pose.orientation.w = 1.0
        self.marker_pub.publish(mark)
        return True
    
    def xyz_target(self, target = "target"):
        try:
            rospy.wait_for_service('/CV_connect/req_cv')
            req = CV_srvRequest()
            req.cv_type.type = CV_type.TargetTracker_Detect
            response = self.cv_client(req)
            my_dict = literal_eval(response.result)
            mypointcloud = response.pointcloud

        except rospy.ServiceException as e:
            rospy.logerr("Failed to call service: %s", str(e))
            exit(1)

        target_kill_comfirm = False
        my_dict = my_dict["res"]

        for a, dick in my_dict.items():
            if dick['name'] == target:
                x = dick['center'][0]
                y = dick['center'][1]
                target_kill_comfirm = True
        
        if target_kill_comfirm:
            assert isinstance(mypointcloud, PointCloud2)
            index = (y * mypointcloud.row_step) + (x * mypointcloud.point_step)
            try:
                mypoint = struct.unpack_from('fff', mypointcloud.data, offset=index)
                self.save_mypoint = mypoint
            except:
            	mypoint = self.save_mypoint
            
            return mypoint
        else:
            return None
    
if __name__ == "__main__":
    a = cv_computer()
        
