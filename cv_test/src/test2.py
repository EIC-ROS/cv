#!/usr/bin/env python 
from tf.transformations import quaternion_from_matrix
import numpy as np
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from cv_connector.srv import CV_srv, CV_srvRequest, CV_srvResponse
from cv_connector.msg import CV_type
from ast import literal_eval
from std_msgs.msg import String
from rospy_message_converter import message_converter
from std_msgs.msg import String
from custom_socket import CustomSocket
import socket
from sensor_msgs.msg import Image



class FixedTFBroadcaster:


    def __init__(self):
        host = socket.gethostname()
        self.client_fr = CustomSocket(host, 12305)
        self.client_fr.clientConnect()

        self.subimg = rospy.Subscriber("/zed2i/zed_node/rgb/image_rect_color", Image,self.cb)
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.client = rospy.ServiceProxy("cv_connector",CV_srv)
        self.sub = rospy.Subscriber("/test/str",String,self.cv_res)
    
    def cv_res(self,msg):
        res = message_converter.convert_ros_message_to_dictionary(msg)
        
        R_dict = {}

        try:
            data = result["data"]
            data = literal_eval(data)
            for id,value in data.items():
                x_vector = np.array(value["normal0"])
                y_vector = np.array(value["normal1"])
                z_vector = np.array(value["normal2"])
                rotation_matrix = np.stack((x_vector,y_vector,z_vector))
                cal = np.transpose(rotation_matrix)
                R_dict.update({id:cal})
        except KeyError:
            pass
        t_list = list()
        print(R_dict)
        for id,value in R_dict.items():
            transform_m = np.eye(4)
            transform_m[:3,:3] = value
            qua = quaternion_from_matrix(transform_m)
            print(qua)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "map"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = str(id)
            t.transform.translation.x = -1
            t.transform.translation.y = 0
            t.transform.translation.z = 0

            t.transform.rotation.x = qua[0]
            t.transform.rotation.y = qua[1]
            t.transform.rotation.z = qua[2]
            t.transform.rotation.w = qua[3]

            t_list.append(t)
            
        tfm = tf2_msgs.msg.TFMessage(t_list)
        self.pub_tf.publish(tfm)

        
        

if __name__ == '__main__':
    rospy.init_node('fixed_tf2_broadcaster')
    tfb = FixedTFBroadcaster()

    rospy.spin()