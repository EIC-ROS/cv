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


class FixedTFBroadcaster:

    test = [(1-0.25/2**0.5,-0.25/2**0.5,0,"a"),
            (1-0.25/2**0.5,0.25/2**0.5,0,"b")]
    
    R = np.array([[0.7071,-0.7071,0],
             [0.7071,0.7071,0],
             [0,0,1]])

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.client = rospy.ServiceProxy("cv_connector",CV_srv)
        transform_m = np.eye(4)
        transform_m[:3,:3] = self.R
        qua = quaternion_from_matrix(transform_m)
        t_list = []

        for i in self.test:
            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "panda_link0"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = i[3]
            t.transform.translation.x = i[0]
            t.transform.translation.y = i[1]
            t.transform.translation.z = i[2]

            t.transform.rotation.x = qua[0]
            t.transform.rotation.y = qua[1]
            t.transform.rotation.z = qua[2]
            t.transform.rotation.w = qua[3]
            t_list.append(t)

        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            rospy.sleep(0.1)
            for t in t_list:
                t.header.stamp = rospy.Time.now()

            

            tfm = tf2_msgs.msg.TFMessage(t_list)
            self.pub_tf.publish(tfm)
    
    def req_cv(self):
        # req = CV_srvRequest()
        # req.cv_type.type = CV_type.
        # res = self.client.call(req)
    
    def make_tf



if __name__ == '__main__':
    rospy.init_node('fixed_tf2_broadcaster')
    tfb = FixedTFBroadcaster()

    rospy.spin()