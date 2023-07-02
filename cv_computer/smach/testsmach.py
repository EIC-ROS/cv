#!/usr/bin/env python3

import rospy
import struct
import smach
from cv_connector.srv import CV_srv, CV_srvRequest
import smach_ros
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from rospy_message_converter import message_converter

class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.client = rospy.ServiceProxy('/CV_connect/req_cv', CV_srv)
        self.srv = CV_srv()

    def execute(self, target = 'target'):
        while True:
            try:
                rospy.wait_for_service('/CV_connect/req_cv')
                response = self.client(self.srv)
                result = String(response.result)
                mydict = message_converter.convert_ros_message_to_dictionary(result)
                mypointcloud = response.pointcloud
                
            except rospy.ServiceException as e:
                rospy.logerr("Failed to call service: %s", str(e))
                exit(1)
                
            for a, b in mydict.items():
                if b['name'] == target:
                    x = b['center'][0]
                    y = b['center'][1]
                    
            assert isinstance(mypointcloud, PointCloud2)
            index = (y * mypointcloud.row_step) + (x * mypointcloud.point_step)
            (X, Y, Z) = struct.unpack_from('fff', mypointcloud.data, offset=index)
            print(X, Y, Z)
        return 'outcome1'
        
def main():
    rospy.init_node('smach_example_state_machine')
    sm = smach.StateMachine(outcomes=['outcome4'])
    with sm:
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'FOO', 'outcome2':'outcome4'})

    outcome = sm.execute()

if __name__ == '__main__':
    main()