import rospy
import socket
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_connector.msg import CV_type
from cv_connector.srv import CV_srv, CV_srvRequest, CV_srvResponse
from cv_connector.custom_socket import CustomSocket

class CVConnector:

    bridge = CvBridge()

    def __init__(self):

        # rospy.wait_for_message("/zed2i/zed_node/rgb/camera_info",CameraInfo)
        camera_info = message_filters.Subscriber("/zed2i/zed_node/rgb/camera_info",CameraInfo)
        camera_image = message_filters.Subscriber("/zed2i/zed_node/rgb/image_rect_color",Image)
        message_synchronizer = message_filters.TimeSynchronizer([camera_info,camera_image],queue_size=1)
        message_synchronizer.registerCallback(self.image_callback)

        self.cv_server = rospy.Service("/CV_connect/req_cv",CV_srv,self.cv_request)
        self.__open_socket()
    
    def __open_socket(self):
        host = socket.gethostname()

        self.client_yolov8      = CustomSocket(host, 12301)
        self.client_facedetect  = CustomSocket(host, 12304)

        self.client_yolov8.clientConnect()
        self.client_facedetect.clientConnect()

    def image_callback(self, info_msg:CameraInfo, image_msg:Image):
        self.image  = image_msg
        self.info   = info_msg

    def cv_request(self, cv_req:CV_srvRequest):
        error = False
        cv_type = CV_srvRequest().CV_type

        image = self.image
        info  = self.info

        try:
            cv_image = self.bridge.imgmsg_to_cv2(image,desired_encoding="bgr8")
            pull_image = cv2.resize(cv_image,(1280,736))
        except CvBridgeError as e:
            rospy.logerr(e)
            error = True
        except Exception as e:
            rospy.logerr(e)
            error = True
        

        print(CV_type.YoloV8_Tracker)
        print(cv_type)

        if cv_type.type == CV_type.YoloV8_Tracker:
            res = self.client_yolov8.req(pull_image)
        elif cv_type.type == CV_type.Face_Recognition:
            res = self.client_facedetect.req(pull_image)
        else:
            rospy.logerr("Error CV_type: %s" % (cv_type))
            error = True

        print(res)

        srv_res = CV_srvResponse()
        if not error:
            srv_res.sucess = True
            srv_res.result = res
            srv_res.camera_info = info
        
        else:
            srv_res.sucess = False
        
        return srv_res
    
if __name__ == "__main__":
    rospy.init_node("cv_connect_test")
    cv_con = CVConnector()
    rospy.spin()