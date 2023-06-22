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
        camera_info = message_filters.Subscriber("/zed2i/zed_node/rgb/camera_info",CameraInfo)
        camera_image = message_filters.Subscriber("/zed2i/zed_node/rgb/image_rect_color",Image)
        message_synchronizer = message_filters.TimeSynchronizer([camera_info,camera_image],queue_size=1)
        message_synchronizer.callbacks(self.image_callback)

        self.cv_server = rospy.Service("/CV_connect/req_cv",CV_srv,self.cv_request)
        self.__open_socket()
    
    def __open_socket(self):
        host = socket.gethostname()

        self.client_yolov8      = CustomSocket(host, 10000)
        self.client_facedetect  = CustomSocket(host, 10001)

        self.client_yolov8.clientConnect()
        self.client_facedetect.clientConnect()

    def image_callback(self, info_msg:CameraInfo, image_msg:Image):
        self.image  = image_msg
        self.info   = info_msg

    def cv_request(self, cv_req:CV_srvRequest):
        error = False
        cv_type = CV_srvRequest.CV_type

        image = self.image

        # return CV_srvResponse(False)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(image,desired_encoding="bgr8")
            pull_image = cv2.resize(cv_image,(1280,736))
        except CvBridgeError as e:
            rospy.logerr(e)
            error = True
        except Exception as e:
            rospy.logerr(e)
            error = True
        



        if cv_type == CV_type.YOLOV8:
            res = self.client_yolov8.req(pull_image)
        elif cv_type == CV_type.FACE_RECONISION:
            pass
        else:
            rospy.logerr("Error CV_type: %s" % (cv_type))
            error = True