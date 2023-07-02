import rospy
import socket
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import cv2
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_connector.msg import CV_type
from cv_connector.srv import CV_srv, CV_srvRequest, CV_srvResponse
from cv_connector.custom_socket import CustomSocket
import numpy as np
from munch import Munch
import yaml
from rospkg import RosPack


class CVConnector:

    with open(RosPack().get_path("cv_connector") + "/cfg/cv_info.yaml", "r") as doc:
        cv_info = Munch(yaml.safe_load(doc))

    bridge = CvBridge()

    def __init__(self):

        self.__subscribing_cameara()
        self.cv_server = rospy.Service(
            "/CV_connect/req_cv", CV_srv, self.cv_request)
        self.__open_socket()

        # PEFR
        self.name_map = dict()
        self.frame_count = 0
        self.frame_count_interval = 10

    def __subscribing_cameara(self):
        rospy.loginfo("Waiting for camera info...")
        rospy.wait_for_message("/zed2i/zed_node/rgb/camera_info", CameraInfo)
        camera_info = message_filters.Subscriber(
            "/zed2i/zed_node/rgb/camera_info", CameraInfo)
        camera_image = message_filters.Subscriber(
            "/zed2i/zed_node/rgb/image_rect_color", Image)
        depth_image = message_filters.Subscriber(
            "/zed2i/zed_node/depth/depth_registered", Image)
        point_cloud = message_filters.Subscriber(
            "/zed2i/zed_node/point_cloud/cloud_registered", PointCloud2)
        message_synchronizer = message_filters.TimeSynchronizer(
            [camera_info, camera_image,depth_image,point_cloud], queue_size=1)
        message_synchronizer.registerCallback(self.image_callback)

    def __open_socket(self):
        host = socket.gethostname()

        self.client_yolov8 = CustomSocket(host, 12301)
        self.client_pe = CustomSocket(host, 12302)
        self.client_ic = CustomSocket(host, 12303)
        self.client_fr = CustomSocket(host, 12304)

        self.client_yolov8.clientConnect()
        self.client_pe.clientConnect()
        self.client_ic.clientConnect()
        self.client_fr.clientConnect()

    def image_callback(self, info_msg: CameraInfo, image_msg: Image, depth_msg: Image, point_cloud_msg: PointCloud2):
        self.image = image_msg
        self.info = info_msg
        self.depth = depth_msg
        self.point_cloud = point_cloud_msg

    def cv_request(self, cv_req: CV_srvRequest):
        error = False
        cv_type = cv_req.cv_type

        image = self.image
        info = self.info
        depth = self.depth
        point_cloud = self.point_cloud

        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                image, desired_encoding="bgr8")
            pull_image = cv2.resize(cv_image, (1280, 736))
        except CvBridgeError as e:
            rospy.logerr(e)
            error = True
        except Exception as e:
            rospy.logerr(e)
            error = True

        if cv_type.type == CV_type.YoloV8_Tracker:
            res = self.client_yolov8.req(pull_image)

        elif cv_type.type == CV_type.TargetTracker_Register:
            msg = self.client_pe.req_with_command(
                pull_image, {"detect_face": True, "detect_pose": False})
            if msg:
                # Find person with biggest area to register
                register_person_id = max(msg, key=lambda x: msg[x]['area'])
                print("Registering ", register_person_id)

                # register_name = input("name: ")
                register_name = "target"
                register_person = msg[register_person_id]
                if "facedim" in register_person:
                    face_dim = register_person["facedim"]
                    face_img = np.array(register_person["faceflatten"].split(
                        ", ")).reshape(face_dim + [3]).astype("uint8")
                    res = self.client_fr.req_with_command(face_img, command={
                        "task": "REGISTER", "name": register_name, "only_face": True, "clear_db": True})

                    self.name_map[register_person_id] = register_name

                else:
                    res = "No face found in ID"
                    # print(res)
            else:
                res = "No person detect"
                # print(res)

        elif cv_type.type == CV_type.TargetTracker_Detect:
            msg = self.client_pe.req_with_command(
                pull_image, {"detect_face": True, "detect_pose": False})
            if msg:
                for person_id, person in msg.items():
                    if "facedim" in person:
                        face_dim = person["facedim"]
                        face_img = np.array(person["faceflatten"].split(
                            ", ")).reshape(face_dim + [3]).astype("uint8")
                        person.pop("faceflatten")

                        if person_id not in self.name_map:
                            fr_res = self.client_fr.req_with_command(
                                pull_image, command={"task": "DETECT", "name": "", "only_face": True})
                            # print(fr_res)
                            if fr_res:
                                self.name_map[person_id] = fr_res["name"]
                            else:
                                self.name_map[person_id] = "unknown"
                        else:
                            if self.name_map[person_id] == "unknown" and self.frame_count % self.frame_count_interval == 0:
                                fr_res = self.client_fr.req_with_command(
                                    pull_image, command={"task": "DETECT", "name": "", "only_face": True})
                                if fr_res:
                                    self.name_map[person_id] = fr_res["name"]

                        person["name"] = self.name_map[person_id]

                    else:
                        if person_id not in self.name_map:
                            person["name"] = "unknown"
                        else:
                            person["name"] = self.name_map[person_id]
            res = msg

        else:
            rospy.logerr("Error CV_type: %s" % (cv_type))
            error = True

        srv_res = CV_srvResponse()
        if not error:
            rospy.loginfo("cv_result: %s" % (res))
            srv_res.sucess = True
            srv_res.result = str(res)
            srv_res.camera_info = info
            srv_res.rgb_image = image
            srv_res.depth_image = depth
            srv_res.pointcloud = point_cloud

        else:
            srv_res.sucess = False

        return srv_res


if __name__ == "__main__":
    rospy.init_node("cv_connect_test")
    cv_con = CVConnector()
    rospy.spin()
