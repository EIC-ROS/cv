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
import rosparam
from rospkg import RosPack
import colorama

class CVConnector:

    bridge = CvBridge()

    def __init__(self):

        if self.__subscribing_cameara() and self.__open_socket():
            pass
        
      
        rospy.logwarn(f"{colorama.Fore.BLUE}[CV_CONNECTER]: Opening the Server...{colorama.Fore.RESET}")
        self.cv_server = rospy.Service("/CV_connect/req_cv", CV_srv, self.cv_request)
        rospy.loginfo(f"{colorama.Style.BRIGHT}{colorama.Fore.LIGHTCYAN_EX}[CV_CONNECTER]: Ready for request{colorama.Style.RESET_ALL}")

        # PEFR
        self.name_map = dict()
        self.frame_count = 0
        self.frame_count_interval = 10
        self.target_id = -1

    def __subscribing_cameara(self):
        rospy.logwarn(f"{colorama.Fore.BLUE}[CAMERA]: Waiting for camera info...{colorama.Fore.RESET}")
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
        rospy.loginfo("\x1b[38;5;39m"+"[CAMERA]: Ready"+'\x1b[0m')
        return True

    def __open_socket(self):
        host = socket.gethostname()

        self.client_yolov8 = CustomSocket(host, 12301)
        self.client_pe = CustomSocket(host, 12302)
        self.client_ic = CustomSocket(host, 12303)
        self.client_fr = CustomSocket(host, 12304)
        self.client_bea = CustomSocket(host, 12305)

        client_list = [self.client_yolov8,
                       self.client_pe,
                       self.client_ic,
                       self.client_fr,
                       self.client_bea]
        
        check_list = [False]*len(client_list)
        check_times = 0

        rospy.logwarn(f"{colorama.Fore.BLUE}[CV]: Waiting for Socket Server to open...{colorama.Fore.RESET}")

        while not rospy.is_shutdown():
            rospy.sleep(0.5)

            for idx, client in enumerate(client_list):
                if not check_list[idx]:
                    status = client.clientConnect()
                    if status:
                        check_list[idx] = status
                        show = f"{colorama.Fore.WHITE}["
                        for stat in check_list:
                            if stat:
                                show += f"{colorama.Fore.GREEN} True  "
                            else:
                                show += f"{colorama.Fore.RED} False "
                        show += f"{colorama.Fore.WHITE}]"

                        rospy.logwarn(f"{colorama.Fore.RED}[CV]: Status: " + show +f'{colorama.Fore.RESET}')

            if all(check_list):
                rospy.loginfo(f"{colorama.Fore.LIGHTGREEN_EX}[CV]: Status: [    ALL SERVER AVAILABLE    ]{colorama.Fore.RESET}")
                rospy.loginfo("\x1b[38;5;39m"+"[CV]: Ready"+'\x1b[0m')
                break
        return True

        

    def image_callback(self, info_msg: CameraInfo, image_msg: Image, depth_msg: Image, point_cloud_msg: PointCloud2):
        self.image = image_msg
        self.info = info_msg
        self.depth = depth_msg
        self.point_cloud = point_cloud_msg

    def cv_request(self, cv_req: CV_srvRequest):
        error = False
        req_info = ""
        
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

        elif cv_type.type == CV_type.HumanPoseEstimation:
            res = self.client_pe.req_with_command(
                pull_image, {"detect_face": False, "detect_pose": True})

        elif cv_type.type == CV_type.TargetTracker_Register:
            msg = self.client_pe.req_with_command(
                pull_image, {"detect_face": True, "detect_pose": False})
            if msg.get('res'):
                res_pe = msg.get('res')
                # Find person with biggest area to register
                register_person_id = max(res_pe, key=lambda x: res_pe[x]['area'])
                print("Registering ", register_person_id)

                if cv_req.req_info:
                    register_name = cv_req.req_info
                else:
                    register_name = "target"

                # register_name = input("name: ")
                # register_name = "target"
                register_person = res_pe[register_person_id]
                if "facedim" in register_person:
                    face_dim = register_person["facedim"]
                    face_img = np.array(register_person["faceflatten"].split(
                        ", ")).reshape(face_dim + [3]).astype("uint8")
                    res = self.client_fr.req_with_command(face_img, command={
                        "task": "REGISTER", "name": register_name, "only_face": True, "clear_db": True}).get('res')
                    
                    res["register_id"] = register_person_id

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
            if msg.get('res'):
                res_pe = msg.get("res")
                for person_id, person in res_pe.items():
                    if "facedim" in person:
                        face_dim = person["facedim"]
                        face_img = np.array(person["faceflatten"].split(
                            ", ")).reshape(face_dim + [3]).astype("uint8")
                        person.pop("faceflatten")

                        if person_id not in self.name_map:
                            res_fr = self.client_fr.req_with_command(
                                pull_image, command={"task": "DETECT", "name": "", "only_face": True}).get("res")
                            # print(res_fr)
                            if res_fr:
                                self.name_map[person_id] = res_fr["name"]
                            else:
                                self.name_map[person_id] = "unknown"
                        else:
                            if self.name_map[person_id] == "unknown" and self.frame_count % self.frame_count_interval == 0:
                                res_fr = self.client_fr.req_with_command(
                                    pull_image, command={"task": "DETECT", "name": "", "only_face": True}).get('res')
                                if res_fr:
                                    self.name_map[person_id] = res_fr["name"]

                        person["name"] = self.name_map[person_id]

                    else:
                        if person_id not in self.name_map:
                            person["name"] = "unknown"
                        else:
                            person["name"] = self.name_map[person_id]
            res = msg

        elif cv_type.type == CV_type.TargetTrackerMod_Detect:
            msg = self.client_pe.req_with_command(pull_image, {"detect_face": True, "detect_pose": True})

            if msg.get('res'):
                res_pe = msg["res"]
                new_res = dict()
                all_distance = dict()
                if self.target_id in res_pe.keys():
                    for person_id, person in res_pe.items():
                        if "facedim" in person:
                            face_dim = person["facedim"]
                            face_img = np.array(person["faceflatten"].split(
                                ", ")).reshape(face_dim + [3]).astype("uint8")
                            person.pop("faceflatten")

                            if person_id == self.target_id:
                                person["name"] = "target"
                                print(f"{person_id} is a target")
                            else:
                                person["name"] = "unknown"
                        new_res[person_id] = person  

                else:
                    for person_id, person in res_pe.items():
                        print(person["center"])
                        print(person["area"])
                        if "pose" in person:
                            print(person["pose"])
                        if "facedim" in person:
                            face_dim = person["facedim"]
                            face_img = np.array(person["faceflatten"].split(
                                ", ")).reshape(face_dim + [3]).astype("uint8")
                            person.pop("faceflatten")
                            res_fr = self.client_fr.req_with_command(
                                    face_img, command={"task": "DETECT", "name": "", "only_face": True, "one_target": True}).get("res")
                            # print(res_fr)
                            if res_fr.get("distance"):
                                all_distance[res_fr["distance"]] = person_id
                            else:
                                all_distance[999] = person_id

                            person["name"] = "unknown"

                            new_res[person_id] = person

                    min_distance = min(all_distance.keys())
                    if min_distance < 0.3:
                        new_target_id = all_distance[min_distance]
                        new_res[new_target_id]["name"] = "target"
                
                res = msg
                res["res"] = new_res
                
                
        
            elif cv_type.type == CV_type.PoseCaptioning:
                msg = self.client_pe.req_with_command(
                    pull_image, {"detect_face": False, "detect_pose": False})
                if msg.get("res"):
                    res_pe = msg.get("res")

                    # Find person with biggest area to caption
                    caption_person_id = max(res_pe, key=lambda x: res_pe[x]['area'])

                    caption_person = res_pe[caption_person_id]
                    x1, y1, x2, y2 = caption_person["bbox"]
                    cropped_caption_person = pull_image[y1:y2, x1:x2]

                    res_ic = self.client_ic.req_with_command(pull_image, command={"task":"CAPTION"})

                    res = res_ic
                else:
                    res = "No person to caption"

        elif cv_type.type == CV_type.VQA:
            # Visual Question Answering, req_info = string of questions seperated by ,
            if cv_req.req_info:
                questions = cv_req.req_info.split(",")
            else:
                questions = ["Where is it?"]
            res = self.client_ic.req_with_command(pull_image, command={"task":"ASK", "questions":questions})


        elif cv_type.type == CV_type.BAE:
            res = self.client_bea.req(pull_image)

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
