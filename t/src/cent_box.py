#!/usr/bin/env python
import rospy
from cv_connector.srv import CV_srv, CV_srvRequest
from cv_connector.msg import CV_type
from geometry_msgs.msg import Vector3, Point, Quaternion, Pose
from visualization_msgs.msg import MarkerArray, Marker
import math
import numpy as np
from tf.transformations import quaternion_from_matrix
from copy import deepcopy


rospy.init_node("swdw")
pub = rospy.Publisher("visualization_marker_array",MarkerArray,queue_size=1)

rospy.sleep(1)

obj_shape = Vector3()
obj_shape.x = 0.5
obj_shape.y = 0.5
obj_shape.z = 0.5

flat_center_point:Point
# rotation_matrix:np.array

corner = [
          ( obj_shape.x/2,  obj_shape.y/2,  obj_shape.z/2),
          ( obj_shape.x/2,  obj_shape.y/2, -obj_shape.z/2),
          ( obj_shape.x/2, -obj_shape.y/2,  obj_shape.z/2),
          ( obj_shape.x/2, -obj_shape.y/2, -obj_shape.z/2),
          (-obj_shape.x/2,  obj_shape.y/2,  obj_shape.z/2),
          (-obj_shape.x/2,  obj_shape.y/2, -obj_shape.z/2),
          (-obj_shape.x/2, -obj_shape.y/2,  obj_shape.z/2),
          (-obj_shape.x/2, -obj_shape.y/2, -obj_shape.z/2)]

print(corner)

vector_list = np.array(corner)
print(vector_list)

# R_box = np.array([[math.sqrt(2),-math.sqrt(2),0], [math.sqrt(2),math.sqrt(2),0], [0,0,1]])
R_cam = np.array([[0,-1,0],
                  [1,0,0],
                  [0,0,1]])
R_box = -R_cam

norm_vector = np.array((1,0,0),dtype=np.float32)
r = np.matmul(norm_vector,R_box)

print(r)

angle_list = []
angle = np.arccos(np.dot(norm_vector, r) / (np.linalg.norm(norm_vector) * np.linalg.norm(r)))
print(math.degrees(angle))

for i in vector_list:
    angle = np.arccos(np.dot(i, r) / (np.linalg.norm(i) * np.linalg.norm(r)))
    print(math.degrees(angle))
    angle_list.append(angle)

corner_vector = vector_list[angle_list.index(min(angle_list))]
print(corner_vector)

d = np.linalg.norm(corner_vector)
print(d)

tm = np.eye(4)
tm[:3, :3] = R_box

quan_main = quaternion_from_matrix(tm)
print("qm:",quan_main)
cen_res = Point()
cen_res.x = 1
cen_res.y = 1
cen_res.z = 1

result = np.array([cen_res.x,cen_res.y,cen_res.z]) + np.matmul(np.array([d,0,0]),R_cam)
print(result)

box_pose = Pose()
box_pose.position.x = result[0]
box_pose.position.y = result[1]
box_pose.position.z = result[2]
box_pose.orientation.x = quan_main[0]
box_pose.orientation.y = quan_main[1]
box_pose.orientation.z = quan_main[2]
box_pose.orientation.w = quan_main[3]
print("--------------------------------------------------------")


box = Marker()
box.header.frame_id = "map"
box.header.stamp = rospy.Time.now()
box.id = 0
box.type = Marker.CUBE
box.action = Marker.ADD
box.scale = obj_shape
box.color.r = 0
box.color.g = 1
box.color.b = 0
box.color.a = 0.8
box.pose.position = box_pose.position
box.pose.orientation = box_pose.orientation

box_ws = Marker()
box_ws.header.frame_id = "map"
box_ws.header.stamp = rospy.Time.now()
box_ws.id = 3
box_ws.type = Marker.SPHERE
box_ws.action = Marker.ADD
box_ws.scale.x = d*2
box_ws.scale.y = d*2
box_ws.scale.z = d*2
box_ws.color.r = 0
box_ws.color.g = 0
box_ws.color.b = 1
box_ws.color.a = 0.5
box_ws.pose.position = box_pose.position
box_ws.pose.orientation = box_pose.orientation

cen_point = Marker()
cen_point.header.frame_id = "map"
cen_point.header.stamp = rospy.Time.now()
cen_point.id = 1
cen_point.type = Marker.SPHERE
cen_point.action = Marker.ADD
cen_point.scale.x = 0.05
cen_point.scale.y = 0.05
cen_point.scale.z = 0.05
cen_point.color.r = 1
cen_point.color.g = 0
cen_point.color.b = 0
cen_point.color.a = 0.8
cen_point.pose.position = cen_res
cen_point.pose.orientation.z = 1

cen_po = Marker()
cen_po.header.frame_id = "map"
cen_po.header.stamp = rospy.Time.now()
cen_po.id = 1
cen_po.type = Marker.ARROW
cen_po.action = Marker.ADD
cen_po.scale.x = 0.1
cen_po.scale.y = 0.05
cen_po.scale.z = 0.05
cen_po.color.r = 1
cen_po.color.g = 0
cen_po.color.b = 0
cen_po.color.a = 0.8
cen_po.pose.position = cen_res
cen_po.pose.orientation = box_pose.orientation

print("-------------")
print(box)
print("-------------")
print(cen_point)

# a = np.matmul(vector_list,R_box)

# print(a)

# for i in range(9):
#     angle = np.arccos(np.dot(vector_list[i], a[i]) / (np.linalg.norm(vector_list[i]) * np.linalg.norm(a[i])))

#     print(angle)
# ------------------------


# Assume you have the rotation matrix stored in a NumPy array called R_box
# R_box = np.array([[math.sqrt(2),-math.sqrt(2),0], [math.sqrt(2),math.sqrt(2),0], [0,0,1]])  # Replace the ellipses with your rotation matrix

# Calculate unit vectors along the x, y, and z axes
unit_x = np.array([1, 0, 0])
unit_y = np.array([0, 1, 0])
unit_z = np.array([0, 0, 1])

# Multiply unit vectors by the rotation matrix to obtain transformed vectors
transformed_x = np.dot(R_box, unit_x)
transformed_y = np.dot(R_box, unit_y)
transformed_z = np.dot(R_box, unit_z)

# Calculate angles between transformed vectors and original unit vectors
angle_x = np.arccos(np.dot(transformed_x, unit_x) / (np.linalg.norm(transformed_x) * np.linalg.norm(unit_x)))
angle_y = np.arccos(np.dot(transformed_y, unit_y) / (np.linalg.norm(transformed_y) * np.linalg.norm(unit_y)))
angle_z = np.arccos(np.dot(transformed_z, unit_z) / (np.linalg.norm(transformed_z) * np.linalg.norm(unit_z)))



# Convert angles to degrees
angle_x_deg = np.degrees(angle_x)
angle_y_deg = np.degrees(angle_y)
angle_z_deg = np.degrees(angle_z)

# Print the resulting angles
print("Angle between rotation axis and x-axis: {:.2f} degrees".format(angle_x_deg))
print("Angle between rotation axis and y-axis: {:.2f} degrees".format(angle_y_deg))
print("Angle between rotation axis and z-axis: {:.2f} degrees".format(angle_z_deg))


print("--------------------------------------------------------")

pub.publish([box,cen_point,box_ws,cen_po])
rospy.spin()
