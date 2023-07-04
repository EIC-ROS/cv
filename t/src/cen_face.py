#!/usr/bin/env python  
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import numpy as np
from tf.transformations import quaternion_from_matrix

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



def calculate_box_center(dimensions, positions, r):
    # Unpack dimensions
    length, width, height = dimensions

    # Unpack positions
    pos_front, pos_back, pos_left, pos_right, pos_top, pos_bottom = positions

    # Calculate translation offsets
    dx = length / 2
    dy = width / 2
    dz = height / 2


    # Calculate final rotation matrix
    R = r

    # Calculate initial center point
    initial_center = np.array(pos_front)

    # Apply translations
    translated_center = initial_center + np.array([dx, dy, dz])

    # Apply rotations
    final_center = R.dot(translated_center)

    return final_center


# Example usage
dimensions = (10, 8, 6)
positions = [(0, 0, 5), (0, 0, -5), (-5, 0, 0), (5, 0, 0), (0, 4, 0), (0, -4, 0)]
R =np.array([[0.7071068,-0.7071068,0],
                  [0.7071068,0.7071068,0],
                  [0,0,1]])

center = calculate_box_center(dimensions, positions,R)
print("Center coordinates:", center)

# Plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the box center
ax.scatter(center[0], center[1], center[2], c='r', marker='o')
ax.scatter(0, 0, 5, c='b', marker='o')
ax.scatter(0, 0, -5, c='b', marker='o')
ax.scatter(-5, 0, 0, c='b', marker='o')
ax.scatter(5, 0, 0, c='b', marker='o')
ax.scatter(0, 4, 0, c='b', marker='o')
ax.scatter(0, -4, 0, c='b', marker='o')
# Set plot limits and labels
ax.set_xlim([-10, 10])
ax.set_ylim([-10, 10])
ax.set_zlim([-10, 10])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Show the plot
plt.show()

# class FixedTFBroadcaster:

#     def __init__(self):

#         matrix = np.array([[0.7071068,-0.7071068,0],
#                            [0.7071068,0.7071068,0],
#                            [0,0,1]])
        
        
    
#     def matrix_2_quaternion(self, matrix = np.eye(3)):
#         tranformation_matrix = np.eye(4)
#         tranformation_matrix[:3, :3] = matrix
#         return quaternion_from_matrix(tranformation_matrix)
    



# if __name__ == '__main__':
#     rospy.init_node('fixed_tf2_broadcaster')
#     tfb = FixedTFBroadcaster()

#     rospy.spin()