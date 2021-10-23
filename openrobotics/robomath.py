import numpy as np
import math

np.set_printoptions(suppress=True, precision=4) # 取消科学计数法

ZERO = 1e-6
PI = math.pi
TO_DEG = 180/PI
TO_RAD = PI/180

def cosd(angle):
    return math.cos(angle*TO_RAD)

def sind(angle):
    return math.sin(angle*TO_RAD)

def cos(angle):
    return math.cos(angle)

def sin(angle):
    return math.sin(angle)

def atan2(y, x):
    return math.atan2(y, x)

def sqrt(x):
    return math.sqrt(x)

def rot_x(angle):
    return np.array([[ 1, 0,           0         ],
                     [ 0, cosd(angle), -sind(angle)],
                     [ 0, sind(angle),  cosd(angle)]])

def rot_y(angle):
    return np.array([[ cosd(angle), 0, sind(angle)],
                     [ 0,          1, 0         ],
                     [-sind(angle), 0, cosd(angle)]])

def rot_z(angle):
    return np.array([[ cosd(angle), -sind(angle), 0],
                     [ sind(angle),  cosd(angle), 0],
                     [ 0,           0,          1]])

def rot_inv(rot):
    return np.array(rot).T

def rot_to_euler(rot):
    sy = math.sqrt(rot[0,0] * rot[0,0] +  rot[1,0] * rot[1,0])
    singular = (sy < 1e-6)
    if (not singular):
        x = math.atan2(rot[2,1] , rot[2,2])
        y = math.atan2(-rot[2,0], sy)
        z = math.atan2(rot[1,0], rot[0,0])
    else:
        x = math.atan2(-rot[1,2], rot[1,1])
        y = math.atan2(-rot[2,0], sy)
        z = 0
    return np.array([x, y, z])*TO_DEG

def euler_to_rot(euler):
    return np.dot(rot_z(euler[2]), np.dot(rot_y(euler[1]), rot_x(euler[0])))

def trans_inv(trans):
    pos, rot = decompose_trans(trans)
    rot_i = rot_inv(rot)
    return np.r_[np.c_[rot_i, -np.dot(rot_i, pos)], [[0, 0, 0, 1]]]

def pose_to_trans(pose):
    pos = np.array(pose[0:3]).T
    euler = pose[3:6]
    rot = euler_to_rot(euler)
    trans = compose_trans(pos, rot)
    return trans

def trans_to_pose(trans):
    pos, rot = decompose_trans(trans)
    euler = rot_to_euler(rot)
    return np.r_[pos, euler]

def compose_trans(pos, rot):
    return np.array(np.r_[np.c_[np.array(rot), np.array([pos[0], pos[1], pos[2]])], [[0, 0, 0, 1]]])

def decompose_trans(trans):
    trans = np.array(trans)
    return np.array(trans[0: 3, 3]), np.array(trans[0: 3, 0: 3])
