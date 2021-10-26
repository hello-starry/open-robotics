#!/usr/bin/env python
# -*-coding:utf-8 -*-

"""六轴机器人的运动学算法模块.

本模块包含六轴机器人的正运动学、逆运动学
- 参考资料：https://zhuanlan.zhihu.com/p/22304823
"""

from openrobotics.robomath import *


class SixAxisRobot(object):
    def __init__(self, param:list=[10.0, 20.0, 30.0, 40.0, 50.0]) -> None:
        """
        构造SixAxis机器人

        Parameters
        ----------
        param: list
            [  ]
        """
        super().__init__()
        assert len(param) == 5, "参数的数量必须等于5"

        self.set_param(param)

    def set_param(self, param: list):
        self.__param = np.array(param)

    def calc_forward_kinematics(self, joint_pos: list):
        a1, a2, a3, d4, d6  = self.__param
        trans = np.eye(4)
        t1,t2,t3,t4,t5,t6 = np.array(joint_pos)*TO_RAD

        trans[0,0] =  ((sin(t1)*sin(t4) + sin(t2 + t3)*cos(t1)*cos(t4))*cos(t5) + sin(t5)*cos(t1)*cos(t2 + t3))*cos(t6) - (-sin(t1)*cos(t4) + sin(t4)*sin(t2 + t3)*cos(t1))*sin(t6)
        trans[1,0] =  ((sin(t1)*sin(t2 + t3)*cos(t4) - sin(t4)*cos(t1))*cos(t5) + sin(t1)*sin(t5)*cos(t2 + t3))*cos(t6) - (sin(t1)*sin(t4)*sin(t2 + t3) + cos(t1)*cos(t4))*sin(t6)
        trans[2,0] =  (-sin(t5)*sin(t2 + t3) + cos(t4)*cos(t5)*cos(t2 + t3))*cos(t6) - sin(t4)*sin(t6)*cos(t2 + t3)
        trans[0,1] = -((sin(t1)*sin(t4) + sin(t2 + t3)*cos(t1)*cos(t4))*cos(t5) + sin(t5)*cos(t1)*cos(t2 + t3))*sin(t6) - (-sin(t1)*cos(t4) + sin(t4)*sin(t2 + t3)*cos(t1))*cos(t6)
        trans[1,1] =  -((sin(t1)*sin(t2 + t3)*cos(t4) - sin(t4)*cos(t1))*cos(t5) + sin(t1)*sin(t5)*cos(t2 + t3))*sin(t6) - (sin(t1)*sin(t4)*sin(t2 + t3) + cos(t1)*cos(t4))*cos(t6)
        trans[2,1] = -(-sin(t5)*sin(t2 + t3) + cos(t4)*cos(t5)*cos(t2 + t3))*sin(t6) - sin(t4)*cos(t6)*cos(t2 + t3)
        trans[0,2] =  -(sin(t1)*sin(t4) + sin(t2 + t3)*cos(t1)*cos(t4))*sin(t5) + cos(t1)*cos(t5)*cos(t2 + t3)
        trans[1,2] =  -(sin(t1)*sin(t2 + t3)*cos(t4) - sin(t4)*cos(t1))*sin(t5) + sin(t1)*cos(t5)*cos(t2 + t3)
        trans[2,2] =  -sin(t5)*cos(t4)*cos(t2 + t3) - sin(t2 + t3)*cos(t5)
        trans[0,3] =  a1*cos(t1) + a2*sin(t2)*cos(t1) + a3*sin(t2 + t3)*cos(t1) + d4*cos(t1)*cos(t2 + t3) + d6*(-(sin(t1)*sin(t4) + sin(t2 + t3)*cos(t1)*cos(t4))*sin(t5) + cos(t1)*cos(t5)*cos(t2 + t3))
        trans[1,3] =  a1*sin(t1) + a2*sin(t1)*sin(t2) + a3*sin(t1)*sin(t2 + t3) + d4*sin(t1)*cos(t2 + t3) + d6*(-(sin(t1)*sin(t2 + t3)*cos(t4) - sin(t4)*cos(t1))*sin(t5) + sin(t1)*cos(t5)*cos(t2 + t3))
        trans[2,3] =  a2*cos(t2) + a3*cos(t2 + t3) - d4*sin(t2 + t3) - d6*(sin(t5)*cos(t4)*cos(t2 + t3) + sin(t2 + t3)*cos(t5))

        print(trans)
        end_pos = trans_to_pose(trans)

        return end_pos

    def calc_inverse_kinematics(self, end_pos: list):
        end_pos = np.array(end_pos)
        joint_pos = np.zeros(6)
        trans = pose_to_trans(end_pos)
        a1, a2, a3, d4, d6  = self.__param
        nx,ox,ax,px = trans[0,:]
        ny,oy,ay,py = trans[1,:]
        nz,oz,az,pz = trans[2,:]

        joint_pos[0] = atan2(py-ay*d6,px-ax*d6)*TO_DEG
        return joint_pos

six_axis = SixAxisRobot()
end_pos = six_axis.calc_forward_kinematics([10.0, 20.0, 30.0, 10.0, 20.0, 30.0])
print(f"The end_pos of six_axis is {end_pos}")
joint_pos = six_axis.calc_inverse_kinematics(end_pos)
print(f"The joint_pos of six_axis is {joint_pos}")