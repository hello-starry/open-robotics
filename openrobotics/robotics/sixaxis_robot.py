from openrobotics.robomath import *


class SixAxisRobot(object):
    def __init__(self, param:list=[45.0,442.0,35.0,424.0,80.0]) -> None:
        """
        构造SixAxis机器人

        Parameters
        ----------
        param: list
            [  ]
        """
        super().__init__()
        assert len(param) == 6, "参数的数量必须等于6"

        self.set_param(param)

    def set_param(self, param: list):
        self.__param = np.array(param)

    def calc_forward_kinematics(self, joint_pos: list):
        jp = np.array(joint_pos)
        param = self.__param
        trans = np.eye(4)

        c1 = cosd(jp[0])
        c2 = cosd(jp[1])
        c3 = cosd(jp[2])
        c4 = cosd(jp[3])
        c5 = cosd(jp[4])
        c6 = cosd(jp[5])
        c23 = cosd(jp[1]+jp[2])
        s1 = sind(jp[0])
        s2 = sind(jp[1])
        s3 = sind(jp[2])
        s4 = sind(jp[3])
        s5 = sind(jp[4])
        s6 = sind(jp[5])
        s23 = sind(jp[1]+jp[2])

        trans[0,0] =  s6*(s1*c4-c1*s4*c23)+c6*(c5*(s4*s1+c4*c1*c23)-s5*c1*s23)
        trans[1,0] = -s6*(c1*c4+s1*s4*c23)-c6*(c5*(s4*c1-c4*s1*c23)+s1*s5-s23)


        return end_pos

    def calc_inverse_kinematics(self, end_pos: list):
        end_pos = np.array(end_pos)
        joint_pos = np.zeros(3)
        
        
        return joint_pos
