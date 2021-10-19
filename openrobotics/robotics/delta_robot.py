from openrobotics.robomath import *


class DeltaRobot(object):
    def __init__(self, param:list=[135.0, 300.0, 43.0, 700.0, 59.5, 60.0]) -> None:
        """
        构造Delta机器人

        Parameters
        ----------
        param: list
            [ 静平台半径, 主动臂水平长度, 主动臂垂直长度, 从动臂长度, 动平台半径, 法兰垂直偏移 ]
        """
        super().__init__()
        assert len(param) == 6, "参数的数量必须等于6"

        self.set_param(param)

    def set_param(self, param: list):
        self.__param = np.array(param)

    def calc_forward_kinematics(self, joint_pos: list):
        joint_pos = np.array(joint_pos)
        param = self.__param
        end_pos = np.zeros(3)

        c = (param[1]**2 + param[2]**2)**0.5
        jf = atan2(param[2], param[1])
        xA = np.zeros(3)
        yA = np.zeros(3)
        zA = np.zeros(3)
        w = np.zeros(3)

        for i in range(3):
            xA[i] = (param[0] + c*cos(joint_pos[i]*TO_RAD+jf) -
                     param[4])*cos(120*i*TO_RAD)
            yA[i] = (param[0] + c*cos(joint_pos[i]*TO_RAD+jf) -
                     param[4])*sin(120*i*TO_RAD)
            zA[i] = c*sin(joint_pos[i]*TO_RAD+jf)
            w[i] = xA[i]**2+yA[i]**2+zA[i]**2

        # 核心算法，不修改
        D = (xA[1]-xA[0])*(yA[2]-yA[0])-(xA[2]-xA[0])*(yA[1]-yA[0])
        A1 = -((yA[2]-yA[0])*(zA[1]-zA[0]) -
               (yA[1]-yA[0])*(zA[2]-zA[0]))  # a2 -
        B1 = 0.5*((w[1]-w[0])*(yA[2]-yA[0]) - (w[2]-w[0])*(yA[1]-yA[0]))  # b2
        A2 = (xA[2]-xA[0])*(zA[1]-zA[0]) - (xA[1]-xA[0])*(zA[2]-zA[0])    # a1
        B2 = -0.5*((w[1]-w[0])*(xA[2]-xA[0]) -
                   (w[2]-w[0])*(xA[1]-xA[0]))  # b1 -

        AD = A1**2+A2**2+D**2
        BD = 2*A1*B1 - 2*A1*xA[0]*D + 2*A2*B2 - 2*A2*yA[0]*D - 2*zA[0]*D**2
        CD = B1**2 - 2*B1*xA[0]*D + xA[0]**2*D**2 + B2**2 - 2*B2 * \
            yA[0]*D - param[3]**2*D**2 + yA[0]**2*D**2 + zA[0]**2*D**2
        delta = BD**2-4*AD*CD

        assert delta >= 0, "正运动学错误"

        zD = 0.5*(-BD+delta**0.5)/AD
        end_pos[2] = zD + param[5]
        end_pos[0] = (A1*zD + B1)/D
        end_pos[1] = (A2*zD + B2)/D

        return end_pos

    def calc_inverse_kinematics(self, end_pos: list):
        end_pos = np.array(end_pos)
        joint_pos = np.zeros(3)
        temp_pos = np.zeros(3)
        
        for i in range(3):
            temp_pos[0] = end_pos[0]*cos(120*i*TO_RAD) + end_pos[1]*sin(120*i*TO_RAD)
            temp_pos[1] = end_pos[1]*cos(120*i*TO_RAD) - end_pos[0]*sin(120*i*TO_RAD)
            temp_pos[2] = end_pos[2]
            angle = self.__calc_inverse_kinematics_sub(temp_pos)
            joint_pos[i] = angle

        return joint_pos

    def __calc_inverse_kinematics_sub(self, pos):
        param = self.__param
        xj = param[0]
        xc, zc = pos[0] + param[4], pos[2] - param[5]
        a3p = (param[3]**2 - pos[1]**2)**0.5
        c = (param[1]**2 + param[2]**2)**0.5

        assert abs(zc) > ZERO, "逆运动学错误"

        M = (xj-xc)/zc
        N = (-a3p**2+c**2+xc**2-xj**2+zc**2)/(2*zc)
        D = 4*M**2*c**2 - 4*M**2*xj**2 - 8*M*N*xj - 4*N**2 + 4*c**2

        assert D > 0, "逆运动学错误"
        
        xA = (0.5*sqrt(D)+xj-M*N)/(M**2+1) # x始终取较大值
        zA = M*xA+N

        Angle = (atan2(zA, xA-xj)-atan2(param[2], param[1])) * TO_DEG
        
        return Angle
