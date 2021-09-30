from openrobotics.robotics import *

class StewartRobot(Robot):
    def __init__(self, param: list) -> None:
        """
        构造Stewart机器人

        Parameters
        ----------
        param: list
            [ 静平台半径, 动平台半径, 静铰点偏角, 动铰点偏角 ]
        """
        super().__init__()
        assert len(param) == 4, "参数的数量必须等于4"

        self.set_param(param)

    def set_param(self, param: list):
        self._param = np.array(param)

        # 计算静平台铰点位置
        self.ab = np.zeros((3, 6))  # ab
        daf = [0-param[2],   0+param[2],
               120-param[2], 120+param[2],
               240-param[2], 240+param[2]]
        ab_i = np.array([[param[0], 0, 0]]).T
        for i in range(6):
            self.ab[:, i] = np.dot(euler_to_rot([0,0,daf[i]]), ab_i).reshape(3)

        # 计算动平台铰点的相对位置
        self.cd_c = np.zeros((3, 6)) 
        dam = [0-param[3],   0+param[3],
               120-param[3], 120+param[3],
               240-param[3], 240+param[3]]
        cd_i = np.array([[param[1], 0, 0]]).T
        for i in range(6):
            self.cd_c[:, i] = np.dot(euler_to_rot([0,0,dam[i]]), cd_i).reshape(3)

    def calc_forward_kinematics(self, joint_pos:list):
        # newton-raphson 迭代阈值
        tol_fun = 1e-3
        tol_ep = 1e-3
        # 最大迭代次数
        max_iter = 10
        num_iter = 0

        jp = joint_pos
        ab = self.ab
        cd_c = self.cd_c
        
        ep = np.array([0,0,0,0,0,0])
        while num_iter < max_iter:
            R = euler_to_rot(ep[3:6])
            euler = ep.euler()*TO_RAD
            ac = ep[0:3]
            bc = ac-ab
            cd = np.zeros((3,6))
            for i in range(6):
                cd[:,i] = np.dot(R,cd_c[:,i])
            bd = bc+cd
            jp2_t = np.sum(np.square(bd),0) # 杆长的平方
            
            fun = -(jp2_t-np.square(jp))
            sum_fun = np.sum(np.abs(fun))
            if sum_fun < tol_fun:
                break
            
            df_dep = np.zeros((6,6))
            df_dep[:,0:3] = 2*bd.T
            for i in range(6):
                df_dep[i, 5] = 2*(-bc[0,i]*cd[1,i] + bc[1,i]*cd[0,i]) #dfda4
                df_dep[i, 4] = 2*((-bc[0,i]*math.cos(euler[2]) + bc[1,i]*math.sin(euler[2]))*cd[2,i] \
                                - (cd_c[0,i]*math.cos(euler[1]) + cd_c[1,i]*math.sin(euler[1])*math.sin(euler[0]))*bc[2,i]) #dfda5
                df_dep[i, 3] = 2*cd_c[1,i]*(np.dot(bc[:,i],R[:,2])) #dfda
            
            delta_ep = np.linalg.solve(df_dep,fun)
            delta_ep[3:6] = delta_ep[3:6]*TO_DEG
            
            if abs(np.sum(delta_ep)) < tol_ep:
                break  
            epu = ep+delta_ep
            ep = epu
            num_iter = num_iter+1

            ## 记录结构点, 用于绘图
            self.bd = bd

        return ep

    def calc_inverse_kinematics(self, end_pos:list):
        ab = self.ab
        cd_c = self.cd_c
        ep = end_pos
        R = euler_to_rot(ep[3:6])
        ac = ep[0:3]
        cd = np.zeros((3,6))
        for i in range(6):
            cd[:,i] = np.dot(R,cd_c[:,i])
        ad = ac+cd
        bd = ad-ab
        jl = np.sum(np.square(bd),0)**0.5
        
        self.bd = bd
    
        return jl