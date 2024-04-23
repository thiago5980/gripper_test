import numpy as np
import math
import rtde_control, rtde_receive
from scipy.spatial.transform import Rotation

# is_moving을 movel 함수 안에 포함시킬 수 있는지 확인 필요 -> 불필요, RTDE사용으로 내부에서 처리하도록 수정 04/14
# is_moving 함수 제거
# 상위단에서 URControl 호출시 USE_SERVER True 해줘야 unity 연결 가능
# force_mode 함수들은 아직 확인하지 못하였음, movel함수도 확인하지 못함, only movel_rpy2rotvel함수만 확인
# get_joint_pose함수는 업데이트되는것 확인, unity랑 연동해서 확인 필요
# ur_rtde 설치버전 : 1.5.6버전 설치 (pip3 install ur_rtde)

class URControl():
    def __init__(self, UR_IP):
        self.control_interface = rtde_control.RTDEControlInterface(UR_IP)
        self.receive_interface = rtde_receive.RTDEReceiveInterface(UR_IP)
        

    def __del__(self):
        self.control_interface.disconnect()
        self.receive_interface.disconnect()
        
    # option : radian | degree
    def get_joint_pose(self, option='degree'):
        joint_pose = self.receive_interface.getActualQ()
        if option == 'degree':
            joint_pose_degree = np.rad2deg(joint_pose)

            return joint_pose_degree

        elif option == 'radian':
            joint_pose_rad = joint_pose

            return joint_pose_rad

    def get_tcp_pose(self):
        return self.receive_interface.getActualTCPPose()
    
    # def movel(self,x,y,z,rx,ry,rz,acc=0.15,vcc=0.2):
    def movel(self,x,y,z,rx,ry,rz,acc=0.4,vcc=0.3):
        return self.control_interface.moveL([x,y,z,rx,ry,rz], acceleration=acc, speed=vcc)

    def movej(self,j1,j2,j3,j4,j5,j6,acc=0.4,vcc=0.3):
        return self.control_interface.moveJ([j1,j2,j3,j4,j5,j6], acceleration=acc, speed=vcc)


    def movel_force_mode(self,x,y,z,rx,ry,rz,acc=0.15,vcc=0.15,force=30):
        selection_vector = [0,0,1,0,0,0]
        wrench = [0,0,force,0,0,0] # unit: N
        limits = [0.01,0.01,0.06,0.17,0.17,0.17]
        type_of_control = 2
        force_mode_damping = 0.1
        self.control_interface.force_mode_start(selection_vector, wrench, type_of_control, limits, force_mode_damping)
        self.movel(x,y,z,rx,ry,rz,acc,vcc)
        self.control_interface.force_mode_stop()
        

    def movel_force_mode_rpy2rotvec(self,x,y,z,yaw,acc=0.15,vcc=0.15,force=30):
        selection_vector = [0,0,1,0,0,0]
        wrench = [0,0,force,0,0,0] # unit: N
        limits = [0.01,0.01,0.06,0.17,0.17,0.17]
        type_of_control = 2
        force_mode_damping = 0.1
        self.control_interface.force_mode_start(selection_vector, wrench, type_of_control, limits, force_mode_damping)
        self.movel_rpy2rotvec(x,y,z,yaw,acc,vcc)
        self.control_interface.force_mode_stop()

    def movel_rpy2rotvec(self,x,y,z,yaw,acc=0.4,vcc=0.3):
        rot = self.rpy2rotvec(math.pi, 0, yaw)
        tpose = np.hstack(([x,y,z], rot)).tolist()
        self.control_interface.moveL(tpose,acceleration=acc, speed=vcc)

    def rpy2rotvec(self, r,p,y):
        r = Rotation.from_euler('xyz', [r,p,y], degrees=False)
        return r.as_rotvec()

    # def is_goal(self, goal, option='movej'):
    #     if option=='movej':
    #         out = True
    #         while(out):
    #             now = 
